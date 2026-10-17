//! Modbus TCP and RTU-over-TCP, declaratively: the instance's options list
//! the registers to read and write. One device per instance, one point per
//! register. The driver does not discover anything: a person gives the host,
//! port and unit id of the gateway, then the register map.
//!
//! Two framings are covered:
//! - `tcp` — Modbus TCP (MBAP header, no CRC), the Geco PAC and Ecoforest;
//! - `rtu_over_tcp` — raw RTU frames (address + PDU + CRC16) over TCP, the
//!   Pool Technologie electrolyser's gateway.
//!
//! Functions used: FC01/FC02 (coils, discrete inputs), FC03/FC04 (holding,
//! input registers), FC05 (write coil), FC06 (write holding register).

use std::collections::BTreeMap;
use std::time::Duration;

use anyhow::{Context as _, bail};
use moli_core::{Access, Device, DeviceId, Kind, PointSpec, Semantic, Unit, Value};
use moli_runtime::{BoxFuture, CommandRequest, Driver, DriverCtx};
use serde::Deserialize;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

const READ_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// The gateway's address (a Modbus TCP server, or an RTU-over-TCP bridge).
    pub host: String,
    #[serde(default = "default_port")]
    pub port: u16,
    #[serde(default = "default_unit")]
    pub unit_id: u8,
    #[serde(default)]
    pub connection: Connection,
    #[serde(default = "default_scan")]
    pub scan_interval: u64,
    /// Display name; a generic one when absent.
    #[serde(default)]
    pub name: Option<String>,
    pub points: Vec<Point>,
}

#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Connection {
    #[default]
    Tcp,
    RtuOverTcp,
}

/// The four register areas Modbus addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegType {
    Coil,
    DiscreteInput,
    Input,
    Holding,
}

/// How a 16-bit word is read (signed or unsigned).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataType {
    #[default]
    Int16,
    UInt16,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Point {
    pub key: String,
    pub label: String,
    pub kind: Kind,
    pub register: RegType,
    pub address: u16,
    #[serde(default)]
    pub data_type: DataType,
    #[serde(default = "one")]
    pub scale: f64,
    #[serde(default)]
    pub offset: f64,
    #[serde(default)]
    pub precision: u8,
    #[serde(default)]
    pub unit: Option<Unit>,
    #[serde(default)]
    pub semantic: Option<Semantic>,
    /// A writable point must sit in a coil or a holding register.
    #[serde(default)]
    pub write: bool,
    /// Enum or binary points: option text → raw value.
    #[serde(default)]
    pub options: BTreeMap<String, u16>,
}

const fn default_port() -> u16 {
    502
}

const fn default_unit() -> u8 {
    1
}

const fn default_scan() -> u64 {
    30
}

const fn one() -> f64 {
    1.0
}

/// The device Moli runs for the house itself, one per instance.
#[derive(Debug)]
pub struct Modbus {
    config: Config,
}

impl Modbus {
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self { config }
    }
}

impl Driver for Modbus {
    fn kind(&self) -> &'static str {
        "modbus"
    }

    fn run<'a>(&'a self, ctx: &'a mut DriverCtx) -> BoxFuture<'a, anyhow::Result<()>> {
        Box::pin(run(self, ctx))
    }
}

/// Modbus CRC16 (poly 0xA001, init 0xFFFF), little-endian on the wire.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 == 0 {
                crc >> 1
            } else {
                (crc >> 1) ^ 0xA001
            };
        }
    }
    crc
}

/// A Modbus TCP frame: MBAP header (transaction, protocol, length, unit) + PDU.
fn frame_tcp(txn: u16, unit_id: u8, pdu: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(7 + pdu.len());
    frame.extend_from_slice(&txn.to_be_bytes());
    frame.extend_from_slice(&[0, 0]);
    #[allow(clippy::cast_possible_truncation)]
    let len = (pdu.len() + 1) as u16;
    frame.extend_from_slice(&len.to_be_bytes());
    frame.push(unit_id);
    frame.extend_from_slice(pdu);
    frame
}

/// A Modbus RTU-over-TCP frame: address + PDU + CRC16, no MBAP header.
fn frame_rtu(unit_id: u8, pdu: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(1 + pdu.len() + 2);
    frame.push(unit_id);
    frame.extend_from_slice(pdu);
    let crc = crc16(&frame);
    frame.extend_from_slice(&crc.to_le_bytes());
    frame
}

struct Client {
    stream: TcpStream,
    connection: Connection,
    unit_id: u8,
    txn: u16,
}

impl Client {
    async fn connect(
        host: &str,
        port: u16,
        connection: Connection,
        unit_id: u8,
    ) -> anyhow::Result<Self> {
        let stream = TcpStream::connect((host, port))
            .await
            .with_context(|| format!("modbus {host}:{port} unreachable"))?;
        Ok(Self {
            stream,
            connection,
            unit_id,
            txn: 0,
        })
    }

    /// Sends one request and returns the response data after the function
    /// code. The function code is checked; exceptions are raised.
    async fn transact(
        &mut self,
        func: u8,
        addr: u16,
        count: u16,
        value: Option<u16>,
    ) -> anyhow::Result<Vec<u8>> {
        let mut pdu = vec![func];
        pdu.extend_from_slice(&addr.to_be_bytes());
        match func {
            0x01..=0x04 => pdu.extend_from_slice(&count.to_be_bytes()),
            0x05 | 0x06 => pdu.extend_from_slice(&value.unwrap_or(0).to_be_bytes()),
            _ => bail!("unsupported function {func:#04x}"),
        }
        let frame = self.frame(&pdu);
        self.stream.write_all(&frame).await?;
        let response = self.read_response().await?;
        let echoed = response.first().copied().unwrap_or(0);
        if echoed == func {
            Ok(response[1..].to_vec())
        } else if echoed == func | 0x80 {
            let code = response.get(1).copied().unwrap_or(0);
            bail!("modbus exception {code:#04x} on function {func:#04x}");
        } else {
            bail!("unexpected function {echoed:#04x} (asked {func:#04x})");
        }
    }

    fn frame(&mut self, pdu: &[u8]) -> Vec<u8> {
        match self.connection {
            Connection::Tcp => {
                self.txn = self.txn.wrapping_add(1);
                frame_tcp(self.txn, self.unit_id, pdu)
            }
            Connection::RtuOverTcp => frame_rtu(self.unit_id, pdu),
        }
    }

    /// Reads one response, returning its PDU (function code + data). For RTU
    /// the CRC is checked, so a desynced stream is caught.
    async fn read_response(&mut self) -> anyhow::Result<Vec<u8>> {
        match self.connection {
            Connection::Tcp => {
                let mut header = [0u8; 7];
                read_exact(&mut self.stream, &mut header).await?;
                let len = usize::from(u16::from_be_bytes([header[4], header[5]]));
                let mut pdu = vec![0u8; len.saturating_sub(1)];
                read_exact(&mut self.stream, &mut pdu).await?;
                Ok(pdu)
            }
            Connection::RtuOverTcp => {
                let mut head = [0u8; 2];
                read_exact(&mut self.stream, &mut head).await?;
                let echoed = head[1];
                let tail = if echoed & 0x80 != 0 {
                    read_bytes(&mut self.stream, 3).await?
                } else if matches!(echoed, 0x01..=0x04) {
                    let n = usize::from(read_bytes(&mut self.stream, 1).await?[0]);
                    read_bytes(&mut self.stream, n + 2).await?
                } else {
                    read_bytes(&mut self.stream, 6).await?
                };
                let (data, crc) = tail.split_at(tail.len().saturating_sub(2));
                let Some(&[lo, hi]) = crc.get(..2) else {
                    bail!("short modbus response");
                };
                let expected = u16::from_le_bytes([lo, hi]);
                let mut full = vec![head[0], echoed];
                full.extend_from_slice(data);
                if crc16(&full) != expected {
                    bail!("modbus CRC mismatch");
                }
                Ok(full[1..].to_vec())
            }
        }
    }

    async fn read_bits(&mut self, addr: u16, is_input: bool) -> anyhow::Result<bool> {
        let func = if is_input { 0x02 } else { 0x01 };
        let data = self.transact(func, addr, 1, None).await?;
        Ok(data.get(1).copied().is_some_and(|b| b & 1 != 0))
    }

    async fn read_word(&mut self, addr: u16, is_input: bool) -> anyhow::Result<u16> {
        let func = if is_input { 0x04 } else { 0x03 };
        let data = self.transact(func, addr, 1, None).await?;
        match data.get(1..) {
            Some([hi, lo, ..]) => Ok(u16::from_be_bytes([*hi, *lo])),
            _ => bail!("short register answer"),
        }
    }

    async fn write_coil(&mut self, addr: u16, on: bool) -> anyhow::Result<()> {
        self.transact(0x05, addr, 0, Some(if on { 0xFF00 } else { 0x0000 }))
            .await?;
        Ok(())
    }

    async fn write_word(&mut self, addr: u16, value: u16) -> anyhow::Result<()> {
        self.transact(0x06, addr, 0, Some(value)).await?;
        Ok(())
    }
}

async fn read_exact(stream: &mut TcpStream, buf: &mut [u8]) -> anyhow::Result<()> {
    tokio::time::timeout(READ_TIMEOUT, stream.read_exact(buf))
        .await
        .context("modbus read timed out")?
        .context("modbus read failed")?;
    Ok(())
}

async fn read_bytes(stream: &mut TcpStream, n: usize) -> anyhow::Result<Vec<u8>> {
    let mut buf = vec![0u8; n];
    read_exact(stream, &mut buf).await?;
    Ok(buf)
}

fn spec(point: &Point) -> PointSpec {
    let semantic = point
        .semantic
        .unwrap_or_else(|| Semantic::infer(&point.key, point.unit.as_ref()));
    PointSpec {
        key: point.key.clone().into(),
        label: point.label.clone().into(),
        kind: point.kind.clone(),
        access: Access {
            read: true,
            write: point.write,
        },
        unit: point.unit.clone(),
        semantic,
    }
}

fn device(config: &Config, ctx: &DriverCtx, id: &DeviceId) -> Device {
    let name = config
        .name
        .clone()
        .unwrap_or_else(|| moli_i18n::tr!("pilotes.modbus.appareil"));
    Device {
        id: id.clone(),
        instance: ctx.instance().clone(),
        native_name: name.into(),
        manufacturer: None,
        model: None,
        description: None,
        native_room: None,
        members: Vec::new(),
        points: config.points.iter().map(spec).collect(),
    }
}

/// A 16-bit word, as a value (signed, scaled, mapped). `None` on a sentinel.
fn to_value(point: &Point, word: u16) -> Option<Value> {
    match point.kind {
        Kind::Binary => Some(Value::Bool(word != 0)),
        Kind::Numeric { .. } => {
            let sentinel = match point.data_type {
                DataType::Int16 => word == 0x8000,
                DataType::UInt16 => word == 0xFFFF,
            };
            if sentinel {
                return None;
            }
            let number = if point.data_type == DataType::Int16 {
                f64::from(i16::from_be_bytes(word.to_be_bytes()))
            } else {
                f64::from(word)
            };
            Some(Value::Float(round(
                number * point.scale + point.offset,
                point.precision,
            )))
        }
        Kind::Enum { .. } => point
            .options
            .iter()
            .find(|(_, raw)| **raw == word)
            .map(|(text, _)| Value::Text(text.clone().into())),
        Kind::Text => None,
    }
}

/// A bit (coil or discrete input), as a value.
fn bit_to_value(point: &Point, bit: bool) -> Value {
    if point.kind == Kind::Binary {
        Value::Bool(bit)
    } else {
        Value::Float(f64::from(u8::from(bit)))
    }
}

/// A value, as the raw word to write into a holding register.
fn to_word(point: &Point, value: &Value) -> anyhow::Result<u16> {
    match point.kind {
        Kind::Binary => Ok(u16::from(matches!(value, Value::Bool(true)))),
        Kind::Numeric { .. } => {
            let n = value.as_f64().context("expected a number")?;
            Ok(clamp_word(
                (n - point.offset) / point.scale,
                point.data_type,
            ))
        }
        Kind::Enum { .. } => {
            let text = match value {
                Value::Text(t) => t.as_ref(),
                _ => bail!("expected text"),
            };
            point.options.get(text).copied().context("unknown option")
        }
        Kind::Text => bail!("text points are read-only"),
    }
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn clamp_word(raw: f64, data_type: DataType) -> u16 {
    let (min, max) = if data_type == DataType::Int16 {
        (-32_768.0, 32_767.0)
    } else {
        (0.0, 65_535.0)
    };
    raw.round().clamp(min, max) as u16
}

fn round(value: f64, precision: u8) -> f64 {
    let factor = 10f64.powi(i32::from(precision));
    (value * factor).round() / factor
}

async fn read_point(client: &mut Client, point: &Point) -> anyhow::Result<Option<Value>> {
    match point.register {
        RegType::Coil | RegType::DiscreteInput => {
            let bit = client
                .read_bits(point.address, point.register == RegType::DiscreteInput)
                .await?;
            Ok(Some(bit_to_value(point, bit)))
        }
        RegType::Input | RegType::Holding => {
            let word = client
                .read_word(point.address, point.register == RegType::Input)
                .await?;
            Ok(to_value(point, word))
        }
    }
}

async fn write_point(client: &mut Client, point: &Point, value: &Value) -> anyhow::Result<()> {
    match point.register {
        RegType::Coil => {
            let on = match value {
                Value::Bool(b) => *b,
                _ => bail!("expected a boolean"),
            };
            client.write_coil(point.address, on).await
        }
        RegType::Holding => {
            client
                .write_word(point.address, to_word(point, value)?)
                .await
        }
        RegType::Input | RegType::DiscreteInput => {
            bail!("{:?} registers are read-only", point.register)
        }
    }
}

async fn run(m: &Modbus, ctx: &mut DriverCtx) -> anyhow::Result<()> {
    let config = &m.config;
    let id = ctx.device_id("modbus");
    ctx.upsert_device(device(config, ctx, &id));

    let mut client = connect(config).await;
    if client.is_some() {
        ctx.ready();
    } else {
        ctx.set_availability(&id, false);
    }

    let mut tick = tokio::time::interval(Duration::from_secs(config.scan_interval.max(1)));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            command = ctx.next_command() => {
                let Some(command) = command else { return Ok(()) };
                let result = handle_command(config, &mut client, &command).await;
                if result.is_ok() {
                    ctx.set_state(&id, &command.key, command.value.clone());
                }
                command.reply(result);
            }
            _ = tick.tick() => {
                let Some(c) = client.as_mut() else {
                    client = connect(config).await;
                    continue;
                };
                match poll(config, c).await {
                    Ok(values) => {
                        ctx.ready();
                        ctx.set_availability(&id, true);
                        for (key, value) in values {
                            if let Some(v) = value {
                                ctx.set_state(&id, &key, v);
                            }
                        }
                    }
                    Err(e) => {
                        ctx.set_availability(&id, false);
                        tracing::warn!(
                            instance = %ctx.instance(),
                            error = %format!("{e:#}"),
                            "modbus unreachable"
                        );
                        client = None;
                    }
                }
            }
        }
    }
}

async fn connect(config: &Config) -> Option<Client> {
    Client::connect(&config.host, config.port, config.connection, config.unit_id)
        .await
        .ok()
}

async fn poll(
    config: &Config,
    client: &mut Client,
) -> anyhow::Result<Vec<(String, Option<Value>)>> {
    let mut values = Vec::with_capacity(config.points.len());
    for point in &config.points {
        let value = read_point(client, point).await?;
        values.push((point.key.clone(), value));
    }
    Ok(values)
}

async fn handle_command(
    config: &Config,
    client: &mut Option<Client>,
    command: &CommandRequest,
) -> Result<(), String> {
    let Some(point) = config.points.iter().find(|p| p.key.as_str() == command.key.as_ref())
    else {
        return Err(format!("unknown point {}", command.key));
    };
    if !point.write {
        return Err(format!("{} is read-only", command.key));
    }
    let Some(c) = client.as_mut() else {
        return Err("unreachable".into());
    };
    write_point(c, point, &command.value)
        .await
        .map_err(|e| format!("{e:#}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(kind: Kind, register: RegType, address: u16, data_type: DataType) -> Point {
        Point {
            key: "p".into(),
            label: "P".into(),
            kind,
            register,
            address,
            data_type,
            scale: 1.0,
            offset: 0.0,
            precision: 0,
            unit: None,
            semantic: None,
            write: false,
            options: BTreeMap::new(),
        }
    }

    #[test]
    fn crc16_matches_the_modbus_reference() {
        assert_eq!(crc16(b"123456789"), 0x4B37);
    }

    #[test]
    fn tcp_frame_carries_the_mbap_header() {
        let frame = frame_tcp(0x0001, 17, &[0x03, 0x00, 0x01, 0x00, 0x01]);
        assert_eq!(
            frame,
            vec![
                0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 17, 0x03, 0x00, 0x01, 0x00, 0x01
            ]
        );
    }

    #[test]
    fn rtu_frame_crc_is_self_consistent() {
        let frame = frame_rtu(1, &[0x03, 0x00, 0x01, 0x00, 0x01]);
        assert_eq!(frame.len(), 8);
        assert_eq!(crc16(&frame), 0);
    }

    #[test]
    fn scale_and_offset_apply_and_sentinels_are_none() {
        let p = Point {
            scale: 0.5,
            offset: -30.0,
            precision: 1,
            unit: Some(Unit::Celsius),
            ..point(
                Kind::Numeric {
                    min: None,
                    max: None,
                    step: None,
                },
                RegType::Input,
                4,
                DataType::Int16,
            )
        };
        assert_eq!(to_value(&p, 100), Some(Value::Float(20.0)));
        assert_eq!(to_value(&p, 0x8000), None);
    }

    #[test]
    fn enum_mapping_round_trips() {
        let p = Point {
            kind: Kind::Enum {
                values: vec!["auto".into(), "heat".into(), "cool".into()],
            },
            register: RegType::Holding,
            write: true,
            options: BTreeMap::from([("auto".into(), 0), ("heat".into(), 1), ("cool".into(), 2)]),
            ..point(Kind::Binary, RegType::Holding, 0, DataType::UInt16)
        };
        assert_eq!(to_value(&p, 1), Some(Value::from("heat")));
        assert_eq!(to_word(&p, &Value::from("cool")).unwrap(), 2);
    }

    #[test]
    fn numeric_write_applies_scale_and_clamps() {
        let p = Point {
            kind: Kind::Numeric {
                min: Some(18.0),
                max: Some(32.0),
                step: Some(0.5),
            },
            register: RegType::Holding,
            scale: 0.5,
            offset: -30.0,
            write: true,
            ..point(Kind::Binary, RegType::Holding, 3, DataType::Int16)
        };
        assert_eq!(to_word(&p, &Value::Float(25.0)).unwrap(), 110);
        assert_eq!(to_word(&p, &Value::Float(200.0)).unwrap(), 32_767);
    }
}
