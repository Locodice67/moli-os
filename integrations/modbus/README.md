# Appareil Modbus

Un driver générique pour tout appareil parlant Modbus **TCP** ou **RTU sur
TCP** (une passerelle série↔TCP). La carte de registres est déclarative :
chaque point (registre, échelle, unité) est un bloc dans `[[driver]]` de
`moli.toml`, sans recompiler.

Framings couverts :

- `connection = "tcp"` — Modbus TCP (en-tête MBAP, sans CRC) ;
- `connection = "rtu_over_tcp"` — trames RTU brutes (adresse + PDU + CRC16).

Fonctions : FC01/FC02 (bobines, entrées TOR), FC03/FC04 (holdings, entrées),
FC05 (écrire bobine), FC06 (écrire holding). Valeurs `int16` (par défaut) ou
`uint16`, avec `scale` et `offset` par point, et `precision` pour l'arrondi.
