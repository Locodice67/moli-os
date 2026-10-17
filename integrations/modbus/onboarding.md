# Configurer un appareil Modbus

Une instance par appareil (ou par esclave). Exemple, un capteur de
température en registre d'entrée 4 (int16, ×0.5 − 30) :

```toml
[[driver]]
id = "piscine"
kind = "modbus"
[driver.options]
host = "192.168.1.50"
port = 4196
unit_id = 1
connection = "rtu_over_tcp"
name = "Piscine"

[[driver.options.point]]
key = "temperature"
label = "Température"
kind = { type = "numeric", min = 0, max = 60 }
register = "input"
address = 4
data_type = "int16"
scale = 0.5
offset = -30
precision = 1
unit = "°C"
```

Champs d'un point : `key` (stable), `label`, `kind` (`binary`, `numeric` avec
`min`/`max`/`step`, `enum` avec `values`), `register` (`coil`, `discrete_input`,
`input`, `holding`), `address`, `data_type` (`int16`/`uint16`), `scale`,
`offset`, `precision`, `unit`, `semantic`, `write` (pour une bobine ou un
holding écrivable) et `options` (enum : texte → valeur brute).
