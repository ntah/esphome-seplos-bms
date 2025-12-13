base : https://github.com/syssi/esphome-seplos-bms/.
Tujuannya hanya agar bisa membaca bms zte zxdc48 fb101, zte zxdc48 fb100c1 dan Shoto mcb. saya hanya memiliki bms tersebut jadi hanya itu yang bisa saya test.

![alt text](https://github.com/ntah/esphome-seplos-bms/blob/main/images/Screenshotwirring.png?raw=true)
```yaml
esphome:
  name: zte-bms
  friendly_name: zte-bms

esp32:
  board: esp32dev
  framework:
    type: esp-idf  

# Enable logging
logger:
  baud_rate: 0
  level: NONE

# Enable Home Assistant API
api:
  encryption:
    key: "key="

ota:
  - platform: esphome
    password: "your default ota password"

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # Enable fallback hotspot (captive portal) in case wifi connection fails
  ap:
    ssid: "Zte-Bms Fallback Hotspot"
    password: "passwprd"

captive_portal:

external_components:
  - source: github://ntah/esphome-seplos-bms@main
    refresh: 0s

uart:
  id: uart_bus1
  baud_rate: 9600
  tx_pin: GPIO17
  rx_pin: GPIO16
  rx_timeout: 50
  rx_buffer_size: 512  
  # The increased RX buffer size is important because
  # the full BMS response must fit into the buffer
  debug:
    direction: BOTH
    dummy_receiver: false
    after:
      delimiter: "\r"
    sequence:
      - lambda: UARTDebug::log_string(direction, bytes);

seplos_modbus:
  id: modbus0
  uart_id: uart_bus1
  rx_timeout: 500ms
  flow_control_pin: GPIO4

seplos_bms:
  - id: battery_bank1
    address: 0x02
    seplos_modbus_id: modbus0
    protocol_version: 0x21
    update_interval: 10s
    tipe: fb101 #ganti dengan fb101 atau fb100c1 atau shotomcb

  - id: battery_bank2
    address: 0x03
    protocol_version: 0x21
    seplos_modbus_id: modbus0
    update_interval: 10s  
    tipe: fb100c1  #ganti dengan fb101 atau fb100c1 atau shotomcb

sensor:
  - platform: seplos_bms
    seplos_bms_id: battery_bank1
    charging_cycles:
      name: "bank 1 charging cycles"    
    min_cell_voltage:
      name: "bank 1 min cell voltage"
    max_cell_voltage:
      name: "bank 1 max cell voltage"
    delta_cell_voltage:
      name: "bank 1 delta cell voltage"
    average_cell_voltage:
      name: "bank 1 average cell voltage"
    cell_voltage_1:
      name: "bank 1 cell voltage 1"
    cell_voltage_2:
      name: "bank 1 cell voltage 2"
    cell_voltage_3:
      name: "bank 1 cell voltage 3"
    cell_voltage_4:
      name: "bank 1 cell voltage 4"
    cell_voltage_5:
      name: "bank 1 cell voltage 5"
    cell_voltage_6:
      name: "bank 1 cell voltage 6"
    cell_voltage_7:
      name: "bank 1 cell voltage 7"
    cell_voltage_8:
      name: "bank 1 cell voltage 8"
    cell_voltage_9:
      name: "bank 1 cell voltage 9"
    cell_voltage_10:
      name: "bank 1 cell voltage 10"
    cell_voltage_11:
      name: "bank 1 cell voltage 11"
    cell_voltage_12:
      name: "bank 1 cell voltage 12"
    cell_voltage_13:
      name: "bank 1 cell voltage 13"
    cell_voltage_14:
      name: "bank 1 cell voltage 14"
    cell_voltage_15:
      name: "bank 1 cell voltage 15"
    state_of_health:
      name: "bank 1 SOH"
    state_of_charge:
      name: "bank 1 SOC"
    total_voltage:
      id: bank1_total_voltage
      name: "bank 1 total voltage"
    current:
      name: "bank 1 current"
    power:
      name: "bank 1 power"
    charging_power:
      name: "bank 1 charging power"
    discharging_power:
      name: "bank 1 discharging power"    
    battery_capacity:
      id: bank1_battery_capacity
      name: "bank 1 battery capacity"   
    temperature_1:
      id: bank1_temperature_1
    temperature_2:
      id: bank1_temperature_2
    temperature_3:
      id: bank1_temperature_3
    residual_capacity:
      id: bank1_residual_capacity      

  - platform: seplos_bms
    seplos_bms_id: battery_bank2
    charging_cycles:
      name: "bank 2 charging cycles"    
    min_cell_voltage:
      name: "bank 2 min cell voltage"
    max_cell_voltage:
      name: "bank 2 max cell voltage"
    delta_cell_voltage:
      name: "bank 2 delta cell voltage"
    average_cell_voltage:
      name: "bank 2 average cell voltage"
    cell_voltage_1:
      name: "bank 2 cell voltage 1"
    cell_voltage_2:
      name: "bank 2 cell voltage 2"
    cell_voltage_3:
      name: "bank 2 cell voltage 3"
    cell_voltage_4:
      name: "bank 2 cell voltage 4"
    cell_voltage_5:
      name: "bank 2 cell voltage 5"
    cell_voltage_6:
      name: "bank 2 cell voltage 6"
    cell_voltage_7:
      name: "bank 2 cell voltage 7"
    cell_voltage_8:
      name: "bank 2 cell voltage 8"
    cell_voltage_9:
      name: "bank 2 cell voltage 9"
    cell_voltage_10:
      name: "bank 2 cell voltage 10"
    cell_voltage_11:
      name: "bank 2 cell voltage 11"
    cell_voltage_12:
      name: "bank 2 cell voltage 12"
    cell_voltage_13:
      name: "bank 2 cell voltage 13"
    cell_voltage_14:
      name: "bank 2 cell voltage 14"
    cell_voltage_15:
      name: "bank 2 cell voltage 15"
    state_of_health:
      name: "bank 2 SOH"
    state_of_charge:
      name: "bank 2 SOC"
    total_voltage:
      id: bank2_total_voltage
      name: "bank 2 total voltage"
    current:
      name: "bank 2 current"
    power:
      name: "bank 2 power"
    charging_power:
      name: "bank 2 charging power"
    discharging_power:
      name: "bank 2 discharging power"     
    battery_capacity:
      id: bank2_battery_capacity
      name: "bank 2 battery capacity"         
    temperature_1:
      id: bank2_temperature_1
    temperature_2:
      id: bank2_temperature_2
    temperature_3:
      id: bank2_temperature_3
    residual_capacity:
      id: bank2_residual_capacity
      name: "bank 2 residual capacity"  
```
# esphome-seplos-bms

ESPHome component to monitor a Seplos BMS via UART, RS485 or BLE

## Supported devices

* 1101-SP05 (reported by [@JacquesdeBruyn](https://github.com/syssi/esphome-seplos-bms/issues/37))
* 1101-SP15 (reported by [@NosIreland](https://github.com/syssi/esphome-seplos-bms/issues/1))
* 1101-SP16 (reported by [@atze09](https://github.com/syssi/esphome-seplos-bms/issues/28))
* 1101-ZH26 (reported by [@faizan-elite](https://github.com/syssi/esphome-seplos-bms/issues/2))
* 1101-MZ02 (reported by [@fajera81](https://github.com/syssi/esphome-seplos-bms/discussions/33))
* 1101-SP76 (reported by [@bagges](https://github.com/syssi/esphome-seplos-bms/issues/46))
* 1101-SP101, PUSUNG-135 (reported by [@manznOnly](https://github.com/syssi/esphome-seplos-bms/discussions/50#discussioncomment-5630209))
* 1101-10E-SP76-16S (reported by [@tobox](https://github.com/syssi/esphome-seplos-bms/discussions/42))
* 1101-10E-JK06-16S (Apex 48200, Apex BMS 48V200A, reported by [@Pho3niX90](https://github.com/syssi/esphome-seplos-bms/issues/74))
* Boqiang BMS007-LD43-16S-HW (reported by [@xdilian](https://github.com/syssi/esphome-seplos-bms/discussions/43)) requires custom settings
  ```
  protocol_version: 0x26
  override_pack: 1
  ```
* Boqiang BMS001-HS01-15S (reported by [@xdilian](https://github.com/syssi/esphome-seplos-bms/discussions/43)) requires custom settings
  ```
  protocol_version: 0x26
  override_pack: 1
  override_cell_count: 10
  ```
* Seplos BMS V3.0 Type C, B-48200-C (BMS16S200A-SP05B, FW 1.3, [@Goaheadz](https://github.com/syssi/esphome-seplos-bms/discussions/98)) using [esp8266-seplos-v3-example.yaml](esp32-seplos-v3-example.yaml)

## Untested devices

* EMU10xx
* 11XX Series

## Schematics

```
                  RS485                      UART
┌────────────┐              ┌──────────┐                ┌─────────┐
│            │              │          │<----- RX ----->│         │
│   Seplos   │<-----B- ---->│  RS485   │<----- TX ----->│ ESP32/  │
│    BMS     │<---- A+ ---->│  to TTL  │<----- GND ---->│ ESP8266 │
│            │<--- GND ---->│  module  │<----- 3.3V --->│         │<-- VCC
│            │              │          │                │         │<-- GND
└────────────┘              └──────────┘                └─────────┘

```

Please make sure to power the RS485 module with 3.3V because it affects the TTL (transistor-transistor logic) voltage between RS485 module and ESP.

### RJ45 jack

|  Pin  | Purpose | RS485-to-TTL pin | Color T-568B |
|:-----:|:--------|:-----------------|--------------|
| **1** | **B-**  | **B-**           | Orange-White |
| **2** | **A+**  | **A+**           | Orange       |
| **3** | **GND** | **GND**          | Green-White  |
|   4   | NC      |                  |              |
|   5   | NC      |                  |              |
|   6   | GND     |                  |              |
|   7   | A+      |                  |              |
|   8   | B-      |                  |              |

Please be aware of the different RJ45 pinout colors ([T-568A vs. T-568B](images/rj45-colors-t568a-vs-t568.png)).

## Requirements

* [ESPHome 2024.6.0 or higher](https://github.com/esphome/esphome/releases).
* Generic ESP32 or ESP8266 board

## Installation

You can install this component with [ESPHome external components feature](https://esphome.io/components/external_components.html) like this:
```yaml
external_components:
  - source: github://syssi/esphome-seplos-bms@main
```

or just use the `esp32-example.yaml` as proof of concept:

```bash
# Install esphome
pip3 install esphome

# Clone this external component
git clone https://github.com/syssi/esphome-seplos-bms.git
cd esphome-seplos-bms

# Create a secrets.yaml containing some setup specific secrets
cat > secrets.yaml <<EOF
wifi_ssid: MY_WIFI_SSID
wifi_password: MY_WIFI_PASSWORD

mqtt_host: MY_MQTT_HOST
mqtt_username: MY_MQTT_USERNAME
mqtt_password: MY_MQTT_PASSWORD
EOF

# Validate the configuration, create a binary, upload it, and start logs
# If you use a esp8266 run the esp8266-examle.yaml
esphome run esp32-example.yaml
```

![Screen recording](install.gif)

## Example response all sensors enabled

```
[I][seplos_bms:031]: Telemetry frame received
[D][sensor:124]: 'seplos-bms cell voltage 1': Sending state 3.28800 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 2': Sending state 3.30400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 3': Sending state 3.31600 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 4': Sending state 3.28700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 5': Sending state 3.31000 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 6': Sending state 3.30100 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 7': Sending state 3.29700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 8': Sending state 3.29300 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 9': Sending state 3.30500 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 10': Sending state 3.31200 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 11': Sending state 3.30400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 12': Sending state 3.31100 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 13': Sending state 3.30700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 14': Sending state 3.29000 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 15': Sending state 3.29400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 16': Sending state 3.28900 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms min cell voltage': Sending state 3.28700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms max cell voltage': Sending state 3.31600 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms max voltage cell': Sending state 3.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms min voltage cell': Sending state 4.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms delta cell voltage': Sending state 0.02900 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms average cell voltage': Sending state 3.30050 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 1': Sending state 29.82000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 2': Sending state 29.76000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 3': Sending state 29.67000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 4': Sending state 29.82000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms environment temperature': Sending state 29.81000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms mosfet temperature': Sending state 29.78000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms current': Sending state -6.54000 A with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms total voltage': Sending state 52.80000 V with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms power': Sending state -345.31198 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms charging power': Sending state 0.00000 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms discharging power': Sending state 345.31198 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms residual capacity': Sending state 133.86000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms battery capacity': Sending state 170.00000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms state of charge': Sending state 78.70000 % with 1 decimals of accuracy
[D][sensor:124]: 'seplos-bms rated capacity': Sending state 180.00000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms charging cycles': Sending state 70.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms state of health': Sending state 100.00000 % with 1 decimals of accuracy
[D][sensor:124]: 'seplos-bms port voltage': Sending state 52.79000 V with 2 decimals of accuracy
```

## Known issues and limitations

None.

## Protocol

See [SEPLOS BMS Communication Protocol_V2.0.pdf](docs/SEPLOS%20BMS%20Communication%20Protocol_V2.0.pdf) and [Seplos 48v 100A BMS RS485 Protocol.pdf](docs/Seplos%2048v%20100A%20BMS%20RS485%20Protocol.pdf).

```
$ echo -ne "~20004642E00200FD37\r" | hexdump -ve '1/1 "%.2X."'
      7E.32.30.30.30.34.36.34.32.45.30.30.32.30.30.46.44.33.37.0D.

# Get pack #0 telemetry data (CID2 `0x42`)
TX -> "~20004642E00200FD37\r"
RX <- "~2000460010960001100CD70CE90CF40CD60CEF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000DC6C\r"

# Get system parameters (CID2 `0x47`)
TX -> "~200046470000FDA9\r"
RX <- ?

# Get protocol version (CID2 `0x4F`)
TX -> "~2000464F0000FD9A\r"
RX <- ?

# Get manufacturer info (CID2 `0x51`)
TX -> "~200046510000FDAE\r"
RX <- "~20004600C040313130312D5350313520020743414E50726F746F636F6C3A536F666172202020F046\r"

# Get management info (pylontech only?)
TX -> "~200046920000FDA9\r"
RX <- ?

# Get module serial number (pylontech only?)
TX -> "~200046930000FDA8\r"
RX <- ?
```

## Debugging

If this component doesn't work out of the box for your device please update your configuration to enable the debug output of the UART component and increase the log level to the see outgoing and incoming serial traffic:

```
logger:
  level: DEBUG

uart:
  id: uart_0
  baud_rate: 9600
  tx_pin: ${tx_pin}
  rx_pin: ${rx_pin}
  rx_buffer_size: 384
  debug:
    dummy_receiver: false
    direction: BOTH
    after:
      delimiter: "\r"
    sequence:
      - lambda: UARTDebug::log_string(direction, bytes);
```

## References

* https://github.com/Frankkkkk/python-pylontech/blob/master/pylontech/pylontech.py
* https://diysolarforum.com/threads/simple-seplos-bms-protocol-decode-bash-script.34993/
* https://github.com/celsworth/lxp-pylon-utils/tree/master/lib/pylon/packet
* https://github.com/meteosat007/solar-pylontech
* https://github.com/BrucePerens/seplos_c
