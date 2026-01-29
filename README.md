# Unified IIoT Monitoring Platform

A complete Industrial IoT monitoring solution combining **LoRaWAN**, **Modbus TCP**, and **BACnet/IP** protocols into a unified Grafana dashboard.

> **Note:** This project evolved from [wk11-unified-monitoring](https://github.com/username/wk11-unified-monitoring), extending it with BACnet/IP support and consolidating all firmware into a single repository.

## Features

- **Multi-Protocol Support**: LoRaWAN, Modbus TCP, and BACnet/IP
- **Unified Dashboard**: All sensor data visualized in Grafana
- **Embedded Rust Firmware**: Embassy async runtime on STM32
- **Docker Infrastructure**: InfluxDB, Grafana, Mosquitto with one command
- **Pure Socket Implementations**: No external protocol libraries

## Hardware

| Device | MCU | Protocols | IP Address |
|--------|-----|-----------|------------|
| Unified Gateway | STM32F446RE + W5500 | Modbus TCP + BACnet/IP | 10.10.10.100 |
| Modbus Node 2 | STM32F446RE + W5500 | Modbus TCP | 10.10.10.200 |
| LoRa Node 1 | STM32WL55JC | LoRaWAN | via gateway |
| LoRa Node 2 | STM32WL55JC | LoRaWAN | via gateway |

All nodes use SHT3x temperature/humidity sensors and SSD1306 OLED displays.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Sensor Nodes                                   │
├─────────────────┬─────────────────┬─────────────────┬───────────────────┤
│   LoRa Node 1   │   LoRa Node 2   │ Unified Gateway │   Modbus Node 2   │
│   (STM32WL55)   │   (STM32WL55)   │  (STM32F446RE)  │   (STM32F446RE)   │
│                 │                 │                 │                   │
│   LoRaWAN       │   LoRaWAN       │ Modbus + BACnet │   Modbus TCP      │
└────────┬────────┴────────┬────────┴────────┬────────┴─────────┬─────────┘
         │                 │                 │                  │
         ▼                 ▼                 ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        Protocol Gateways                                 │
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────────┐ │
│  │ ChirpStack       │  │ Direct TCP:502   │  │ Direct UDP:47808       │ │
│  │ (LoRaWAN → MQTT) │  │ (Modbus TCP)     │  │ (BACnet/IP)            │ │
│  └────────┬─────────┘  └────────┬─────────┘  └───────────┬────────────┘ │
└───────────┼─────────────────────┼────────────────────────┼──────────────┘
            │                     │                        │
            ▼                     ▼                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        Python Data Collectors                            │
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────────┐ │
│  │ mqtt_to_influx   │  │ modbus_to_influx │  │ bacnet_to_influx       │ │
│  └────────┬─────────┘  └────────┬─────────┘  └───────────┬────────────┘ │
└───────────┼─────────────────────┼────────────────────────┼──────────────┘
            │                     │                        │
            ▼                     ▼                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            InfluxDB                                      │
│                    Bucket: sensors                                       │
│   ┌────────────────┐ ┌────────────────┐ ┌────────────────┐              │
│   │ lorawan_sensor │ │ modbus_sensor  │ │ bacnet_sensor  │              │
│   └────────────────┘ └────────────────┘ └────────────────┘              │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            Grafana                                       │
│                    Unified Dashboard                                     │
└─────────────────────────────────────────────────────────────────────────┘
```

## Quick Start

### 1. Start Infrastructure

```bash
# Start Docker services (InfluxDB, Grafana, Mosquitto)
./start_services.sh

# Access Grafana at http://localhost:3000 (admin / admin123456)
```

### 2. Flash Firmware

```bash
# Prerequisites
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools

# Flash Unified Gateway (Modbus + BACnet)
cd firmware/unified-gateway
cargo run --release

# Flash other nodes as needed
cd firmware/lorawan/lora-1 && cargo run --release
cd firmware/lorawan/lora-2 && cargo run --release
cd firmware/modbus && cargo run --release --bin modbus_2
```

### 3. Run Data Collectors

```bash
# LoRaWAN (via Docker)
# Already running in mqtt-bridge container

# Modbus TCP
python3 modbus_to_influx.py &

# BACnet/IP
python3 bacnet_to_influx.py &
```

## Protocol Details

### Modbus TCP (Port 502)

| Register | Description | Type |
|----------|-------------|------|
| 0-1 | Temperature | f32 (IEEE 754) |
| 2-3 | Humidity | f32 (IEEE 754) |
| 4 | Status | u16 |
| 5-6 | Uptime | u32 |

### BACnet/IP (Port 47808)

| Object | Instance | Property | Value |
|--------|----------|----------|-------|
| Device | 1234 | Object_Name | "BACnet-Gateway" |
| Analog Input | 0 | Present_Value | Temperature (°C) |
| Analog Input | 1 | Present_Value | Humidity (%) |

### LoRaWAN

OTAA join with automatic rejoin on gateway restart. Uplinks every 30 seconds with BME680 sensor data (temperature, humidity, pressure, gas resistance).

## Project Structure

```
unified-iiot-platform/
├── firmware/
│   ├── unified-gateway/     # Modbus TCP + BACnet/IP (10.10.10.100)
│   ├── modbus/              # Modbus node 2 (10.10.10.200)
│   └── lorawan/
│       ├── lora-1/          # LoRaWAN node 1
│       ├── lora-2/          # LoRaWAN node 2
│       ├── lora-phy-patched/
│       └── lorawan-device-patched/
├── grafana/
│   └── dashboards/          # Auto-provisioned dashboards
├── mosquitto/
│   └── config/              # MQTT broker configuration
├── bacnet_to_influx.py      # BACnet data collector
├── modbus_to_influx.py      # Modbus data collector
├── mqtt_to_influx.py        # LoRaWAN MQTT bridge
├── mqtt_subscriber.py       # Debug MQTT subscriber
├── docker-compose.yml       # Infrastructure services
├── start_services.sh
├── stop_services.sh
├── CLAUDE.md                # AI assistant context
└── TROUBLESHOOTING.md       # Common issues and solutions
```

## Service Endpoints

| Service | URL | Credentials |
|---------|-----|-------------|
| Grafana | http://localhost:3000 | admin / admin123456 |
| InfluxDB | http://localhost:8086 | admin / admin123456 |
| MQTT Broker | localhost:1883 | anonymous |

## Testing

### Modbus TCP

```bash
# Using modbus-cli
modbus tcp://10.10.10.100 read 0 4
modbus tcp://10.10.10.200 read 0 4
```

### BACnet/IP

1. Install [Yabe](https://sourceforge.net/projects/yetanotherbacnetexplorer/)
2. Send Who-Is broadcast
3. Device 1234 should appear
4. Read Analog Input 0 (temperature) and 1 (humidity)

### LoRaWAN

```bash
# View decoded sensor readings
python3 mqtt_subscriber.py
```

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for common issues including:

- W5500 SPI communication failures
- BACnet packet parsing issues
- Embassy task arena sizing
- LoRaWAN session management
- Docker service issues

## Evolution from wk11-unified-monitoring

This project builds upon and consolidates [wk11-unified-monitoring](../wk11-unified-monitoring/):

| Feature | wk11 | This Project |
|---------|------|--------------|
| LoRaWAN | ✓ | ✓ |
| Modbus TCP | ✓ (2 nodes) | ✓ (1 node + unified) |
| BACnet/IP | ✗ | ✓ |
| Unified Gateway | ✗ | ✓ (Modbus + BACnet) |
| Single Repository | ✗ | ✓ |

## References

- [Embassy Framework](https://embassy.dev/)
- [BACnet Standard (ASHRAE 135)](https://www.ashrae.org/)
- [Modbus Application Protocol](https://modbus.org/specs.php)
- [LoRaWAN Specification](https://lora-alliance.org/lorawan-for-developers/)
- [W5500 Datasheet](https://www.wiznet.io/product-item/w5500/)

## License

MIT
