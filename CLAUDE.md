# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

Unified IIoT Monitoring Platform combining LoRaWAN, Modbus TCP, and BACnet/IP sensor data into a single Grafana dashboard. This is an educational project using embedded Rust, Python 3.11, Docker Compose, InfluxDB 2, Grafana, and MQTT.

**Hardware:**
- 2 LoRaWAN nodes (STM32WL55JC)
- 1 Unified Gateway - Modbus TCP + BACnet/IP (STM32F446RE + W5500) at 10.10.10.100
- 1 Modbus TCP node (STM32F446RE + W5500) at 10.10.10.200

## Common Commands

```bash
# Start all services
./start_services.sh

# Stop all services
./stop_services.sh

# View logs
docker compose logs -f                    # All services
docker compose logs -f mqtt-bridge        # LoRaWAN bridge only
docker compose logs -f modbus-bridge      # Modbus bridge only

# Check service status
docker compose ps

# View decoded LoRaWAN sensor readings in terminal
python3 mqtt_subscriber.py

# Test BACnet
python3 bacnet_to_influx.py
```

## Architecture

```
Sensor Nodes → Protocol Gateways → Python Bridges → InfluxDB → Grafana
                                        ↓
                    ┌──────────────────────────────────────┐
                    │ mqtt_to_influx.py (LoRaWAN → MQTT)   │
                    │ modbus_to_influx.py (Modbus TCP)     │
                    │ bacnet_to_influx.py (BACnet/IP)      │
                    └──────────────────────────────────────┘
```

**Key Design Decisions:**
- Raw socket implementations for MQTT, Modbus, and BACnet (no external libraries)
- Single InfluxDB bucket (`sensors`) with measurements: `lorawan_sensor`, `modbus_sensor`, `bacnet_sensor`
- `modbus-bridge` uses Docker host network mode for OT subnet (10.10.10.x) access
- Grafana dashboards auto-provision from `grafana/dashboards/`

## Data Model

**InfluxDB Bucket:** `sensors`

| Measurement | Tags | Fields |
|------------|------|--------|
| `lorawan_sensor` | dev_eui, node, sensor, protocol=lorawan | temperature, humidity, pressure, gas_resistance, rssi, snr, frame_count |
| `modbus_sensor` | node, sensor, protocol=modbus, ip | temperature, humidity, status, uptime |
| `bacnet_sensor` | node, device_id, protocol=bacnet, ip | temperature, humidity, uptime |

**Modbus Register Map:**
- 40001-40002: Temperature (float32)
- 40003-40004: Humidity (float32)
- 40005: Device Status (u16)
- 40006-40007: Uptime (u32)

**BACnet Objects (Unified Gateway):**
- Device (instance 1234)
- Analog Input 0: Temperature (°C)
- Analog Input 1: Humidity (%)
- Analog Value 0: Uptime (seconds)

## Service Endpoints

| Service | URL | Credentials |
|---------|-----|-------------|
| Grafana | http://localhost:3000 | admin / admin123456 |
| InfluxDB | http://localhost:8086 | admin / admin123456 |
| MQTT Broker | localhost:1883 | anonymous allowed |

## Network Topology

- **OT Subnet:** 10.10.10.x
  - 10.10.10.100: Unified Gateway (Modbus TCP:502 + BACnet/IP:47808)
  - 10.10.10.200: Modbus Node 2 (Modbus TCP:502)
  - 10.10.10.254: LoRaWAN Gateway
- **Docker Network:** monitoring-network (bridge)
- **Host Network:** modbus-bridge container only

## Firmware

Source code for all sensor nodes is in `firmware/`. Requires Rust with `thumbv7em-none-eabihf` target and probe-rs.

```bash
# Prerequisites
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools

# Build Unified Gateway (Modbus + BACnet)
cd firmware/unified-gateway && cargo build --release

# Build LoRaWAN nodes
cd firmware/lorawan/lora-1 && cargo build --release
cd firmware/lorawan/lora-2 && cargo build --release

# Build Modbus node 2
cd firmware/modbus && cargo build --release --bin modbus_2

# Flash with probe-rs
cd firmware/unified-gateway && cargo run --release --probe 0483:374b:0671FF3833554B3043164817
cd firmware/lorawan/lora-1 && cargo run --release --probe 0483:374e:003E00463234510A33353533
cd firmware/lorawan/lora-2 && cargo run --release --probe 0483:374e:0026003A3234510A33353533
cd firmware/modbus && cargo run --release --bin modbus_2 --probe 0483:374b:066DFF3833584B3043115433
```

## Unified Gateway (Modbus + BACnet)

The unified gateway at 10.10.10.100 runs both protocols on a single STM32F446RE:

| Protocol | Port | Purpose |
|----------|------|---------|
| Modbus TCP | 502 | SCADA/PLC integration |
| BACnet/IP | 47808 | Building Management Systems |

**BACnet Services:**
- Who-Is / I-Am: Device discovery
- ReadProperty: Sensor value retrieval

**Pin Assignments:**
| Function | Pins |
|----------|------|
| SPI1 (W5500) | PA5 (SCK), PA6 (MISO), PA7 (MOSI), PB6 (CS) |
| I2C1 (SHT3x/OLED) | PB8 (SCL), PB9 (SDA) |
| W5500 RST | PC7 |

## LoRaWAN Firmware Behavior

The LoRaWAN nodes operate independently of network connectivity:

**Startup Sequence:**
1. Initialize hardware (clocks, I2C, radio)
2. Initialize LCD display immediately
3. Begin sensor readings and LCD updates
4. Attempt OTAA join (up to 5 attempts) while continuing to display sensor data
5. If join fails, continue in "offline mode" with periodic retries (~60s)

**Transmission Intervals:**
- Uplinks: Every ~30 seconds when joined
- Join retries (offline): Every ~60 seconds

**Stale Session Detection:**
- Every 5th uplink is sent as confirmed
- If 3 consecutive confirmed uplinks fail, triggers rejoin

**LoRaWAN Patched Dependencies:** The `lora-phy-patched` and `lorawan-device-patched` directories contain modified versions for embassy-time 0.4.0 compatibility.

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for common issues and solutions including:
- W5500 SPI communication failures
- BACnet ReadProperty parsing bugs
- UDP socket issues
- Embassy task arena sizing
- LoRaWAN session/connectivity issues
- Docker/Grafana service issues
- Firmware flashing with probe-rs

## Security Notes

This is configured for **development only**. Critical gaps include unauthenticated MQTT/Modbus/BACnet, default credentials, and no TLS.
