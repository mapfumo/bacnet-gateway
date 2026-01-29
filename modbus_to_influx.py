#!/usr/bin/env python3
"""
Modbus TCP to InfluxDB bridge for industrial sensor data.
Polls STM32F446RE Modbus TCP slaves, decodes registers, writes to InfluxDB.

Part of wk11-unified-monitoring - Unified IIoT Monitoring Platform.

Modbus Register Map (per device):
  40001-40002: Temperature (IEEE 754 float32, big-endian)
  40003-40004: Humidity (IEEE 754 float32, big-endian)
  40005:       Device Status (u16)
  40006-40007: Uptime seconds (u32, big-endian)
"""

import struct
import socket
import time
from datetime import datetime
import signal
import urllib.request
import urllib.error

# =========================
# Configuration
# =========================
MODBUS_DEVICES = [
    {"name": "modbus1", "host": "10.10.10.100", "port": 502, "sensor": "SHT3x"},
    {"name": "modbus2", "host": "10.10.10.200", "port": 502, "sensor": "SHT3x"},
]

POLL_INTERVAL = 2  # seconds

# InfluxDB - use localhost since this runs with network_mode: host
INFLUXDB_HOST = "localhost"
INFLUXDB_PORT = 8086
INFLUXDB_TOKEN = "my-super-secret-auth-token"
INFLUXDB_ORG = "my-org"
INFLUXDB_BUCKET = "sensors"  # Unified bucket for all sensors

# =========================
# Graceful shutdown
# =========================
shutdown_flag = False

def shutdown(sig, frame):
    global shutdown_flag
    print("\nReceived shutdown signal...")
    shutdown_flag = True

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)

# =========================
# IEEE 754 decoding
# =========================
def decode_float32(reg_high, reg_low):
    """Decode two Modbus registers into IEEE 754 float32."""
    bytes_data = struct.pack('>HH', reg_high, reg_low)
    return struct.unpack('>f', bytes_data)[0]

def decode_uint32(reg_high, reg_low):
    """Decode two Modbus registers into uint32."""
    return (reg_high << 16) | reg_low

# =========================
# Modbus TCP client
# =========================
def modbus_read_holding_registers(host, port, start_addr, count, unit_id=1, timeout=5):
    """
    Read holding registers from Modbus TCP slave.
    Returns list of register values or None on error.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))

        # MBAP Header
        transaction_id = 1
        protocol_id = 0  # Modbus
        unit_id_byte = unit_id
        function_code = 3  # Read Holding Registers

        # PDU
        pdu = struct.pack('>BHH', function_code, start_addr, count)
        length = len(pdu) + 1  # +1 for unit_id

        # Full request
        mbap = struct.pack('>HHHB', transaction_id, protocol_id, length, unit_id_byte)
        request = mbap + pdu

        sock.send(request)
        response = sock.recv(256)
        sock.close()

        if len(response) < 9:
            return None

        # Parse response
        resp_transaction_id = struct.unpack('>H', response[0:2])[0]
        resp_protocol_id = struct.unpack('>H', response[2:4])[0]
        resp_length = struct.unpack('>H', response[4:6])[0]
        resp_unit_id = response[6]
        resp_function_code = response[7]

        if resp_function_code != function_code:
            # Error response
            return None

        byte_count = response[8]
        register_data = response[9:9+byte_count]

        # Unpack registers
        registers = []
        for i in range(0, byte_count, 2):
            reg_value = struct.unpack('>H', register_data[i:i+2])[0]
            registers.append(reg_value)

        return registers

    except Exception as e:
        print(f"Modbus error ({host}): {e}")
        return None

# =========================
# InfluxDB write
# =========================
def write_to_influx(measurement, tags, fields, timestamp=None):
    """Write a point to InfluxDB using line protocol over HTTP."""
    tag_str = ",".join(f"{k}={v}" for k, v in tags.items())
    field_str = ",".join(
        f'{k}={v}' if isinstance(v, (int, float)) else f'{k}="{v}"'
        for k, v in fields.items()
    )
    line = f"{measurement},{tag_str} {field_str}"
    if timestamp:
        line += f" {timestamp}"

    url = f"http://{INFLUXDB_HOST}:{INFLUXDB_PORT}/api/v2/write?org={INFLUXDB_ORG}&bucket={INFLUXDB_BUCKET}&precision=ns"
    req = urllib.request.Request(url, data=line.encode(), method='POST')
    req.add_header('Authorization', f'Token {INFLUXDB_TOKEN}')
    req.add_header('Content-Type', 'text/plain')

    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status == 204
    except urllib.error.URLError as e:
        print(f"InfluxDB write error: {e}")
        return False

# =========================
# Poll and process device
# =========================
def poll_device(device):
    """Poll a single Modbus device and write to InfluxDB."""
    name = device["name"]
    host = device["host"]
    port = device["port"]
    sensor = device["sensor"]

    # Read 7 registers starting at address 0 (40001-40007)
    registers = modbus_read_holding_registers(host, port, 0, 7)

    if registers is None or len(registers) < 7:
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] {name}: Connection failed")
        return False

    # Decode values
    temperature = decode_float32(registers[0], registers[1])
    humidity = decode_float32(registers[2], registers[3])
    status = registers[4]
    uptime = decode_uint32(registers[5], registers[6])

    # Build fields and tags
    fields = {
        "temperature": round(temperature, 2),
        "humidity": round(humidity, 2),
        "status": status,
        "uptime": uptime,
    }
    tags = {
        "node": name,
        "sensor": sensor,
        "protocol": "modbus",
        "ip": host,
    }

    timestamp = int(time.time() * 1e9)
    success = write_to_influx("modbus_sensor", tags, fields, timestamp)

    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {name}: Temp={temperature:.1f}C Hum={humidity:.1f}% Status={status} Uptime={uptime}s -> InfluxDB: {'OK' if success else 'FAIL'}")

    return success

# =========================
# Main loop
# =========================
def main():
    global shutdown_flag
    print("=" * 60)
    print("Modbus TCP -> InfluxDB Bridge")
    print("Part of Unified IIoT Monitoring Platform")
    print(f"Devices: {', '.join(d['host'] for d in MODBUS_DEVICES)}")
    print(f"InfluxDB: {INFLUXDB_HOST}:{INFLUXDB_PORT}/{INFLUXDB_BUCKET}")
    print(f"Poll interval: {POLL_INTERVAL}s")
    print("=" * 60)

    while not shutdown_flag:
        for device in MODBUS_DEVICES:
            if shutdown_flag:
                break
            poll_device(device)

        # Wait for next poll interval
        if not shutdown_flag:
            time.sleep(POLL_INTERVAL)

    print("Bridge exiting cleanly.")

# =========================
# Entry point
# =========================
if __name__ == "__main__":
    main()
