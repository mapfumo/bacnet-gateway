#!/usr/bin/env python3
"""
BACnet/IP to InfluxDB Bridge

This script polls a BACnet/IP device and writes sensor data to InfluxDB.
It integrates with the wk11-unified-monitoring platform.

How it works:
1. Send Who-Is broadcast to discover the device
2. Send ReadProperty requests to get temperature and humidity
3. Write values to InfluxDB (same bucket as Modbus/LoRaWAN data)

BACnet Objects on our device:
- Device (instance 1234)
- Analog Input 0: Temperature (°C)
- Analog Input 1: Humidity (%)
- Analog Value 0: Uptime (seconds)

Usage:
    python3 bacnet_to_influx.py

Requirements:
    pip install BAC0
    (or use the pure socket implementation below)
"""

import socket
import struct
import time
import signal
import urllib.request
import urllib.error
from datetime import datetime

# =============================================================================
# Configuration
# =============================================================================

# BACnet device settings
# This is the unified gateway running Modbus TCP:502 + BACnet/IP:47808
BACNET_DEVICE_IP = "10.10.10.100"  # Unified gateway IP (same as modbus_1)
BACNET_PORT = 47808                 # BACnet/IP port (0xBAC0)
DEVICE_INSTANCE = 1234              # Our device instance ID

# Polling interval
POLL_INTERVAL = 2  # seconds

# InfluxDB settings (same as modbus_to_influx.py)
INFLUXDB_HOST = "localhost"
INFLUXDB_PORT = 8086
INFLUXDB_TOKEN = "my-super-secret-auth-token"
INFLUXDB_ORG = "my-org"
INFLUXDB_BUCKET = "sensors"

# =============================================================================
# Graceful Shutdown
# =============================================================================

shutdown_flag = False

def shutdown_handler(sig, frame):
    global shutdown_flag
    print("\nShutting down...")
    shutdown_flag = True

signal.signal(signal.SIGTERM, shutdown_handler)
signal.signal(signal.SIGINT, shutdown_handler)

# =============================================================================
# BACnet/IP Protocol - Packet Building
# =============================================================================

def build_who_is():
    """
    Build a Who-Is broadcast packet.

    Packet structure:
    - BVLC Header (4 bytes): type=0x81, function=0x0B (broadcast), length
    - NPDU (2 bytes): version=1, control=0
    - APDU (2 bytes): PDU type=0x10 (unconfirmed), service=0x08 (Who-Is)
    """
    # APDU: Unconfirmed Request, Who-Is service
    apdu = bytes([0x10, 0x08])

    # NPDU: Version 1, no routing
    npdu = bytes([0x01, 0x00])

    # BVLC: BACnet/IP, broadcast
    total_length = 4 + len(npdu) + len(apdu)
    bvlc = bytes([0x81, 0x0B, (total_length >> 8) & 0xFF, total_length & 0xFF])

    return bvlc + npdu + apdu


def build_read_property(invoke_id, object_type, object_instance, property_id):
    """
    Build a ReadProperty request packet.

    Arguments:
        invoke_id: Transaction ID (0-255)
        object_type: BACnet object type (0=Analog Input, 8=Device)
        object_instance: Object instance number
        property_id: Property to read (85=Present Value, 77=Object Name)
    """
    # Build APDU
    apdu = bytearray()

    # PDU Type: Confirmed Request (0x00), no segmentation
    apdu.append(0x00)

    # Max segments (0) + Max APDU (5 = 480 bytes)
    apdu.append(0x05)

    # Invoke ID
    apdu.append(invoke_id)

    # Service Choice: ReadProperty (0x0C)
    apdu.append(0x0C)

    # Context Tag 0: Object Identifier
    # Tag: 0x0C (context 0, constructed, 4 bytes)
    apdu.append(0x0C)
    object_id = (object_type << 22) | (object_instance & 0x3FFFFF)
    apdu.extend(object_id.to_bytes(4, 'big'))

    # Context Tag 1: Property Identifier
    # Tag depends on property_id size
    if property_id < 256:
        apdu.append(0x19)  # Context 1, 1 byte
        apdu.append(property_id)
    else:
        apdu.append(0x1A)  # Context 1, 2 bytes
        apdu.extend(property_id.to_bytes(2, 'big'))

    # NPDU: Version 1, expecting reply
    npdu = bytes([0x01, 0x04])  # 0x04 = expecting reply

    # BVLC: BACnet/IP, unicast
    total_length = 4 + len(npdu) + len(apdu)
    bvlc = bytes([0x81, 0x0A, (total_length >> 8) & 0xFF, total_length & 0xFF])

    return bvlc + npdu + bytes(apdu)


# =============================================================================
# BACnet/IP Protocol - Response Parsing
# =============================================================================

def parse_i_am(data):
    """
    Parse an I-Am response to extract device instance.

    Returns device instance or None if parsing fails.
    """
    if len(data) < 12:
        return None

    # Check BVLC type
    if data[0] != 0x81:
        return None

    # Skip BVLC (4) + NPDU (2) = 6 bytes
    # Check APDU: Unconfirmed (0x10), I-Am (0x00)
    if len(data) < 8:
        return None

    if data[6] != 0x10 or data[7] != 0x00:
        return None

    # Parse Object Identifier (starts at byte 8)
    if data[8] != 0xC4:  # Object ID tag
        return None

    obj_id = int.from_bytes(data[9:13], 'big')
    obj_type = (obj_id >> 22) & 0x3FF
    obj_instance = obj_id & 0x3FFFFF

    if obj_type == 8:  # Device
        return obj_instance

    return None


def parse_read_property_ack(data):
    """
    Parse a ReadProperty-ACK response.

    Returns the property value as a float, or None if parsing fails.
    """
    if len(data) < 15:
        return None

    # Check BVLC type
    if data[0] != 0x81:
        return None

    # Skip BVLC (4) + NPDU (2) = 6 bytes
    # Check APDU: Complex ACK (0x30)
    if (data[6] & 0xF0) != 0x30:
        return None

    # Find the property value (context tag 3)
    # Look for opening tag 0x3E
    try:
        idx = data.index(0x3E)
        idx += 1  # Move past opening tag

        # Check the application tag
        tag = data[idx]

        if tag == 0x44:  # REAL (float32)
            # Next 4 bytes are the float value (big-endian)
            float_bytes = data[idx+1:idx+5]
            value = struct.unpack('>f', float_bytes)[0]
            return value

        elif tag == 0x91:  # Enumerated (1 byte)
            return float(data[idx+1])

        elif tag == 0x21:  # Unsigned (1 byte)
            return float(data[idx+1])

        elif tag == 0x22:  # Unsigned (2 bytes)
            return float(int.from_bytes(data[idx+1:idx+3], 'big'))

    except (ValueError, IndexError):
        pass

    return None


# =============================================================================
# BACnet/IP Communication
# =============================================================================

def send_bacnet_request(sock, dest_ip, dest_port, request, timeout=2.0):
    """
    Send a BACnet request and wait for response.

    Returns response data or None on timeout/error.
    """
    try:
        sock.sendto(request, (dest_ip, dest_port))
        sock.settimeout(timeout)
        data, addr = sock.recvfrom(1024)
        return data
    except socket.timeout:
        return None
    except Exception as e:
        print(f"Socket error: {e}")
        return None


def discover_device(sock):
    """
    Send Who-Is and wait for I-Am response.

    Returns device instance or None.
    """
    request = build_who_is()

    # Broadcast Who-Is
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(request, ('255.255.255.255', BACNET_PORT))
        sock.settimeout(2.0)

        # Wait for I-Am response
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                instance = parse_i_am(data)
                if instance is not None:
                    print(f"Discovered device {instance} at {addr[0]}")
                    return instance, addr[0]
            except socket.timeout:
                break

    except Exception as e:
        print(f"Discovery error: {e}")

    return None, None


def read_present_value(sock, device_ip, object_type, object_instance, invoke_id=1):
    """
    Read the Present Value property from an object.

    Arguments:
        sock: UDP socket
        device_ip: Device IP address
        object_type: 0=Analog Input, 8=Device
        object_instance: Instance number
        invoke_id: Transaction ID

    Returns float value or None.
    """
    # Property ID 85 = Present Value
    request = build_read_property(invoke_id, object_type, object_instance, 85)
    response = send_bacnet_request(sock, device_ip, BACNET_PORT, request)

    if response:
        return parse_read_property_ack(response)

    return None


# =============================================================================
# InfluxDB Writer
# =============================================================================

def write_to_influx(measurement, tags, fields, timestamp=None):
    """
    Write a data point to InfluxDB using the HTTP API.

    Uses the same format as modbus_to_influx.py for consistency.
    """
    # Build line protocol
    tag_str = ",".join(f"{k}={v}" for k, v in tags.items())
    field_str = ",".join(
        f'{k}={v}' if isinstance(v, (int, float)) else f'{k}="{v}"'
        for k, v in fields.items()
    )
    line = f"{measurement},{tag_str} {field_str}"
    if timestamp:
        line += f" {timestamp}"

    # Send to InfluxDB
    url = f"http://{INFLUXDB_HOST}:{INFLUXDB_PORT}/api/v2/write"
    url += f"?org={INFLUXDB_ORG}&bucket={INFLUXDB_BUCKET}&precision=ns"

    req = urllib.request.Request(url, data=line.encode(), method='POST')
    req.add_header('Authorization', f'Token {INFLUXDB_TOKEN}')
    req.add_header('Content-Type', 'text/plain')

    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status == 204
    except urllib.error.URLError as e:
        print(f"InfluxDB error: {e}")
        return False


# =============================================================================
# Main Loop
# =============================================================================

def main():
    global shutdown_flag

    print("=" * 60)
    print("BACnet/IP to InfluxDB Bridge")
    print("Part of Unified IIoT Monitoring Platform")
    print(f"Target Device: {BACNET_DEVICE_IP}:{BACNET_PORT}")
    print(f"Device Instance: {DEVICE_INSTANCE}")
    print(f"InfluxDB: {INFLUXDB_HOST}:{INFLUXDB_PORT}/{INFLUXDB_BUCKET}")
    print(f"Poll interval: {POLL_INTERVAL}s")
    print("=" * 60)

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind to any available port
    sock.bind(('', 0))
    local_port = sock.getsockname()[1]
    print(f"Local UDP port: {local_port}")

    # Try to discover device first
    print("\nSending Who-Is broadcast...")
    discovered_instance, discovered_ip = discover_device(sock)

    if discovered_instance:
        print(f"Found device: instance={discovered_instance}, ip={discovered_ip}")
        device_ip = discovered_ip
    else:
        print(f"No response to Who-Is, using configured IP: {BACNET_DEVICE_IP}")
        device_ip = BACNET_DEVICE_IP

    print("\nStarting polling loop...")
    invoke_id = 1

    while not shutdown_flag:
        ts = datetime.now().strftime("%H:%M:%S")

        # Read temperature (Analog Input 0)
        temperature = read_present_value(sock, device_ip, 0, 0, invoke_id)
        invoke_id = (invoke_id + 1) % 256

        # Small delay between requests
        time.sleep(0.1)

        # Read humidity (Analog Input 1)
        humidity = read_present_value(sock, device_ip, 0, 1, invoke_id)
        invoke_id = (invoke_id + 1) % 256

        time.sleep(0.1)

        # Read uptime from Analog Value 0 (object_type=2)
        # BACnet uses Analog Value for computed/derived data vs Analog Input for physical sensors
        uptime = read_present_value(sock, device_ip, 2, 0, invoke_id)
        invoke_id = (invoke_id + 1) % 256

        if temperature is not None and humidity is not None:
            # Write to InfluxDB
            tags = {
                "node": "bacnet1",
                "sensor": "SHT3x",
                "protocol": "bacnet",
                "ip": device_ip,
            }
            fields = {
                "temperature": round(temperature, 2),
                "humidity": round(humidity, 2),
                "device_instance": DEVICE_INSTANCE,
            }
            if uptime is not None:
                fields["uptime"] = int(uptime)

            timestamp = int(time.time() * 1e9)
            success = write_to_influx("bacnet_sensor", tags, fields, timestamp)

            uptime_str = f" Uptime={int(uptime)}s" if uptime is not None else ""
            print(f"[{ts}] Temp={temperature:.1f}°C Hum={humidity:.1f}%{uptime_str} -> InfluxDB: {'OK' if success else 'FAIL'}")
        else:
            print(f"[{ts}] Failed to read from device (temp={temperature}, hum={humidity})")

        # Wait for next poll
        time.sleep(POLL_INTERVAL)

    sock.close()
    print("Bridge stopped.")


if __name__ == "__main__":
    main()
