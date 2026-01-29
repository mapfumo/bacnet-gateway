#!/usr/bin/env python3
"""
MQTT subscriber for LoRaWAN sensor data - displays decoded readings.
Usage: python3 mqtt_subscriber.py
"""

import json
import base64
import struct
import socket
import time
from datetime import datetime

GATEWAY_MQTT_HOST = "10.10.10.254"
GATEWAY_MQTT_PORT = 1883

LORA1_DEVEUI = "23ce1bfeff091fac"
LORA2_DEVEUI = "24ce1bfeff091fac"

def decode_lora1(payload_bytes):
    if len(payload_bytes) < 4:
        return None
    temp_raw, hum_raw = struct.unpack('>hH', payload_bytes[:4])
    return {'temp': temp_raw / 100.0, 'hum': hum_raw / 100.0}

def decode_lora2(payload_bytes):
    if len(payload_bytes) < 8:
        return None
    temp_raw, hum_raw, press_raw, gas_raw = struct.unpack('>hHHH', payload_bytes[:8])
    return {
        'temp': temp_raw / 100.0,
        'hum': hum_raw / 100.0,
        'press': press_raw / 10.0,
        'gas': gas_raw,
    }

def mqtt_connect(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(60)
    sock.connect((host, port))
    client_id = b"cli-subscriber"
    protocol_name = b"MQIsdp"
    var_header = struct.pack(">H", len(protocol_name)) + protocol_name + bytes([3, 0x02]) + struct.pack(">H", 60)
    payload = struct.pack(">H", len(client_id)) + client_id
    remaining_len = len(var_header) + len(payload)
    sock.send(bytes([0x10, remaining_len]) + var_header + payload)
    response = sock.recv(4)
    if len(response) >= 4 and response[0] == 0x20 and response[3] == 0x00:
        return sock
    raise Exception("MQTT connection failed")

def mqtt_subscribe(sock, topic):
    topic_bytes = topic.encode()
    var_header = struct.pack(">H", 1)
    payload = struct.pack(">H", len(topic_bytes)) + topic_bytes + bytes([0])
    remaining_len = len(var_header) + len(payload)
    sock.send(bytes([0x82, remaining_len]) + var_header + payload)
    sock.recv(5)

def mqtt_read_message(sock):
    try:
        first_byte = sock.recv(1)
        if not first_byte:
            return None, None
        packet_type = first_byte[0] >> 4
        multiplier, remaining_length = 1, 0
        while True:
            byte = sock.recv(1)
            if not byte:
                return None, None
            remaining_length += (byte[0] & 0x7F) * multiplier
            multiplier *= 128
            if not (byte[0] & 0x80):
                break
        payload = b''
        while len(payload) < remaining_length:
            chunk = sock.recv(remaining_length - len(payload))
            if not chunk:
                return None, None
            payload += chunk
        if packet_type == 3:
            topic_len = struct.unpack(">H", payload[:2])[0]
            topic = payload[2:2+topic_len].decode()
            message = payload[2+topic_len:]
            return topic, message
        return None, None
    except socket.timeout:
        return None, None

def main():
    print(f"Connecting to MQTT broker at {GATEWAY_MQTT_HOST}...")
    sock = mqtt_connect(GATEWAY_MQTT_HOST, GATEWAY_MQTT_PORT)
    mqtt_subscribe(sock, "application/#")
    print("Subscribed to application/#")
    print("-" * 60)

    while True:
        topic, message = mqtt_read_message(sock)
        if topic and message and "/rx" in topic:
            try:
                data = json.loads(message)
                dev_eui = data.get("devEUI", "").lower()
                payload_b64 = data.get("data", "")
                if not payload_b64:
                    continue
                payload_bytes = base64.b64decode(payload_b64)
                rx_info = data.get("rxInfo", [{}])[0]
                rssi = rx_info.get("rssi", 0)
                snr = rx_info.get("loRaSNR", 0)
                ts = datetime.now().strftime("%H:%M:%S")

                if dev_eui == LORA1_DEVEUI:
                    d = decode_lora1(payload_bytes)
                    if d:
                        print(f"[{ts}] LoRa-1 (SHT41)  : Temp={d['temp']:.1f}°C  Humidity={d['hum']:.1f}%  RSSI={rssi}dBm  SNR={snr}dB")
                elif dev_eui == LORA2_DEVEUI:
                    d = decode_lora2(payload_bytes)
                    if d:
                        print(f"[{ts}] LoRa-2 (BME680) : Temp={d['temp']:.1f}°C  Humidity={d['hum']:.1f}%  Pressure={d['press']:.0f}hPa  Gas={d['gas']}kΩ  RSSI={rssi}dBm  SNR={snr}dB")
            except Exception as e:
                pass

if __name__ == "__main__":
    main()
