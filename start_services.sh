#!/bin/bash
# Start all Unified IIoT Monitoring services
# LoRaWAN + Modbus TCP -> InfluxDB -> Grafana

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "Starting Unified IIoT Monitoring Platform"
echo "=============================================="

docker compose up -d

echo ""
echo "=== Service Status ==="
docker compose ps

echo ""
echo "=== Waiting for services to initialize ==="
sleep 5

echo ""
echo "=== MQTT Bridge Logs (last 5 lines) ==="
docker compose logs mqtt-bridge --tail 5

echo ""
echo "=== Modbus Bridge Logs (last 5 lines) ==="
docker compose logs modbus-bridge --tail 5

echo ""
echo "=============================================="
echo "Services available at:"
echo "  Grafana:  http://localhost:3000  (admin/admin)"
echo "  InfluxDB: http://localhost:8086  (admin/admin123456)"
echo ""
echo "Dashboard: http://localhost:3000/d/unified-monitoring"
echo "=============================================="
