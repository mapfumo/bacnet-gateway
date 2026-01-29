#!/bin/bash
# Stop all Unified IIoT Monitoring services

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "Stopping Unified IIoT Monitoring Platform"
echo "=============================================="

echo ""
echo "=== Current Service Status ==="
docker compose ps

echo ""
echo "=== Stopping all services ==="
docker compose down

echo ""
echo "=============================================="
echo "All services stopped."
echo ""
echo "To restart: ./start_services.sh"
echo "To view logs: docker compose logs -f"
echo "=============================================="
