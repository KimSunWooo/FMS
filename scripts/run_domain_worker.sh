#!/usr/bin/env bash
# Start the worker for one registered robot.
# The API supervisor already does this when FMS_DOMAIN_WORKERS_ENABLED is true.
# Do not start a second writer for the same robot while that supervisor is running.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT="${1:?usage: run_domain_worker.sh pinky-1|pinky-2|pinky-3}"
DOMAIN="$("$ROOT/.venv/bin/python" -c '
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]) / "backend"))
from app.services.fleet_registry import load_registry
robot_id = sys.argv[2]
robots = load_registry(Path(sys.argv[1]) / "config" / "physical_fleet.json")
match = next(robot for robot in robots if robot["robot_id"] == robot_id)
print(match["ros_domain_id"])
' "$ROOT" "$ROBOT")"
cd "$ROOT/backend"
exec env \
  ROS_DOMAIN_ID="$DOMAIN" \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  FMS_WORKER_ROBOT_ID="$ROBOT" \
  FMS_PHYSICAL_FLEET_PATH="$ROOT/config/physical_fleet.json" \
  FMS_DATA_DIR="$ROOT/data" \
  "$ROOT/.venv/bin/python" -m app.services.domain_worker
