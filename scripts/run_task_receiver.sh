#!/usr/bin/env bash
# Receive-only ROS subscriber. execute stays false. No Nav2 goal.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -z "${FMS_RECEIVER_ROBOT_ID:-}" || -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo "FMS_RECEIVER_ROBOT_ID and ROS_DOMAIN_ID are required" >&2
  exit 2
fi
if [[ ! -f "$ROOT/ros_ws/install/setup.bash" ]]; then
  echo "fms_msgs is not built" >&2
  exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
exec python3 "$ROOT/robot_receiver/ros_task_node.py"
