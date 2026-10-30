#!/usr/bin/env bash
# One physical robot gateway. Does not change the parent process ROS_DOMAIN_ID.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -z "${FMS_WORKER_ROBOT_ID:-}" || -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo "FMS_WORKER_ROBOT_ID and ROS_DOMAIN_ID are required" >&2
  exit 2
fi
if [[ ! -f "$ROOT/ros_ws/install/setup.bash" ]]; then
  echo "fms_msgs is not built. Run: source /opt/ros/jazzy/setup.bash && cd $ROOT/ros_ws && colcon build --packages-select fms_msgs" >&2
  exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FMS_DATA_DIR="${FMS_DATA_DIR:-$ROOT/data}"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
# System Python with ROS packages (not the project venv).
exec /usr/bin/python3 "$ROOT/robot_receiver/ros_gateway_node.py"
