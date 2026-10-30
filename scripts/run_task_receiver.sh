#!/usr/bin/env bash
# Task receiver: store + receive ACK. Nav2 motion is fms_nav2_executor.
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
export FMS_DATA_DIR="${FMS_DATA_DIR:-$ROOT/data}"
export FMS_RECEIVER_INBOX="${FMS_RECEIVER_INBOX:-$FMS_DATA_DIR/ros_ipc/${FMS_RECEIVER_ROBOT_ID}/inbox}"
export FMS_TASK_EXECUTION_ENABLED="${FMS_TASK_EXECUTION_ENABLED:-true}"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
# System Python keeps rclpy/fms_msgs; preserve stderr in the parent launcher.
exec /usr/bin/python3 "$ROOT/robot_receiver/ros_task_node.py"
