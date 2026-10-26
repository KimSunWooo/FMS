#!/usr/bin/env bash
# Relaunch Nav2 with the FMS parameter overlay. Gazebo is left running.
# The Pinky manufacturer tree is not modified. Requires the simulator to
# already be up on ROS domain 73.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PARAMS="$ROOT/simulation/config/nav2/fms_nav2_params.yaml"
if [[ ! -f "$PARAMS" ]]; then
  echo "missing $PARAMS" >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/pinky_pro/install/setup.bash
set -u
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

POSE_FILE="$(mktemp)"
python3 - "$POSE_FILE" << 'PY'
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
import math
import time

rclpy.init()
node = Node("fms_capture_pose", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
buf = Buffer()
TransformListener(buf, node, spin_thread=False)
deadline = time.monotonic() + 8.0
pose = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    except Exception:
        continue
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    pose = (tf.transform.translation.x, tf.transform.translation.y, yaw)
    break
node.destroy_node()
rclpy.shutdown()
if pose is None:
    raise SystemExit("map pose was not available; Nav2 was not restarted")
open(sys.argv[1], "w").write(f"{pose[0]} {pose[1]} {pose[2]}\n")
print(f"captured {pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f}")
PY

# Stop only the Nav2 launch. Gazebo's launch_sim process stays up.
if pgrep -f "pinky_navigation gz_bringup_launch.xml" >/dev/null; then
  pkill -f "pinky_navigation gz_bringup_launch.xml" || true
fi
# The container is reparented to init when the launch process exits, so stop it by name.
if pgrep -f "component_container_isolated --ros-args --log-level info --ros-args -r __node:=nav2_container" >/dev/null; then
  pkill -f "component_container_isolated --ros-args --log-level info --ros-args -r __node:=nav2_container" || true
fi
for _ in $(seq 1 40); do
  if ! pgrep -f "__node:=nav2_container" >/dev/null; then
    break
  fi
  sleep 0.25
done
if pgrep -f "__node:=nav2_container" >/dev/null; then
  echo "nav2 container is still running" >&2
  exit 1
fi

read -r PX PY PYAW < "$POSE_FILE"
rm -f "$POSE_FILE"
SESSION="pinky-nav2"
tmux -f /exec-daemon/tmux.portal.conf has-session -t "=$SESSION" 2>/dev/null || tmux -f /exec-daemon/tmux.portal.conf new-session -d -s "$SESSION" -c "$ROOT"
tmux -f /exec-daemon/tmux.portal.conf send-keys -t "$SESSION:0.0" C-c
sleep 0.3
tmux -f /exec-daemon/tmux.portal.conf send-keys -t "$SESSION:0.0" \
  "set +u; source /opt/ros/jazzy/setup.bash; source /home/ubuntu/pinky_pro/install/setup.bash; set -u; export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch pinky_navigation gz_bringup_launch.xml params_file:=$PARAMS" C-m

python3 - "$PX" "$PY" "$PYAW" << 'PY'
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

x, y, yaw = map(float, sys.argv[1:4])
rclpy.init()
node = Node("fms_initial_pose", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
buf = Buffer()
TransformListener(buf, node, spin_thread=False)
deadline = time.monotonic() + 60.0
sent = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if not sent and pub.get_subscription_count() > 0:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        pub.publish(msg)
        sent = True
        print("published initial pose")
    if sent:
        try:
            tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            continue
        dx = tf.transform.translation.x - x
        dy = tf.transform.translation.y - y
        if math.hypot(dx, dy) < 0.5:
            print(f"map tf {tf.transform.translation.x:.3f} {tf.transform.translation.y:.3f}")
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit(0)
print("map tf did not return near the captured pose", file=sys.stderr)
raise SystemExit(1)
PY
echo "nav2 overlay is up"
