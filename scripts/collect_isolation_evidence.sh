#!/usr/bin/env bash
# Read-only. Does not kill processes, change ROS state, or touch another git repo.
set -u
echo "collected_at=$(date -Is)"
echo "hostname=$(hostname)"
echo "pwd=$(pwd)"
echo "branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
echo "--- legacy path probe ---"
ls -ld /home/sw /home/sw/DevSpace /home/sw/DevSpace/care-move-rmf-starter /home/sw/DevSpace/care-move-fms 2>&1 || true
echo "--- report probe ---"
find /workspace /home /opt /tmp -iname '*Care_Move*' -o -iname '*care-move-rmf*' 2>/dev/null | head -50 || true
echo "--- ros/rmf processes ---"
ps aux | grep -E 'ros2|rmw|nav2|gazebo|ign gazebo|rmf' | grep -v grep || echo "none"
echo "--- ros/rmf environment ---"
env | grep -Ei '^(ROS_|RMW_|FASTRTPS|CYCLONEDDS)' || echo "none"
echo "--- tool presence ---"
command -v ros2 || echo "ros2 missing"
command -v docker || echo "docker missing"
echo "--- listeners ---"
ss -ltnp 2>/dev/null || true
