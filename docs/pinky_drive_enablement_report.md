# Pinky waypoint drive enablement (2026-10-30)

## Goal
JSON → Gateway (Domain 24) → Receiver store+ACK → Nav2 Executor → motion → FMS result.

## Blockers removed / unified

| Previous block | Change |
|---|---|
| `Settings` refuses `execution_mode=physical` | Allowed; `.env` sets `FMS_EXECUTION_MODE=physical` |
| `canonical_payload` / codec / gateway force `execute=false` | Single flag `FMS_FLEET_TASK_EXECUTE` (default true); hash then never rewrite |
| Receiver refuses `execute=true` | Stores + receive ACK; Executor runs Nav2 |
| Gateway overwrites `execute` / `task_execution_enabled` | Pass-through |
| `receiver_ready` required `task_execution_enabled=false` | Ready if fresh status; execute flag independent |
| Map identity requires identical absolute YAML path | Geometry + occupancy content match is enough |
| Stale `/amcl_pose` masking fresh TF | Fresh TF preferred; stale amcl is last resort |
| No in-repo Executor | Added `robot_receiver/nav2_executor_node.py` (`NavigateThroughPoses`, all waypoints) |
| Gateway child stderr discarded | Logged under `logs/ros_gateway_<robot>.log` |

## Role split
- **fms_task_receiver**: validate, store inbox, publish receive `TaskAck` (`status=stored`) only
- **fms_nav2_executor**: poll inbox `execute=true`, send `navigate_through_poses`, write `ros_ipc/<robot>/execution/<task_id>.json` (no TaskAck)
- Cancel: Receiver cancel ACK + Executor cancels active Nav2 goal when inbox marked non-executable

## Files touched (primary)
- `backend/app/core/config.py`
- `backend/app/services/{fleet_task_service,ros_task_gateway,domain_worker,map_identity,robot_state}.py`
- `robot_receiver/{task_codec,pinky_task_receiver,ros_messages,ros_gateway_node,ros_task_node,nav2_executor_node}.py`
- `scripts/{run_task_receiver,run_ros_gateway,run_nav2_executor,pinky_drive_smoke}.sh`
- `.env`: `FMS_FLEET_TASK_EXECUTE=true`, `FMS_EXECUTION_MODE=physical`, `FMS_TASK_EXECUTION_ENABLED=true`

## Agent host access (this session)
Live drive from the Cursor agent sandbox was **blocked**:
- `curl 127.0.0.1:8741` → connection refused (host uvicorn netns not reachable)
- `ros2 node list` on Domain 24 → empty (DDS / network namespace)
- `/proc/<pid>/environ` → Permission denied for host processes
- Host still shows uvicorn:8741, gateway, map_server/amcl/Nav2 PIDs via `ps`

## Operator: run the smoke drive on the host

```bash
# 1) Restart API so Settings + code load (from a host terminal)
pkill -f 'uvicorn app.main:app' || true
cd /home/sw/DevSpace/FMS
source .venv/bin/activate
# keep existing .env
uvicorn app.main:app --app-dir backend --host 0.0.0.0 --port 8741

# 2) Confirm Domain 24 Nav2 + TF (second terminal)
export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_HOME=/home/sw/DevSpace/FMS/data/ros_home
source /opt/ros/jazzy/setup.bash
source /home/sw/DevSpace/FMS/ros_ws/install/setup.bash
ros2 node list
ros2 action list   # expect /navigate_through_poses /navigate_to_pose
timeout 3 ros2 topic echo /amcl_pose --once

# 3) Receiver + Executor (if domain_worker did not already start gateway)
export FMS_RECEIVER_ROBOT_ID=pinky-1 ROS_DOMAIN_ID=24 FMS_DATA_DIR=/home/sw/DevSpace/FMS/data
bash /home/sw/DevSpace/FMS/scripts/run_task_receiver.sh
# other terminal:
bash /home/sw/DevSpace/FMS/scripts/run_nav2_executor.sh

# 4) Short WP + cancel
bash /home/sw/DevSpace/FMS/scripts/pinky_drive_smoke.sh
# Artifacts: logs/drive_*.json — distinguish Publish/ACK/Goal accept/motion/cancel
```

## Evidence checklist (fill after smoke)
- [ ] Publish success (IPC command consumed)
- [ ] Receive ACK (`status=stored`, matching task/command/hash)
- [ ] Inbox JSON `execute=true`
- [ ] Nav2 Goal accept (`execution.status` accepted/executing)
- [ ] Pose change on `/amcl_pose` or TF
- [ ] Cancel ACK + Nav2 cancel / `execution.status=cancelled|succeeded|failed`

## Known remaining
- Restart host uvicorn/gateway to load new Python (old gateway from 12:18 still running until restart)
- Confirm `/amcl_pose` freshness before motion (IPC sample was stale at ~14:03 while battery still updated)
- Agent cannot complete live motion until host network/DDS is reachable from the agent runtime
