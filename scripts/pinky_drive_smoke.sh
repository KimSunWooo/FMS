#!/usr/bin/env bash
# Single Pinky short waypoint drive: restart FMS ROS path, dispatch, wait, cancel.
# Run on the host (not in an agent sandbox): bash scripts/pinky_drive_smoke.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT_ID="${FMS_RECEIVER_ROBOT_ID:-pinky-1}"
DOMAIN="${ROS_DOMAIN_ID:-24}"
DATA="${FMS_DATA_DIR:-$ROOT/data}"
LOG="$ROOT/logs"
API="${FMS_API_URL:-http://127.0.0.1:8741}"
mkdir -p "$LOG" "$DATA/ros_ipc/$ROBOT_ID/inbox" "$DATA/ros_ipc/$ROBOT_ID/execution" "$DATA/ros_home"
export ROS_HOME="${ROS_HOME:-$DATA/ros_home}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FMS_DATA_DIR="$DATA"
export FMS_RECEIVER_ROBOT_ID="$ROBOT_ID"
export FMS_WORKER_ROBOT_ID="$ROBOT_ID"
export FMS_RECEIVER_INBOX="$DATA/ros_ipc/$ROBOT_ID/inbox"
export FMS_TASK_EXECUTION_ENABLED=true
export FMS_FLEET_TASK_EXECUTE=true
export ROS_DOMAIN_ID="$DOMAIN"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros_ws/install/setup.bash"
set -u

echo "== prepare: Domain $DOMAIN nodes =="
ros2 node list | tee "$LOG/drive_nodes_before.txt" || true
ros2 action list | tee "$LOG/drive_actions_before.txt" || true
timeout 3 ros2 topic echo /amcl_pose --once | tee "$LOG/drive_amcl_before.txt" || true

start_bg() {
  local name="$1"; shift
  local out="$LOG/${name}.log"
  if pgrep -f "$name" >/dev/null 2>&1; then
    echo "reuse $name"
    return 0
  fi
  nohup "$@" >>"$out" 2>&1 &
  echo "started $name pid=$! log=$out"
}

# Gateway is normally started by domain_worker; start if missing.
if ! pgrep -f 'ros_gateway_node.py' >/dev/null 2>&1; then
  start_bg ros_gateway env FMS_WORKER_ROBOT_ID="$ROBOT_ID" ROS_DOMAIN_ID="$DOMAIN" \
    bash "$ROOT/scripts/run_ros_gateway.sh"
fi
start_bg fms_task_receiver env FMS_RECEIVER_ROBOT_ID="$ROBOT_ID" ROS_DOMAIN_ID="$DOMAIN" \
  bash "$ROOT/scripts/run_task_receiver.sh"
start_bg fms_nav2_executor env FMS_RECEIVER_ROBOT_ID="$ROBOT_ID" ROS_DOMAIN_ID="$DOMAIN" \
  bash "$ROOT/scripts/run_nav2_executor.sh"
sleep 2

echo "== live pose =="
POSE_JSON=$(curl -fsS "$API/api/fleet" | python3 - <<'PY'
import json,sys
raw=sys.stdin.read()
d=json.loads(raw)
robots=d if isinstance(d,list) else d.get("robots") or d.get("items") or []
for r in robots:
    if r.get("robot_id")=="pinky-1":
        print(json.dumps(r, ensure_ascii=False))
        break
else:
    print("{}")
PY
)
echo "$POSE_JSON" | tee "$LOG/drive_fleet_pinky.json"
SX=$(python3 -c 'import json,sys; r=json.loads(sys.argv[1]); p=r.get("pose") or {}; print(p.get("x",""))' "$POSE_JSON")
SY=$(python3 -c 'import json,sys; r=json.loads(sys.argv[1]); p=r.get("pose") or {}; print(p.get("y",""))' "$POSE_JSON")
if [[ -z "$SX" || -z "$SY" ]]; then
  echo "No live pose for pinky-1; abort before motion" >&2
  exit 3
fi
# Short offset ~0.25 m along +x in map frame (keep on free space; operator may edit).
GX=$(python3 -c "print(round(float('$SX')+0.25, 4))")
GY=$(python3 -c "print(round(float('$SY'), 4))")
echo "start=($SX,$SY) goal=($GX,$GY)"

echo "== create/plan/dispatch =="
TASK=$(curl -fsS -X POST "$API/api/fleet-tasks" -H 'Content-Type: application/json' \
  -d "{\"assignment_mode\":\"manual\",\"robot_id\":\"$ROBOT_ID\",\"task_type\":\"navigate\",\"label\":\"drive-smoke\",\"goal\":{\"x\":$GX,\"y\":$GY},\"start\":{\"x\":$SX,\"y\":$SY}}")
echo "$TASK" | tee "$LOG/drive_task_create.json"
TID=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["task_id"])' <<<"$TASK")
PLANNED=$(curl -fsS -X POST "$API/api/fleet-tasks/$TID/plan")
echo "$PLANNED" | tee "$LOG/drive_task_plan.json"
DISPATCHED=$(curl -fsS -X POST "$API/api/fleet-tasks/$TID/dispatch")
echo "$DISPATCHED" | tee "$LOG/drive_task_dispatch.json"
echo "task_id=$TID command/ack:" 
python3 - <<PY
import json
d=json.load(open("$LOG/drive_task_dispatch.json"))
ack=d.get("ack") or {}
print("status", d.get("status"), "ack_status", d.get("ack_status"))
print("command_id", ack.get("command_id"))
print("payload_hash", d.get("payload_hash") or ack.get("payload_hash"))
print("execute", (json.loads(d.get("schedule_json") or "{}").get("payload") if False else None))
print("transport", d.get("transport"))
print("ack.execute", ack.get("execute"), "ack.status", ack.get("status"))
PY
# Prefer payload from plan file
python3 - <<PY
import json
from pathlib import Path
p=Path("$LOG/drive_task_plan.json")
d=json.loads(p.read_text())
sched=d.get("schedule") or {}
# schedule may be nested differently in public view
payload=None
for key in ("payload","schedule"):
    pass
# public task often has payload_hash at top and planned path
print("payload_hash", d.get("payload_hash"))
print("status", d.get("status"))
inbox=Path("$DATA/ros_ipc/$ROBOT_ID/inbox")/f"{d.get('task_id')}.json"
print("inbox_exists", inbox.is_file(), inbox)
if inbox.is_file():
    stored=json.loads(inbox.read_text())
    print("stored.execute", (stored.get("payload") or {}).get("execute"))
    print("stored.ack", stored.get("ack"))
PY

echo "== wait for Nav2 execution (up to 90s) =="
for i in $(seq 1 45); do
  EXEC="$DATA/ros_ipc/$ROBOT_ID/execution/$TID.json"
  if [[ -f "$EXEC" ]]; then
    cp "$EXEC" "$LOG/drive_execution.json"
    STATUS=$(python3 -c 'import json; print(json.load(open("'"$EXEC"'")).get("status"))')
    echo "t=${i}s execution.status=$STATUS"
    if [[ "$STATUS" == "executing" || "$STATUS" == "accepted" ]]; then
      echo "Nav2 goal accepted/executing — cancel test"
      curl -fsS -X POST "$API/api/fleet-tasks/$TID/cancel" -H 'Content-Type: application/json' \
        -d '{"reason":"drive-smoke-cancel"}' | tee "$LOG/drive_task_cancel.json"
      sleep 3
      cp "$EXEC" "$LOG/drive_execution_after_cancel.json" || true
      break
    fi
    if [[ "$STATUS" == "succeeded" || "$STATUS" == "failed" || "$STATUS" == "cancelled" ]]; then
      break
    fi
  else
    echo "t=${i}s waiting execution ipc"
  fi
  sleep 2
done

timeout 3 ros2 topic echo /amcl_pose --once | tee "$LOG/drive_amcl_after.txt" || true
echo "== done; artifacts under $LOG/drive_*.json =="
ls -la "$LOG"/drive_* 2>/dev/null || true
