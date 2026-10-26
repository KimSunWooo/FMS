#!/usr/bin/env bash
# Run one robot agent. Do not source this file. It does not edit shell startup
# files and it does not export ROS_DOMAIN_ID into the parent shell.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  python3 -m venv "$ROOT/.venv"
  "$ROOT/.venv/bin/pip" install -e "$ROOT/backend[dev]"
fi
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
# A sourced Jazzy environment can import rclpy. The project venv cannot, and
# the unit tests rely on that. Do not source ROS from this script.
if python3 -c 'import rclpy' >/dev/null 2>&1; then
  PY="$(command -v python3)"
else
  PY="$ROOT/.venv/bin/python"
fi
exec "$PY" -m robot_agent.agent_main --expected-domain 73 "$@"
