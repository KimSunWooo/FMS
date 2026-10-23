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
exec "$ROOT/.venv/bin/python" -m robot_agent.agent_main --expected-domain 73 "$@"
