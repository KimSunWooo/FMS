#!/usr/bin/env bash
# Run as an executable. Do not source this file.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ ! -x "$ROOT/.venv/bin/uvicorn" ]]; then
  python3 -m venv "$ROOT/.venv"
  "$ROOT/.venv/bin/pip" install -e "$ROOT/backend[dev]"
fi
if [[ ! -f "$ROOT/.env" ]]; then
  secret="$("$ROOT/.venv/bin/python" -c 'import secrets; print(secrets.token_hex(32))')"
  cat > "$ROOT/.env" <<EOF
FMS_ENV=development
FMS_ENVIRONMENT_ID=care-move-fms-local
FMS_LEASE_SECRET=${secret}
FMS_DATABASE_URL=sqlite:///${ROOT}/data/fms.sqlite3
FMS_ROS_DOMAIN_ID=73
FMS_API_HOST=0.0.0.0
FMS_API_PORT=8741
FMS_PHYSICAL_CONTROL_ENABLED=false
FMS_ROS_BRIDGE_ENABLED=false
FMS_AUTO_TICK=true
FMS_SEED_FLEET=true
EOF
  echo "Wrote .env with a new local lease secret. It is not copied from Care-Move."
fi
mkdir -p "$ROOT/data" "$ROOT/logs"
exec "$ROOT/.venv/bin/uvicorn" app.main:app --app-dir "$ROOT/backend" --host 0.0.0.0 --port 8741
