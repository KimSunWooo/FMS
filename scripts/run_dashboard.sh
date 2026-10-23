#!/usr/bin/env bash
# Run as an executable. Do not source this file.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/frontend"
if [[ ! -d node_modules ]]; then
  npm install
fi
export FMS_API_ORIGIN="${FMS_API_ORIGIN:-http://127.0.0.1:8741}"
exec npm run dev
