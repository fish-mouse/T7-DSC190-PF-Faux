#!/usr/bin/env bash
# Wrapper invoked by container entrypoint to run the harness.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "${SCRIPT_DIR}/container_runner.py" "$@"