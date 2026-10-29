#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$workspace_dir/src/virtual_control/scripts/mavproxy_real_compat.py" \
  --master=/dev/ttyACM0,115200 \
  --out=127.0.0.1:14540 \
  --out=127.0.0.1:14541 \
  --out=192.168.0.33:14550 "$@"
