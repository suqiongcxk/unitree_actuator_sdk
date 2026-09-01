#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ctest --test-dir "$root/build" --output-on-failure
if command -v strace >/dev/null; then
  trace="$(mktemp)"; trap 'rm -f "$trace"' EXIT
  strace -f -e trace=openat -o "$trace" "$root/build/mujoco_robot_control" --model "$root/models/creeper/creeper.xml" --headless --duration .1 --no-policy >/dev/null
  if grep -E '/dev/(tty|i2c-|gpio)' "$trace"; then echo "hardware device access detected" >&2; exit 1; fi
fi
echo "smoke test: PASS"
