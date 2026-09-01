#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 ]]; then echo "usage: $0 POLICY.onnx [duration]" >&2; exit 2; fi
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$root/build/mujoco_robot_control" --model "$root/models/creeper/creeper.xml" --onnx "$1" --mode ground-truth --command 0.05 0 0 --duration "${2:-10}" --headless
