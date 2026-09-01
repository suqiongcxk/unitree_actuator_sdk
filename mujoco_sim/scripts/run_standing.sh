#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$root/build/mujoco_robot_control" --model "$root/models/creeper/creeper.xml" --headless --duration "${1:-10}" --no-policy --keyframe standing
