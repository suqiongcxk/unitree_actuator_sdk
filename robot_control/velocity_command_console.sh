#!/usr/bin/env bash
set -u

fifo_path="${1:-/tmp/creeper_velocity_command.fifo}"
if [[ ! -p "$fifo_path" ]]; then
    echo "命令FIFO不存在: $fifo_path"
    echo "请先启动 robot_control。"
    exit 1
fi

echo "速度命令控制台 -> $fifo_path"
echo "  v <vx> <vy> <yaw_rate> [hold_ms]"
echo "  z  平滑回零"
echo "  s  请求主控制进程安全停机"

while IFS= read -r -p "robot> " command; do
    [[ -z "$command" ]] && continue
    # 控制进程异常消失时，最多等待1秒，避免被残留FIFO永久阻塞。
    if ! printf '%s\n' "$command" | timeout 1s tee "$fifo_path" >/dev/null; then
        echo "发送失败：robot_control可能已经退出。"
        exit 1
    fi
    [[ "$command" == "s" || "$command" == "S" ]] && break
done
