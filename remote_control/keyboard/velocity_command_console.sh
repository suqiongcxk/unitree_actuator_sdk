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

send_fifo_command() {
    # 不使用tee/重定向：它们在主进程刚好删除FIFO时可能
    # 重新创建同名普通文件。这里不带O_CREAT打开，竞态时只会失败。
    python3 - "$fifo_path" "$1" <<'PY'
import errno
import os
import stat
import sys

path, command = sys.argv[1], sys.argv[2]
fd = -1
try:
    info = os.lstat(path)
    if not stat.S_ISFIFO(info.st_mode):
        raise RuntimeError("命令路径不是FIFO")
    flags = os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    # 故意不传O_CREAT；FIFO被主进程删除后绝不会留下普通文件。
    fd = os.open(path, flags)
    payload = (command + "\n").encode("utf-8")
    if len(payload) > 4096:
        raise RuntimeError("命令过长")
    if os.write(fd, payload) != len(payload):
        raise RuntimeError("命令未完整写入")
except FileNotFoundError:
    print("命令FIFO已被主进程删除。", file=sys.stderr)
    sys.exit(1)
except OSError as exc:
    if exc.errno == errno.ENXIO:
        print("FIFO已无主控读取端。", file=sys.stderr)
    else:
        print(f"FIFO写入失败: {exc}", file=sys.stderr)
    sys.exit(1)
except RuntimeError as exc:
    print(str(exc), file=sys.stderr)
    sys.exit(1)
finally:
    if fd >= 0:
        os.close(fd)
PY
}

while IFS= read -r -p "robot> " command; do
    [[ -z "$command" ]] && continue
    if ! send_fifo_command "$command"; then
        echo "发送失败：robot_control可能已经退出。"
        exit 1
    fi
    [[ "$command" == "s" || "$command" == "S" ]] && break
done
