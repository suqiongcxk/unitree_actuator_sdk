#!/usr/bin/env python3
"""通过第二终端向 robot_control 发送 WASDQE 速度命令。

脚本只写入命名 FIFO，不访问电机或任何硬件。
"""

import argparse
import errno
import os
import select
import signal
import stat
import sys
import termios
import time
import tty
from dataclasses import dataclass


@dataclass
class VelocityState:
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0


@dataclass
class SpeedSettings:
    vx: float
    vy: float
    yaw: float


@dataclass
class AxisActivity:
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0


def motion_axis(key: str):
    """返回运动按键对应的速度轴。"""
    key = key.lower()
    if key in ("w", "s"):
        return "vx"
    if key in ("a", "d"):
        return "vy"
    if key in ("q", "e"):
        return "yaw"
    return None


def expire_inactive_axes(state: VelocityState, activity: AxisActivity,
                         now: float, timeout_sec: float) -> bool:
    """将长时未收到按键事件的轴清零。"""
    changed = False
    for axis in ("vx", "vy", "yaw"):
        if (getattr(state, axis) != 0.0
                and now - getattr(activity, axis) >= timeout_sec):
            setattr(state, axis, 0.0)
            changed = True
    return changed


def apply_motion_key(key: str, state: VelocityState,
                     vx_speed: float, vy_speed: float,
                     yaw_speed: float) -> bool:
    """更新速度状态；返回该按键是否为有效运动按键。"""
    key = key.lower()
    if key == "w":
        state.vx = vx_speed
    elif key == "s":
        state.vx = -vx_speed
    elif key == "a":
        state.vy = vy_speed
    elif key == "d":
        state.vy = -vy_speed
    elif key == "q":
        state.yaw = yaw_speed
    elif key == "e":
        state.yaw = -yaw_speed
    elif key == " ":
        state.vx = state.vy = state.yaw = 0.0
    else:
        return False
    return True


def apply_speed_key(key: str, settings: SpeedSettings,
                    state: VelocityState) -> bool:
    """以0.1为步长调整速度幅值，并同步当前激活轴。"""
    if key in ("+", "="):
        settings.vx = min(1.0, round(settings.vx + 0.1, 2))
        settings.vy = min(1.0, round(settings.vy + 0.1, 2))
    elif key in ("-", "_"):
        settings.vx = max(0.1, round(settings.vx - 0.1, 2))
        settings.vy = max(0.1, round(settings.vy - 0.1, 2))
    elif key == "]":
        settings.yaw = min(1.0, round(settings.yaw + 0.1, 2))
    elif key == "[":
        settings.yaw = max(0.1, round(settings.yaw - 0.1, 2))
    else:
        return False

    # 如果该轴正在运动，立即更新其目标幅值；主进程仍会做斜率限制。
    if state.vx != 0.0:
        state.vx = settings.vx if state.vx > 0.0 else -settings.vx
    if state.vy != 0.0:
        state.vy = settings.vy if state.vy > 0.0 else -settings.vy
    if state.yaw != 0.0:
        state.yaw = settings.yaw if state.yaw > 0.0 else -settings.yaw
    return True


def write_fifo_no_create(path: str, command: str) -> None:
    """原子写入已存在的FIFO；绝不创建同名普通文件。"""
    info = os.lstat(path)
    if not stat.S_ISFIFO(info.st_mode):
        raise RuntimeError(f"命令路径不是FIFO: {path}")

    flags = os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags)  # 故意不使用 O_CREAT。
    try:
        payload = (command + "\n").encode("ascii")
        if len(payload) > 4096:
            raise RuntimeError("命令过长")
        if os.write(fd, payload) != len(payload):
            raise RuntimeError("命令未完整写入")
    finally:
        os.close(fd)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WASDQE 四足机器人速度控制台")
    parser.add_argument("--fifo", default="/tmp/creeper_velocity_command.fifo")
    parser.add_argument("--vx", type=float, default=0.30,
                        help="W/S 前后速度幅值 m/s (默认: 0.30)")
    parser.add_argument("--vy", type=float, default=0.20,
                        help="A/D 侧向速度幅值 m/s (默认: 0.20)")
    parser.add_argument("--yaw", type=float, default=0.40,
                        help="Q/E 偏航角速度幅值 rad/s (默认: 0.40)")
    parser.add_argument("--refresh-ms", type=int, default=100,
                        help="命令刷新周期 ms (默认: 100)")
    parser.add_argument("--deadman-ms", type=int, default=650,
                        help="无按键后单轴自动回零时间 ms (默认: 650)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    settings = SpeedSettings(args.vx, args.vy, args.yaw)
    if any(not (0.0 < value <= 1.0)
           for value in (settings.vx, settings.vy, settings.yaw)):
        print("错误: --vx/--vy/--yaw 必须在 (0, 1] 内。", file=sys.stderr)
        return 2
    if not (20 <= args.refresh_ms <= 400):
        print("错误: --refresh-ms 必须在 [20, 400] ms 内。", file=sys.stderr)
        return 2
    if not (200 <= args.deadman_ms <= 1500):
        print("错误: --deadman-ms 必须在 [200, 1500] ms 内。", file=sys.stderr)
        return 2
    if not sys.stdin.isatty():
        print("错误: WASDQE控制台必须在交互式终端运行。", file=sys.stderr)
        return 2

    try:
        info = os.lstat(args.fifo)
        if not stat.S_ISFIFO(info.st_mode):
            raise RuntimeError(f"命令路径不是FIFO: {args.fifo}")
    except FileNotFoundError:
        print(f"命令FIFO不存在: {args.fifo}", file=sys.stderr)
        print("请先启动 robot_control。", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    stop_requested = False
    interrupted = False

    def handle_signal(_signum, _frame):
        nonlocal stop_requested, interrupted
        interrupted = True
        stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    state = VelocityState()
    activity = AxisActivity()
    stdin_fd = sys.stdin.fileno()
    saved_terminal = termios.tcgetattr(stdin_fd)
    emergency_sent = False
    connection_lost = False

    print(f"WASDQE速度控制台 -> {args.fifo}")
    print(f"  W/S: 前后 {settings.vx:.2f} m/s    A/D: 左右 {settings.vy:.2f} m/s")
    print(f"  Q/E: 左右转 {settings.yaw:.2f} rad/s")
    print("  +/-: 前后与左右速度 ±0.1 m/s    ]/[: 转向速度 ±0.1 rad/s")
    print("  空格: 平滑回零    X: 统一安全停机    Ctrl+C: 统一安全停机")
    print(f"  死手保护: 任一轴超过 {args.deadman_ms} ms 没有按键事件就自动平滑回零。")
    print("  持续按住按键依赖终端键盘重复；空格可立即请求全轴回零。")

    try:
        tty.setcbreak(stdin_fd)
        next_refresh = time.monotonic()
        while not stop_requested:
            now = time.monotonic()
            timeout = max(0.0, next_refresh - now)
            readable, _, _ = select.select([stdin_fd], [], [], timeout)
            if readable:
                key = os.read(stdin_fd, 1).decode("utf-8", errors="ignore")
                if not key:
                    break
                if key.lower() == "x":
                    write_fifo_no_create(args.fifo, "s")
                    emergency_sent = True
                    stop_requested = True
                    continue
                if apply_motion_key(
                        key, state, settings.vx, settings.vy, settings.yaw):
                    key_time = time.monotonic()
                    axis = motion_axis(key)
                    if axis is not None:
                        setattr(activity, axis, key_time)
                    else:
                        activity = AxisActivity()
                    if key == " ":
                        write_fifo_no_create(args.fifo, "z")
                    else:
                        write_fifo_no_create(
                            args.fifo,
                            f"v {state.vx:.6f} {state.vy:.6f} {state.yaw:.6f}")
                    print(f"\rCMD vx={state.vx:+.2f} vy={state.vy:+.2f} "
                          f"yaw={state.yaw:+.2f}      ", end="", flush=True)
                    next_refresh = time.monotonic() + args.refresh_ms / 1000.0
                elif apply_speed_key(key, settings, state):
                    if state.vx != 0.0 or state.vy != 0.0 or state.yaw != 0.0:
                        write_fifo_no_create(
                            args.fifo,
                            f"v {state.vx:.6f} {state.vy:.6f} {state.yaw:.6f}")
                    print(f"\rSPEED linear=({settings.vx:.2f},{settings.vy:.2f})m/s "
                          f"yaw={settings.yaw:.2f}rad/s | "
                          f"CMD=({state.vx:+.2f},{state.vy:+.2f},{state.yaw:+.2f})      ",
                          end="", flush=True)
                    next_refresh = time.monotonic() + args.refresh_ms / 1000.0

            now = time.monotonic()
            if expire_inactive_axes(
                    state, activity, now, args.deadman_ms / 1000.0):
                if state.vx == 0.0 and state.vy == 0.0 and state.yaw == 0.0:
                    write_fifo_no_create(args.fifo, "z")
                else:
                    write_fifo_no_create(
                        args.fifo,
                        f"v {state.vx:.6f} {state.vy:.6f} {state.yaw:.6f}")
                print(f"\rDEADMAN vx={state.vx:+.2f} vy={state.vy:+.2f} "
                      f"yaw={state.yaw:+.2f}      ", end="", flush=True)
                next_refresh = now + args.refresh_ms / 1000.0

            if now >= next_refresh:
                if state.vx != 0.0 or state.vy != 0.0 or state.yaw != 0.0:
                    # 不带hold_ms：脚本掉线后主进程500 ms看门狗会自动回零。
                    write_fifo_no_create(
                        args.fifo,
                        f"v {state.vx:.6f} {state.vy:.6f} {state.yaw:.6f}")
                next_refresh = now + args.refresh_ms / 1000.0
    except FileNotFoundError:
        connection_lost = True
        print("\n命令FIFO已被主进程删除。", file=sys.stderr)
    except RuntimeError as exc:
        connection_lost = True
        print(f"\nFIFO通信失败: {exc}", file=sys.stderr)
    except OSError as exc:
        connection_lost = True
        if exc.errno == errno.ENXIO:
            print("\nFIFO已无主控读取端。", file=sys.stderr)
        else:
            print(f"\nFIFO通信失败: {exc}", file=sys.stderr)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, saved_terminal)
        if interrupted and not emergency_sent and not connection_lost:
            try:
                write_fifo_no_create(args.fifo, "s")
                emergency_sent = True
            except (OSError, RuntimeError, FileNotFoundError):
                pass
        elif not emergency_sent and not connection_lost:
            try:
                write_fifo_no_create(args.fifo, "z")
            except (OSError, RuntimeError, FileNotFoundError):
                pass
        print()

    return 1 if connection_lost else 0


if __name__ == "__main__":
    raise SystemExit(main())
