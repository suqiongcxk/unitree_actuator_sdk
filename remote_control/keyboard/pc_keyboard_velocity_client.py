#!/usr/bin/env python3
"""Windows PC keyboard client for simultaneous quadruped velocity control.

Run this file on the PC connected to the keyboard, not inside an SSH terminal.
It polls Windows key-down state and sends complete vx/vy/yaw commands through
one persistent SSH process to ssh_velocity_command_bridge.py on the RK3588.
"""

import argparse
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, Tuple


MIN_VX_SPEED = 0.2
MAX_VX_SPEED = 1.0
MIN_VY_SPEED = 0.2
MAX_VY_SPEED = 0.3
MIN_YAW_SPEED = 0.5
MAX_YAW_SPEED = 1.0
SPEED_STEP = 0.1


@dataclass
class SpeedSettings:
    vx: float = MIN_VX_SPEED
    vy: float = MIN_VY_SPEED
    yaw: float = MIN_YAW_SPEED


def clamp_step(value: float, delta: float,
               minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, round(value + delta, 2)))


def compose_velocity(keys: Dict[str, bool],
                     speeds: SpeedSettings) -> Tuple[float, float, float]:
    """Convert simultaneous key state into one complete body-frame command."""
    vx_direction = int(keys.get("w", False)) - int(keys.get("s", False))
    vy_direction = int(keys.get("a", False)) - int(keys.get("d", False))
    yaw_direction = int(keys.get("q", False)) - int(keys.get("e", False))
    return (vx_direction * speeds.vx,
            vy_direction * speeds.vy,
            yaw_direction * speeds.yaw)


def apply_speed_edge(name: str, speeds: SpeedSettings) -> None:
    if name == "vx_up":
        speeds.vx = clamp_step(speeds.vx, SPEED_STEP,
                               MIN_VX_SPEED, MAX_VX_SPEED)
    elif name == "vx_down":
        speeds.vx = clamp_step(speeds.vx, -SPEED_STEP,
                               MIN_VX_SPEED, MAX_VX_SPEED)
    elif name == "vy_up":
        speeds.vy = clamp_step(speeds.vy, SPEED_STEP,
                               MIN_VY_SPEED, MAX_VY_SPEED)
    elif name == "vy_down":
        speeds.vy = clamp_step(speeds.vy, -SPEED_STEP,
                               MIN_VY_SPEED, MAX_VY_SPEED)
    elif name == "yaw_up":
        speeds.yaw = clamp_step(speeds.yaw, SPEED_STEP,
                                MIN_YAW_SPEED, MAX_YAW_SPEED)
    elif name == "yaw_down":
        speeds.yaw = clamp_step(speeds.yaw, -SPEED_STEP,
                                MIN_YAW_SPEED, MAX_YAW_SPEED)


def wait_for_bridge(process: subprocess.Popen) -> None:
    if process.stdout is None:
        raise RuntimeError("SSH stdout is unavailable")
    while True:
        line = process.stdout.readline()
        if line == "":
            raise RuntimeError(
                f"SSH bridge exited before READY (code={process.poll()})")
        line = line.strip()
        if line == "BRIDGE_READY":
            return
        if line.startswith("BRIDGE_ERROR"):
            raise RuntimeError(line)


def send_line(process: subprocess.Popen, command: str) -> None:
    if process.stdin is None:
        raise RuntimeError("SSH stdin is unavailable")
    process.stdin.write(command + "\n")
    process.stdin.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Windows simultaneous-key robot velocity client")
    parser.add_argument("--host", default="orangepi@192.168.31.100")
    parser.add_argument(
        "--remote-root", default="/home/orangepi/unitree_actuator_sdk")
    parser.add_argument("--fifo", default="/tmp/creeper_velocity_command.fifo")
    parser.add_argument("--vx", type=float, default=MIN_VX_SPEED)
    parser.add_argument("--vy", type=float, default=MIN_VY_SPEED)
    parser.add_argument("--yaw", type=float, default=MIN_YAW_SPEED)
    parser.add_argument("--refresh-ms", type=int, default=100)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not MIN_VX_SPEED <= args.vx <= MAX_VX_SPEED:
        raise ValueError("--vx must be in [0.2, 1.0] m/s")
    if not MIN_VY_SPEED <= args.vy <= MAX_VY_SPEED:
        raise ValueError("--vy must be in [0.2, 0.3] m/s")
    if not MIN_YAW_SPEED <= args.yaw <= MAX_YAW_SPEED:
        raise ValueError("--yaw must be in [0.5, 1.0] rad/s")
    if not 50 <= args.refresh_ms <= 400:
        raise ValueError("--refresh-ms must be in [50, 400]")


def windows_key_reader() -> Tuple[Callable[[int], bool], Dict[str, int]]:
    if os.name != "nt":
        raise RuntimeError(
            "this client must run in a local Windows PowerShell/Terminal")
    import ctypes

    get_async_key_state = ctypes.windll.user32.GetAsyncKeyState

    def is_down(vk: int) -> bool:
        return bool(get_async_key_state(vk) & 0x8000)

    # Letters use ASCII virtual-key values. OEM punctuation values are stable
    # for GetAsyncKeyState and do not depend on whether Shift is held.
    virtual_keys = {
        "w": ord("W"), "s": ord("S"),
        "a": ord("A"), "d": ord("D"),
        "q": ord("Q"), "e": ord("E"),
        "stop": ord("X"),
        "zero": 0x20,       # Space
        "exit": 0x1B,       # Escape
        "vx_up": 0xBB,      # = / +
        "vx_down": 0xBD,    # - / _
        "vy_up": 0xBE,      # . / >
        "vy_down": 0xBC,    # , / <
        "yaw_up": 0xDD,     # ] / }
        "yaw_down": 0xDB,   # [ / {
    }
    return is_down, virtual_keys


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        is_down, virtual_keys = windows_key_reader()
    except (ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    bridge_path = (
        f"{args.remote_root}/remote_control/keyboard/"
        "ssh_velocity_command_bridge.py")
    remote_command = " ".join([
        "python3", shlex.quote(bridge_path),
        "--fifo", shlex.quote(args.fifo),
    ])
    process = subprocess.Popen(
        ["ssh", "-T", args.host, remote_command],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        text=True, bufsize=1)

    try:
        print("Waiting for SSH authentication and RK3588 FIFO bridge...")
        wait_for_bridge(process)
    except (RuntimeError, OSError) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        process.terminate()
        return 1

    speeds = SpeedSettings(args.vx, args.vy, args.yaw)
    previous_edges = {name: False for name in (
        "vx_up", "vx_down", "vy_up", "vy_down",
        "yaw_up", "yaw_down")}
    previous_command = None
    refresh_sec = args.refresh_ms / 1000.0
    next_send = 0.0

    print("Connected. Key capture is now active.")
    print("W/S forward/back, A/D lateral, Q/E yaw; combinations supported")
    print("+/- vx, >/< vy, ]/[ yaw; Space zero; X E-stop; Esc safe exit")
    print(f"speed vx={speeds.vx:.1f} vy={speeds.vy:.1f} "
          f"yaw={speeds.yaw:.1f}")

    try:
        while process.poll() is None:
            if is_down(virtual_keys["stop"]):
                send_line(process, "s")
                print("\nE-stop requested.")
                return 0
            if is_down(virtual_keys["exit"]):
                send_line(process, "z")
                print("\nZero requested; client exiting.")
                return 0

            speed_changed = False
            for name in previous_edges:
                current = is_down(virtual_keys[name])
                if current and not previous_edges[name]:
                    apply_speed_edge(name, speeds)
                    speed_changed = True
                previous_edges[name] = current
            if speed_changed:
                print(f"\nspeed vx={speeds.vx:.1f} vy={speeds.vy:.1f} "
                      f"yaw={speeds.yaw:.1f}")

            motion_keys = {
                name: is_down(virtual_keys[name])
                for name in ("w", "s", "a", "d", "q", "e")
            }
            command = compose_velocity(motion_keys, speeds)
            if is_down(virtual_keys["zero"]):
                command = (0.0, 0.0, 0.0)

            now = time.monotonic()
            changed = command != previous_command
            moving = any(abs(value) > 1.0e-9 for value in command)
            if changed and not moving:
                send_line(process, "z")
            elif changed or (moving and now >= next_send):
                send_line(process, "v {:.6f} {:.6f} {:.6f}".format(*command))
            if changed:
                print("\rCMD vx={:+.1f} vy={:+.1f} yaw={:+.1f}      ".format(
                    *command), end="", flush=True)
            if changed or now >= next_send:
                next_send = now + refresh_sec
            previous_command = command
            time.sleep(0.02)
    except KeyboardInterrupt:
        try:
            send_line(process, "s")
            print("\nCtrl+C: E-stop requested.")
        except (BrokenPipeError, OSError, RuntimeError):
            pass
        return 0
    except (BrokenPipeError, OSError, RuntimeError) as exc:
        print(f"\nSSH command channel lost: {exc}", file=sys.stderr)
        return 1
    finally:
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()

    return 1


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    raise SystemExit(main())
