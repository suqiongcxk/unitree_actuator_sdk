#!/usr/bin/env python3
"""Read validated velocity commands from SSH stdin and forward them to FIFO.

This process never opens motor devices.  It intentionally never creates the
FIFO, so losing robot_control cannot leave a command-looking regular file.
"""

import argparse
import math
import os
import re
import stat
import sys


VELOCITY_RE = re.compile(
    r"^v ([+-]?(?:\d+(?:\.\d*)?|\.\d+)) "
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)) "
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+))$")


def validate_command(line: str) -> str:
    command = line.strip()
    if command in ("z", "s"):
        return command
    match = VELOCITY_RE.fullmatch(command)
    if match is None:
        raise ValueError("invalid command syntax")
    values = tuple(float(value) for value in match.groups())
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0
               for value in values):
        raise ValueError("velocity outside [-1,1] or not finite")
    return "v {:.6f} {:.6f} {:.6f}".format(*values)


def write_fifo_no_create(path: str, command: str) -> None:
    info = os.lstat(path)
    if not stat.S_ISFIFO(info.st_mode):
        raise RuntimeError(f"command path is not a FIFO: {path}")
    flags = os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags)
    try:
        payload = (command + "\n").encode("ascii")
        if os.write(fd, payload) != len(payload):
            raise RuntimeError("incomplete FIFO write")
    finally:
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(description="SSH stdin to robot FIFO bridge")
    parser.add_argument("--fifo", default="/tmp/creeper_velocity_command.fifo")
    args = parser.parse_args()

    try:
        info = os.lstat(args.fifo)
        if not stat.S_ISFIFO(info.st_mode):
            raise RuntimeError(f"command path is not a FIFO: {args.fifo}")
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"BRIDGE_ERROR {exc}", flush=True)
        return 1

    print("BRIDGE_READY", flush=True)
    stop_sent = False
    try:
        for line in sys.stdin:
            try:
                command = validate_command(line)
            except ValueError as exc:
                print(f"BRIDGE_REJECT {exc}", flush=True)
                continue
            write_fifo_no_create(args.fifo, command)
            if command == "s":
                stop_sent = True
                return 0
    except (FileNotFoundError, OSError, RuntimeError) as exc:
        print(f"BRIDGE_ERROR {exc}", flush=True)
        return 1
    finally:
        # Normal EOF means the PC client or SSH link ended.  Ask for a smooth
        # zero immediately; robot_control's 500 ms watchdog remains the backup.
        if not stop_sent:
            try:
                write_fifo_no_create(args.fifo, "z")
            except (FileNotFoundError, OSError, RuntimeError):
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
