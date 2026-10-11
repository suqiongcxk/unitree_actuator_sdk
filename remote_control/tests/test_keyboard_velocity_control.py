#!/usr/bin/env python3

import importlib.util
from pathlib import Path


REMOTE_CONTROL = Path(__file__).resolve().parents[1]
KEYBOARD = REMOTE_CONTROL / "keyboard"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


client = load_module(
    "pc_keyboard_velocity_client",
    KEYBOARD / "pc_keyboard_velocity_client.py")
bridge = load_module(
    "ssh_velocity_command_bridge",
    KEYBOARD / "ssh_velocity_command_bridge.py")


def expect(actual, expected):
    if actual != expected:
        raise AssertionError(f"expected {expected!r}, got {actual!r}")


def main():
    speeds = client.SpeedSettings(vx=0.2, vy=0.2, yaw=0.5)
    expect(client.compose_velocity({"w": True}, speeds), (0.2, 0.0, 0.0))
    expect(client.compose_velocity({"w": True, "a": True}, speeds),
           (0.2, 0.2, 0.0))
    expect(client.compose_velocity(
        {"w": True, "a": True, "q": True}, speeds), (0.2, 0.2, 0.5))
    expect(client.compose_velocity({"w": True, "s": True}, speeds),
           (0.0, 0.0, 0.0))
    expect(client.compose_velocity({"a": True, "d": True}, speeds),
           (0.0, 0.0, 0.0))

    client.apply_speed_edge("vx_up", speeds)
    expect(speeds.vx, 0.3)
    expect(speeds.vy, 0.2)
    client.apply_speed_edge("vy_up", speeds)
    expect(speeds.vx, 0.3)
    expect(speeds.vy, 0.3)
    client.apply_speed_edge("vy_up", speeds)
    expect(speeds.vy, 0.3)
    client.apply_speed_edge("yaw_up", speeds)
    expect(speeds.yaw, 0.6)

    expect(bridge.validate_command("v 0.2 -0.3 0.5"),
           "v 0.200000 -0.300000 0.500000")
    expect(bridge.validate_command("z"), "z")
    expect(bridge.validate_command("s"), "s")
    for invalid in ("v nan 0 0", "v 1.1 0 0", "v 0 0", "anything"):
        try:
            bridge.validate_command(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid command: {invalid}")

    print("[PASS] simultaneous keyboard state, independent speed and bridge tests")


if __name__ == "__main__":
    main()
