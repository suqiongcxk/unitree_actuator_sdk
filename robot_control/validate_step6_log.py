#!/usr/bin/env python3
"""校验 Step 6 实机 dry-run CSV，不访问硬件、不驱动电机。"""

import csv
import math
import sys


DEFAULT_POSE = [
    0.1, -0.1, 0.1, -0.1,
    0.8, 0.8, 1.0, 1.0,
    -1.5, -1.5, -1.5, -1.5,
]
ACTION_SCALE = 0.25
TOLERANCE = 2.0e-6


def fail(message):
    print(f"[FAIL] {message}")
    return 1


def value(row, name):
    result = float(row[name])
    if not math.isfinite(result):
        raise ValueError(f"{name} 不是有限值: {row[name]}")
    return result


def main():
    if len(sys.argv) != 2:
        print(f"用法: {sys.argv[0]} /tmp/step6_real_policy.csv")
        return 2

    try:
        with open(sys.argv[1], newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            fields = set(reader.fieldnames or [])
    except (OSError, csv.Error) as error:
        return fail(f"无法读取 CSV: {error}")

    required = {
        "timestamp_ns", "valid", "body_lv_x", "body_lv_y", "body_lv_z",
        "av_x", "av_y", "av_z", "pg_x", "pg_y", "pg_z",
        *(f"j{i}_pos" for i in range(12)),
        *(f"j{i}_vel" for i in range(12)),
        *(f"obs{i}" for i in range(48)),
        *(f"raw_action{i}" for i in range(12)),
        *(f"tgt{i}" for i in range(12)),
    }
    missing = sorted(required - fields)
    if missing:
        return fail("CSV 缺列: " + ", ".join(missing))
    if len(rows) < 20:
        return fail(f"只有 {len(rows)} 帧，至少需要 20 帧")

    peaks = {
        "body_linear_velocity": 0.0,
        "angular_velocity": 0.0,
        "projected_gravity": 0.0,
        "zero_velocity_command": 0.0,
        "relative_joint_position": 0.0,
        "joint_velocity": 0.0,
        "previous_raw_action": 0.0,
        "target_formula": 0.0,
    }
    source_names = ["body_lv_x", "body_lv_y", "body_lv_z"]
    angular_names = ["av_x", "av_y", "av_z"]
    gravity_names = ["pg_x", "pg_y", "pg_z"]

    try:
        previous_timestamp = -1
        for frame, row in enumerate(rows):
            timestamp = int(row["timestamp_ns"])
            if timestamp <= previous_timestamp:
                return fail(f"frame {frame}: timestamp 未严格递增")
            previous_timestamp = timestamp
            if int(row["valid"]) != 1:
                return fail(f"frame {frame}: ONNX 推理 valid != 1")

            for axis in range(3):
                peaks["body_linear_velocity"] = max(
                    peaks["body_linear_velocity"],
                    abs(value(row, f"obs{axis}") - value(row, source_names[axis])))
                peaks["angular_velocity"] = max(
                    peaks["angular_velocity"],
                    abs(value(row, f"obs{3 + axis}") - value(row, angular_names[axis])))
                peaks["projected_gravity"] = max(
                    peaks["projected_gravity"],
                    abs(value(row, f"obs{6 + axis}") - value(row, gravity_names[axis])))
                # 本次验收未接遥控速度命令，三维 command 必须明确为零。
                peaks["zero_velocity_command"] = max(
                    peaks["zero_velocity_command"], abs(value(row, f"obs{9 + axis}")))

            for joint in range(12):
                peaks["relative_joint_position"] = max(
                    peaks["relative_joint_position"],
                    abs(value(row, f"obs{12 + joint}")
                        - (value(row, f"j{joint}_pos") - DEFAULT_POSE[joint])))
                peaks["joint_velocity"] = max(
                    peaks["joint_velocity"],
                    abs(value(row, f"obs{24 + joint}") - value(row, f"j{joint}_vel")))

                expected_history = (0.0 if frame == 0 else
                                    value(rows[frame - 1], f"raw_action{joint}"))
                peaks["previous_raw_action"] = max(
                    peaks["previous_raw_action"],
                    abs(value(row, f"obs{36 + joint}") - expected_history))

                expected_target = (DEFAULT_POSE[joint]
                                   + ACTION_SCALE * value(row, f"raw_action{joint}"))
                peaks["target_formula"] = max(
                    peaks["target_formula"],
                    abs(value(row, f"tgt{joint}") - expected_target))
    except (KeyError, ValueError) as error:
        return fail(str(error))

    print(f"[Step6] frames={len(rows)}, action_scale={ACTION_SCALE}")
    for name, peak in peaks.items():
        print(f"  {name:28s} max_abs={peak:.3e}")
    bad = {name: peak for name, peak in peaks.items() if peak > TOLERANCE}
    if bad:
        return fail(f"{len(bad)} 项超过容差 {TOLERANCE:.1e}")
    print("[PASS] 实机 48 维 observation、raw-action 历史和目标公式一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
