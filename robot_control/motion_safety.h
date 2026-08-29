#ifndef __ROBOT_CONTROL_MOTION_SAFETY_H
#define __ROBOT_CONTROL_MOTION_SAFETY_H

#include "shared_data.h"

// 与具体策略无关的运动指令保护。所有角度均为URDF输出端rad。
struct MotionSafetyConfig {
    bool enabled = true;
    // 50 Hz下分别对应0.05 rad/帧和实测3 rad/s；当前为commissioning保守值。
    float max_target_velocity_rad_s = 2.5f;
    float max_feedback_velocity_rad_s = 3.0f;
    // 防止多个关节在同一策略帧内同时发生中等幅度变化。
    float significant_target_delta_rad = 0.03f;
    int max_simultaneous_target_changes = 3;
    float max_aggregate_target_delta_rad = 0.25f;
};

enum class MotionSafetyViolation {
    NONE = 0,
    INVALID_INPUT,
    TARGET_VELOCITY,
    FEEDBACK_VELOCITY,
    SIMULTANEOUS_TARGET_CHANGE,
    AGGREGATE_TARGET_CHANGE,
};

struct MotionSafetyResult {
    MotionSafetyViolation violation = MotionSafetyViolation::NONE;
    int detail = -1;  // 关节ID；聚合故障时为超阈值关节数。
    float max_target_velocity_rad_s = 0.0f;
    float max_feedback_velocity_rad_s = 0.0f;
    float aggregate_target_delta_rad = 0.0f;
    int simultaneous_target_changes = 0;

    bool passed() const { return violation == MotionSafetyViolation::NONE; }
};

bool isMotionSafetyConfigValid(const MotionSafetyConfig& config) noexcept;
const char* motionSafetyViolationName(MotionSafetyViolation violation) noexcept;

MotionSafetyResult evaluateMotionSafety(
    const NNCommandSet& candidate,
    const float previous_target[12],
    const float feedback_velocity[12],
    float nominal_policy_dt_sec,
    const MotionSafetyConfig& config) noexcept;

#endif  // __ROBOT_CONTROL_MOTION_SAFETY_H
