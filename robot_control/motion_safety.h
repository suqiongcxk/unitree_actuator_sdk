#ifndef __ROBOT_CONTROL_MOTION_SAFETY_H
#define __ROBOT_CONTROL_MOTION_SAFETY_H

#include "shared_data.h"

// 与具体策略无关的运动指令保护。所有角度均为URDF输出端rad。
struct MotionSafetyConfig {
    bool enabled = true;
    // 临时实机诊断：仍统计单关节目标速度，但不因超限触发停机。
    // 恢复保护时设为true，无需重写判定逻辑。
    bool enforce_target_velocity = false;
    // 50 Hz下对应0.10 rad/帧。
    float max_target_velocity_rad_s = 5.0f;
    // 临时实机诊断：仍统计反馈速度，但不因超限触发停机。
    // 恢复保护时设为true，无需重写判定逻辑。
    bool enforce_feedback_velocity = false;
    float max_feedback_velocity_rad_s = 3.0f;
    // 仅用于诊断统计；四足步态需要多关节协同，数量本身不触发停机。
    float significant_target_delta_rad = 0.03f;
    // 临时实机诊断：仍统计12关节总变化量，但不因超限触发停机。
    bool enforce_aggregate_target_delta = false;
    // 12关节单个策略帧的目标位置变化绝对值之和，单位rad。
    float max_aggregate_target_delta_rad = 0.50f;
};

enum class MotionSafetyViolation {
    NONE = 0,
    INVALID_INPUT,
    TARGET_VELOCITY,
    FEEDBACK_VELOCITY,
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
