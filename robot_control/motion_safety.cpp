#include "motion_safety.h"

#include <cmath>

bool isMotionSafetyConfigValid(const MotionSafetyConfig& config) noexcept
{
    return std::isfinite(config.max_target_velocity_rad_s)
        && config.max_target_velocity_rad_s > 0.0f
        && std::isfinite(config.max_feedback_velocity_rad_s)
        && config.max_feedback_velocity_rad_s > 0.0f
        && std::isfinite(config.significant_target_delta_rad)
        && config.significant_target_delta_rad > 0.0f
        && config.max_simultaneous_target_changes >= 0
        && config.max_simultaneous_target_changes <= 12
        && std::isfinite(config.max_aggregate_target_delta_rad)
        && config.max_aggregate_target_delta_rad > 0.0f;
}

const char* motionSafetyViolationName(MotionSafetyViolation violation) noexcept
{
    switch (violation) {
    case MotionSafetyViolation::NONE: return "NONE";
    case MotionSafetyViolation::INVALID_INPUT: return "INVALID_INPUT";
    case MotionSafetyViolation::TARGET_VELOCITY: return "TARGET_VELOCITY";
    case MotionSafetyViolation::FEEDBACK_VELOCITY: return "FEEDBACK_VELOCITY";
    case MotionSafetyViolation::SIMULTANEOUS_TARGET_CHANGE:
        return "SIMULTANEOUS_TARGET_CHANGE";
    case MotionSafetyViolation::AGGREGATE_TARGET_CHANGE:
        return "AGGREGATE_TARGET_CHANGE";
    }
    return "UNKNOWN";
}

MotionSafetyResult evaluateMotionSafety(
    const NNCommandSet& candidate,
    const float previous_target[12],
    const float feedback_velocity[12],
    float nominal_policy_dt_sec,
    const MotionSafetyConfig& config) noexcept
{
    MotionSafetyResult result;
    if (!config.enabled) return result;
    if (!isMotionSafetyConfigValid(config)
            || !std::isfinite(nominal_policy_dt_sec)
            || nominal_policy_dt_sec <= 0.0f) {
        result.violation = MotionSafetyViolation::INVALID_INPUT;
        return result;
    }

    int max_target_joint = -1;
    int max_feedback_joint = -1;
    for (int i = 0; i < 12; ++i) {
        const float target = candidate.joint_position_target[i];
        const float previous = previous_target[i];
        const float velocity = feedback_velocity[i];
        if (!std::isfinite(target) || !std::isfinite(previous)
                || !std::isfinite(velocity)) {
            result.violation = MotionSafetyViolation::INVALID_INPUT;
            result.detail = i;
            return result;
        }

        const float delta = std::fabs(target - previous);
        const float target_velocity = delta / nominal_policy_dt_sec;
        const float feedback_speed = std::fabs(velocity);
        result.aggregate_target_delta_rad += delta;
        if (delta > config.significant_target_delta_rad)
            ++result.simultaneous_target_changes;
        if (target_velocity > result.max_target_velocity_rad_s) {
            result.max_target_velocity_rad_s = target_velocity;
            max_target_joint = i;
        }
        if (feedback_speed > result.max_feedback_velocity_rad_s) {
            result.max_feedback_velocity_rad_s = feedback_speed;
            max_feedback_joint = i;
        }
    }

    // 候选指令必须先被拦截，不能等电机已经产生高速反馈后才停机。
    if (result.max_target_velocity_rad_s > config.max_target_velocity_rad_s) {
        result.violation = MotionSafetyViolation::TARGET_VELOCITY;
        result.detail = max_target_joint;
    } else if (result.simultaneous_target_changes
               > config.max_simultaneous_target_changes) {
        result.violation = MotionSafetyViolation::SIMULTANEOUS_TARGET_CHANGE;
        result.detail = result.simultaneous_target_changes;
    } else if (result.aggregate_target_delta_rad
               > config.max_aggregate_target_delta_rad) {
        result.violation = MotionSafetyViolation::AGGREGATE_TARGET_CHANGE;
        result.detail = result.simultaneous_target_changes;
    } else if (result.max_feedback_velocity_rad_s
               > config.max_feedback_velocity_rad_s) {
        result.violation = MotionSafetyViolation::FEEDBACK_VELOCITY;
        result.detail = max_feedback_joint;
    }
    return result;
}
