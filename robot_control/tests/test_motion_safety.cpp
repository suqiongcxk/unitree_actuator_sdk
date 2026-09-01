#include "motion_safety.h"

#include <iostream>
#include <limits>

namespace {
NNCommandSet commandAt(float value)
{
    NNCommandSet command;
    for (int i = 0; i < 12; ++i) command.joint_position_target[i] = value;
    command.valid = true;
    return command;
}

bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}
}

int main()
{
    bool ok = true;
    MotionSafetyConfig config;
    float previous[12] = {0};
    float velocity[12] = {0};

    NNCommandSet safe = commandAt(0.0f);
    safe.joint_position_target[2] = 0.02f;
    MotionSafetyResult result = evaluateMotionSafety(
        safe, previous, velocity, 0.02f, config);
    ok &= expect(result.passed(), "零命令实测基线幅度应通过");

    NNCommandSet target_rate = commandAt(0.0f);
    target_rate.joint_position_target[7] = 0.101f;
    result = evaluateMotionSafety(target_rate, previous, velocity, 0.02f, config);
    ok &= expect(result.passed()
                     && result.max_target_velocity_rad_s > 5.0f,
                 "监测模式应记录目标超速但不得停机");

    MotionSafetyConfig target_enforced = config;
    target_enforced.enforce_target_velocity = true;
    result = evaluateMotionSafety(
        target_rate, previous, velocity, 0.02f, target_enforced);
    ok &= expect(result.violation == MotionSafetyViolation::TARGET_VELOCITY
                     && result.detail == 7,
                 "恢复目标速度保护后必须拦截超限目标");

    MotionSafetyConfig aggregate_config = config;
    aggregate_config.max_target_velocity_rad_s = 10.0f;
    NNCommandSet coordinated = commandAt(0.0f);
    for (int i = 0; i < 4; ++i) coordinated.joint_position_target[i] = 0.031f;
    result = evaluateMotionSafety(
        coordinated, previous, velocity, 0.02f, aggregate_config);
    ok &= expect(result.passed()
                     && result.simultaneous_target_changes == 4,
                 "四关节协同变化只应统计，不得触发停机");

    for (int i = 0; i < 12; ++i) coordinated.joint_position_target[i] = 0.042f;
    result = evaluateMotionSafety(
        coordinated, previous, velocity, 0.02f, aggregate_config);
    ok &= expect(result.passed()
                     && result.aggregate_target_delta_rad > 0.50f,
                 "监测模式应记录总变化量超限但不得停机");

    aggregate_config.enforce_aggregate_target_delta = true;
    result = evaluateMotionSafety(
        coordinated, previous, velocity, 0.02f, aggregate_config);
    ok &= expect(result.violation
                     == MotionSafetyViolation::AGGREGATE_TARGET_CHANGE,
                 "恢复总变化量保护后必须拦截超限目标");

    NNCommandSet unchanged = commandAt(0.0f);
    velocity[10] = -3.01f;
    result = evaluateMotionSafety(
        unchanged, previous, velocity, 0.02f, config);
    ok &= expect(result.passed()
                     && result.max_feedback_velocity_rad_s > 3.0f,
                 "监测模式应记录反馈超速但不得停机");

    MotionSafetyConfig feedback_enforced = config;
    feedback_enforced.enforce_feedback_velocity = true;
    result = evaluateMotionSafety(
        unchanged, previous, velocity, 0.02f, feedback_enforced);
    ok &= expect(result.violation == MotionSafetyViolation::FEEDBACK_VELOCITY
                     && result.detail == 10,
                 "恢复反馈速度保护后，实测关节超速必须停机");

    velocity[10] = 0.0f;
    unchanged.joint_position_target[3] =
        std::numeric_limits<float>::quiet_NaN();
    result = evaluateMotionSafety(
        unchanged, previous, velocity, 0.02f, config);
    ok &= expect(result.violation == MotionSafetyViolation::INVALID_INPUT
                     && result.detail == 3,
                 "NaN不得绕过独立运动安全层");

    if (!ok) return 1;
    std::cout << "[PASS] target-rate, feedback-speed and aggregate modes; "
                 "multi-joint count is diagnostic only"
              << std::endl;
    return 0;
}
