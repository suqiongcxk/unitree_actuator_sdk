#include "velocity_command_manager.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
float moveToward(float current, float target, float maximum_step)
{
    const float delta = target - current;
    if (delta > maximum_step) return current + maximum_step;
    if (delta < -maximum_step) return current - maximum_step;
    return target;
}

float applyCommandRamp(float current, float target, float dt_sec,
                       float acceleration_rate, float deceleration_rate)
{
    // 反向命令必须先按减速斜率回到零，下一周期才从零向反方向加速。
    // 这样不会在一个策略周期内穿越零点并同时混用两种斜率。
    if (current * target < 0.0f) {
        return moveToward(current, 0.0f, deceleration_rate * dt_sec);
    }

    const bool increasing_magnitude = std::fabs(target) > std::fabs(current);
    const float rate = increasing_magnitude ? acceleration_rate
                                            : deceleration_rate;
    return moveToward(current, target, rate * dt_sec);
}
}

VelocityCommandManager::VelocityCommandManager(
    const VelocityCommandConfig& config)
    : config_(config)
{
    for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(config_.minimum[i])
                || !std::isfinite(config_.maximum[i])
                || !std::isfinite(config_.acceleration_rate[i])
                || !std::isfinite(config_.deceleration_rate[i])
                || config_.minimum[i] > config_.maximum[i]
                || config_.acceleration_rate[i] <= 0.0f
                || config_.deceleration_rate[i] <= 0.0f) {
            throw std::invalid_argument("invalid velocity command config");
        }
    }
    if (config_.watchdog_timeout_ns == 0 || config_.max_single_hold_ns == 0)
        throw std::invalid_argument("velocity command timeout must be positive");
}

VelocityCommandSubmitResult VelocityCommandManager::submit(
    const std::array<float, 3>& command, uint64_t now_ns,
    uint64_t requested_hold_ns)
{
    for (float value : command) {
        if (!std::isfinite(value)) return VelocityCommandSubmitResult::INVALID;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    status_.raw = command;
    status_.clamped = false;
    status_.value_clamped = false;
    status_.hold_clamped = false;
    for (size_t i = 0; i < 3; ++i) {
        status_.limited[i] = std::max(
            config_.minimum[i], std::min(config_.maximum[i], command[i]));
        if (status_.limited[i] != command[i]) status_.value_clamped = true;
    }
    const uint64_t hold_ns = requested_hold_ns == 0
        ? config_.watchdog_timeout_ns
        : std::min(requested_hold_ns, config_.max_single_hold_ns);
    status_.hold_clamped = requested_hold_ns > config_.max_single_hold_ns;
    status_.clamped = status_.value_clamped || status_.hold_clamped;
    status_.effective_hold_ns = hold_ns;
    status_.last_input_ns = now_ns;
    status_.age_ns = 0;
    status_.source_active = true;
    status_.timed_out = false;
    expiry_ns_ = now_ns > UINT64_MAX - hold_ns ? UINT64_MAX : now_ns + hold_ns;
    return status_.clamped ? VelocityCommandSubmitResult::CLAMPED
                           : VelocityCommandSubmitResult::ACCEPTED;
}

std::array<float, 3> VelocityCommandManager::update(uint64_t now_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    float dt_sec = 0.0f;
    if (last_update_ns_ > 0 && now_ns >= last_update_ns_) {
        dt_sec = static_cast<float>(now_ns - last_update_ns_) / 1.0e9f;
    }
    last_update_ns_ = now_ns;

    status_.timed_out = status_.source_active && now_ns > expiry_ns_;
    const std::array<float, 3> zero{{0.0f, 0.0f, 0.0f}};
    const auto& desired = status_.timed_out ? zero : status_.limited;
    for (size_t i = 0; i < 3; ++i) {
        status_.applied[i] = applyCommandRamp(
            status_.applied[i], desired[i], dt_sec,
            config_.acceleration_rate[i], config_.deceleration_rate[i]);
    }
    status_.age_ns = status_.source_active && now_ns >= status_.last_input_ns
        ? now_ns - status_.last_input_ns : 0;
    return status_.applied;
}

VelocityCommandStatus VelocityCommandManager::status(uint64_t now_ns) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    VelocityCommandStatus result = status_;
    result.age_ns = result.source_active && now_ns >= result.last_input_ns
        ? now_ns - result.last_input_ns : 0;
    result.timed_out = result.source_active && now_ns > expiry_ns_;
    return result;
}
