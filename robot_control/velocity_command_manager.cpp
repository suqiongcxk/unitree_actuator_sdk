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
}

VelocityCommandManager::VelocityCommandManager(
    const VelocityCommandConfig& config)
    : config_(config)
{
    for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(config_.minimum[i])
                || !std::isfinite(config_.maximum[i])
                || !std::isfinite(config_.slew_rate[i])
                || config_.minimum[i] > config_.maximum[i]
                || config_.slew_rate[i] <= 0.0f) {
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
        const float maximum_step = config_.slew_rate[i] * dt_sec;
        status_.applied[i] = moveToward(
            status_.applied[i], desired[i], maximum_step);
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
