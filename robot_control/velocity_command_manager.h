#ifndef __ROBOT_CONTROL_VELOCITY_COMMAND_MANAGER_H
#define __ROBOT_CONTROL_VELOCITY_COMMAND_MANAGER_H

#include <array>
#include <cstdint>
#include <mutex>

struct VelocityCommandConfig {
    // model_700.pt 训练最终范围，单位[m/s,m/s,rad/s]。
    std::array<float, 3> minimum{{-1.0f, -1.0f, -1.0f}};
    std::array<float, 3> maximum{{ 1.0f,  1.0f,  1.0f}};
    // 真机部署平滑参数，单位[m/s²,m/s²,rad/s²]。
    std::array<float, 3> slew_rate{{0.5f, 0.5f, 1.0f}};
    uint64_t watchdog_timeout_ns = 500'000'000ULL;
    uint64_t max_single_hold_ns = 2'000'000'000ULL;
};

enum class VelocityCommandSubmitResult {
    ACCEPTED = 0,
    CLAMPED,
    INVALID,
};

struct VelocityCommandStatus {
    std::array<float, 3> raw{{0.0f, 0.0f, 0.0f}};
    std::array<float, 3> limited{{0.0f, 0.0f, 0.0f}};
    std::array<float, 3> applied{{0.0f, 0.0f, 0.0f}};
    uint64_t last_input_ns = 0;
    uint64_t age_ns = 0;
    bool source_active = false;
    bool timed_out = false;
    bool clamped = false;
    bool value_clamped = false;
    bool hold_clamped = false;
    uint64_t effective_hold_ns = 0;
};

class VelocityCommandManager {
public:
    explicit VelocityCommandManager(
        const VelocityCommandConfig& config = VelocityCommandConfig{});

    VelocityCommandSubmitResult submit(
        const std::array<float, 3>& command, uint64_t now_ns,
        uint64_t requested_hold_ns = 0);

    // NN线程每个有效策略周期调用，返回可直接进入obs[9..11]的值。
    std::array<float, 3> update(uint64_t now_ns);
    VelocityCommandStatus status(uint64_t now_ns) const;

private:
    VelocityCommandConfig config_;
    mutable std::mutex mutex_;
    VelocityCommandStatus status_;
    uint64_t expiry_ns_ = 0;
    uint64_t last_update_ns_ = 0;
};

#endif  // __ROBOT_CONTROL_VELOCITY_COMMAND_MANAGER_H
