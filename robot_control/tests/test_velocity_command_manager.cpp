#include "velocity_command_manager.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {
bool near(float a, float b, float tolerance = 1.0e-6f)
{
    return std::fabs(a - b) <= tolerance;
}
}

int main()
{
    VelocityCommandManager manager;
    constexpr uint64_t ms = 1'000'000ULL;

    auto command = manager.update(100 * ms);
    if (!near(command[0], 0.0f) || manager.status(100 * ms).timed_out) return 1;

    const auto result = manager.submit({{2.0f, -0.4f, 0.5f}}, 100 * ms);
    if (result != VelocityCommandSubmitResult::CLAMPED) return 1;
    command = manager.update(120 * ms);
    if (!near(command[0], 0.01f) || !near(command[1], -0.01f)
            || !near(command[2], 0.02f)) return 1;

    // 500 ms边界仍有效，超过后平滑回零。
    manager.update(600 * ms);
    if (manager.status(600 * ms).timed_out) return 1;
    const auto before_timeout = manager.status(600 * ms).applied;
    command = manager.update(620 * ms);
    if (!manager.status(620 * ms).timed_out
            || !(std::fabs(command[0]) < std::fabs(before_timeout[0]))) return 1;

    if (manager.submit({{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}},
                       700 * ms)
            != VelocityCommandSubmitResult::INVALID) return 1;

    // 终端显式保持最长允许5 s，即使请求更长。
    if (manager.submit({{0.2f, 0.0f, 0.0f}}, 1000 * ms, 10'000 * ms)
            != VelocityCommandSubmitResult::CLAMPED) return 1;
    const auto clamped_hold = manager.status(1000 * ms);
    if (!clamped_hold.hold_clamped || !clamped_hold.clamped
            || clamped_hold.value_clamped
            || clamped_hold.effective_hold_ns != 5000 * ms) return 1;
    manager.update(5999 * ms);
    if (manager.status(5999 * ms).timed_out) return 1;
    manager.update(6001 * ms);
    if (!manager.status(6001 * ms).timed_out) return 1;

    // 前进/左移/左转直接切换到反向时，必须按斜率限制平滑过零。
    VelocityCommandManager reversal;
    reversal.update(100 * ms);
    reversal.submit({{0.3f, 0.2f, 0.4f}}, 100 * ms, 5000 * ms);
    for (uint64_t now_ms = 120; now_ms <= 700; now_ms += 20) {
        reversal.submit({{0.3f, 0.2f, 0.4f}}, now_ms * ms);
        reversal.update(now_ms * ms);
    }
    const auto before_reverse = reversal.status(700 * ms).applied;
    reversal.submit({{-0.3f, -0.2f, -0.4f}}, 700 * ms, 5000 * ms);
    const auto after_reverse = reversal.update(720 * ms);
    if (!near(before_reverse[0], 0.3f)
            || !near(before_reverse[1], 0.2f)
            || !near(before_reverse[2], 0.4f)
            || !near(after_reverse[0], 0.29f)
            || !near(after_reverse[1], 0.19f)
            || !near(after_reverse[2], 0.38f)) return 1;

    std::cout << "[PASS] velocity command limits, slew and watchdog tests"
              << std::endl;
    return 0;
}
