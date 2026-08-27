#include "parallel_bus.h"
#include "emergency_stop.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>

namespace {
std::atomic<bool> running{true};
void signalHandler(int) { running.store(false, std::memory_order_release); }

struct BusConfig {
    int chip;
    int line;
    const char* port;
    std::array<unsigned short, 3> ids;
};

constexpr BusConfig kBuses[4] = {
    {1,  7, "/dev/ttyS6", {{0, 4,  8}}},
    {1, 31, "/dev/ttyS4", {{1, 5,  9}}},
    {1,  3, "/dev/ttyS7", {{2, 6, 10}}},
    {4,  5, "/dev/ttyS0", {{3, 7, 11}}},
};

constexpr const char* kNames[12] = {
    "FL_H", "FR_H", "RL_H", "RR_H",
    "FL_T", "FR_T", "RL_T", "RR_T",
    "FL_C", "FR_C", "RL_C", "RR_C"
};

ParallelBus* findBus(MultiBusController& controller, int id)
{
    for (std::size_t b = 0; b < controller.busCount(); ++b) {
        for (const auto candidate : controller.bus(b).getMotorIds())
            if (candidate == id) return &controller.bus(b);
    }
    return nullptr;
}

uint64_t monotonicNowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}
}

int main(int argc, char** argv)
{
    int duration_sec = 30;
    int selected_motor = -1;
    int selected_bus = -1;
    int target_hz = 500;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--duration" && i + 1 < argc) duration_sec = std::atoi(argv[++i]);
        else if (arg == "--motor" && i + 1 < argc) selected_motor = std::atoi(argv[++i]);
        else if (arg == "--bus" && i + 1 < argc) selected_bus = std::atoi(argv[++i]);
        else if (arg == "--hz" && i + 1 < argc) target_hz = std::atoi(argv[++i]);
        else {
            std::cerr << "用法: sudo " << argv[0]
                      << " [--duration 秒数] [--motor 0..11 | --bus 0..3] [--hz 频率]\n";
            return 1;
        }
    }
    if (duration_sec <= 0 || duration_sec > 3600) {
        std::cerr << "duration 必须在1..3600秒\n";
        return 1;
    }
    if (selected_motor < -1 || selected_motor >= 12) {
        std::cerr << "motor 必须在0..11\n";
        return 1;
    }
    if (selected_bus < -1 || selected_bus >= 4) {
        std::cerr << "bus 必须在0..3（A..D）\n";
        return 1;
    }
    if (selected_motor >= 0 && selected_bus >= 0) {
        std::cerr << "--motor 和 --bus 不能同时使用\n";
        return 1;
    }
    if (target_hz < 1 || target_hz > 1000) {
        std::cerr << "hz 必须在1..1000\n";
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    resetEmergencyStop();

    const bool single_motor = selected_motor >= 0;
    const bool single_bus = selected_bus >= 0;
    const char* mode_name = single_motor ? "单电机" : (single_bus ? "单总线三电机" : "12电机");
    std::cout << mode_name << target_hz
              << " Hz阻尼通信丢包率测试\n"
              << (single_motor ? "只打开目标电机所在UART并只注册该一个ID。\n"
                               : (single_bus ? "只打开指定UART，每周期依次轮询该总线3个电机。\n"
                                             : "每路按目标频率循环，每周期依次轮询该腿3个电机。\n"))
              << (single_motor ? "目标 ID=" + std::to_string(selected_motor)
                                   + " (" + kNames[selected_motor] + ")\n" : "")
              << (single_bus ? "目标总线=" + std::string(1, static_cast<char>('A' + selected_bus))
                                  + " (" + kBuses[selected_bus].port + ")\n" : "")
              << "指令: kp=0, kd=0.02, dq=0, tau=0；没有位置目标。\n"
              << "机身必须可靠悬空，不能同时运行其他电机程序。\n"
              << "持续 " << duration_sec << " 秒；s/S回车或Ctrl+C可提前退出。\n"
              << "确认安全后按 Enter 启动。" << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return 1;

    MultiBusController controller;
    try {
        std::array<bool, 12> active{};
        for (int leg = 0; leg < 4; ++leg) {
            if (single_motor && leg != selected_motor % 4) continue;
            if (single_bus && leg != selected_bus) continue;
            const auto& cfg = kBuses[leg];
            auto& bus = controller.addBus(cfg.chip, cfg.line, cfg.port);
            for (const auto id : cfg.ids) {
                if (single_motor && id != selected_motor) continue;
                bus.addMotor(id);
                bus.setDamping(id, 0.02f);
                active[id] = true;
            }
        }
        controller.startAll(target_hz);

        std::array<uint64_t, 12> last_total{};
        std::array<uint64_t, 12> last_success{};
        std::array<uint64_t, 12> max_feedback_age_ns{};
        std::array<uint32_t, 12> max_consecutive_failures{};
        const auto start = std::chrono::steady_clock::now();
        auto next_report = start + std::chrono::seconds(1);

        while (running.load(std::memory_order_acquire)) {
            pollfd pfd{STDIN_FILENO, POLLIN, 0};
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                if (!std::getline(std::cin, line) || line == "s" || line == "S") break;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - start >= std::chrono::seconds(duration_sec)) break;

            // 以10 ms主循环持续采样，捕捉累计丢包率无法反映的突发断流。
            const uint64_t now_ns = monotonicNowNs();
            for (int id = 0; id < 12; ++id) {
                if (!active[id]) continue;
                ParallelBus* bus = findBus(controller, id);
                if (!bus) throw std::runtime_error("活动电机未找到对应总线");
                const MotorState state = bus->getState(id);
                max_consecutive_failures[id] = std::max(
                    max_consecutive_failures[id], state.consecutive_failures);
                if (state.feedback_timestamp_ns > 0
                        && now_ns >= state.feedback_timestamp_ns) {
                    max_feedback_age_ns[id] = std::max(
                        max_feedback_age_ns[id],
                        now_ns - state.feedback_timestamp_ns);
                }
            }

            if (now < next_report) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            next_report += std::chrono::seconds(1);

            std::cout << "\nsec="
                      << std::chrono::duration_cast<std::chrono::seconds>(now - start).count()
                      << " bus_hz={";
            for (std::size_t b = 0; b < controller.busCount(); ++b) {
                if (b) std::cout << ",";
                std::cout << std::fixed << std::setprecision(1) << controller.bus(b).getActualHz();
            }
            std::cout << "}\n ID name   total      ok   short  crc/id window_loss total_loss"
                         " max_age_ms max_streak\n";

            for (int id = 0; id < 12; ++id) {
                if (!active[id]) continue;
                ParallelBus* bus = findBus(controller, id);
                if (!bus) throw std::runtime_error("活动电机未找到对应总线");
                const MotorState s = bus->getState(id);
                const uint64_t window_total = s.transaction_count - last_total[id];
                const uint64_t window_ok = s.success_count - last_success[id];
                const double window_loss = window_total > 0
                    ? 100.0 * (window_total - window_ok) / window_total : 100.0;
                const double total_loss = s.transaction_count > 0
                    ? 100.0 * (s.transaction_count - s.success_count) / s.transaction_count : 100.0;
                std::cout << std::setw(3) << id << " " << std::setw(5) << kNames[id]
                          << " " << std::setw(7) << s.transaction_count
                          << " " << std::setw(7) << s.success_count
                          << " " << std::setw(7) << s.short_frame_count
                          << " " << std::setw(7) << s.protocol_failure_count
                          << " " << std::setw(10) << std::setprecision(3) << window_loss << "%"
                          << " " << std::setw(9) << total_loss << "%"
                          << " " << std::setw(10) << std::setprecision(2)
                          << max_feedback_age_ns[id] * 1e-6
                          << " " << std::setw(10) << max_consecutive_failures[id]
                          << "\n";
                last_total[id] = s.transaction_count;
                last_success[id] = s.success_count;
            }
        }

        controller.enterEmergencyDampingAll(0.02f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        controller.stopAll();
        requestEmergencyStop();
        std::cout << "\n[SAFE] 所有活动电机已持续发送阻尼并停止总线线程\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "\n[FAIL] " << error.what() << "，尝试阻尼退出\n";
        if (controller.busCount() > 0) {
            controller.enterEmergencyDampingAll(0.02f);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            controller.stopAll();
        }
        requestEmergencyStop();
        return 1;
    }
}
