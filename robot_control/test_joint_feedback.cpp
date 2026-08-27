#include "parallel_bus.h"
#include "ZeroPointCalibration.h"
#include "emergency_stop.h"

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <time.h>
#include <unistd.h>

namespace {
std::atomic<bool> running{true};
void signalHandler(int) { running.store(false, std::memory_order_release); }

uint64_t monotonicNowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

struct BusHardware {
    int chip;
    int line;
    const char* port;
    std::array<unsigned short, 3> motors;
};

constexpr BusHardware kBuses[4] = {
    {1,  7, "/dev/ttyS6", {{0, 4,  8}}},  // FL
    {1, 31, "/dev/ttyS4", {{1, 5,  9}}},  // FR
    {1,  3, "/dev/ttyS7", {{2, 6, 10}}},  // RL
    {4,  5, "/dev/ttyS0", {{3, 7, 11}}},  // RR
};

constexpr const char* kJointNames[12] = {
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf"
};

ParallelBus* findBus(MultiBusController& controller, int motor_id)
{
    for (std::size_t b = 0; b < controller.busCount(); ++b) {
        for (const auto id : controller.bus(b).getMotorIds())
            if (id == motor_id) return &controller.bus(b);
    }
    return nullptr;
}

void printUsage(const char* program)
{
    std::cout << "用法: sudo " << program << " --motor <0..11> [--single-bus]\n"
              << "  --single-bus  只启动目标关节所在腿的3电机总线，用于隔离并发故障\n"
              << "默认向全部12电机发送低阻尼零速度指令；单路模式只处理目标腿3电机。"
                 "程序不会发送位置目标。\n";
}
}

int main(int argc, char** argv)
{
    int selected_id = -1;
    bool single_bus = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--motor" && i + 1 < argc) selected_id = std::atoi(argv[++i]);
        else if (arg == "--single-bus") single_bus = true;
        else if (arg == "-h" || arg == "--help") { printUsage(argv[0]); return 0; }
        else { printUsage(argv[0]); return 1; }
    }
    if (selected_id < 0 || selected_id >= 12) { printUsage(argv[0]); return 1; }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    resetEmergencyStop();

    const JointCalibConfig* configs = getCalibrationConfigs();
    int direction[12] = {0};
    for (int i = 0; i < 12; ++i) direction[configs[i].motor_id] = configs[i].motor_direction;

    std::cout << "Step 4 关节反馈诊断\n"
              << "目标: ID " << selected_id << " (" << kJointNames[selected_id] << ")\n"
              << "安全条件: 机身必须可靠悬空，人员远离夹点；本程序不做机械限位标定。\n"
              << "模式: " << (single_bus ? "单路隔离（目标腿3电机）" : "四路并发（全部12电机）") << "\n"
              << "控制报文: kp=0, kd=0.005, dq=0, tau=0（低阻尼，不发送位置目标）。\n"
              << "操作: 只缓慢转动上述一个关节；输入 s/S 回车或 Ctrl+C 结束。\n"
              << "注意: delta_urdf 是相对启动位置的变化量，绝对URDF零点需机械标定。\n\n"
              << "确认机器狗已可靠悬空后按 Enter 启动；否则按 Ctrl+C。" << std::flush;
    std::string confirmation;
    if (!std::getline(std::cin, confirmation) || !running.load()) return 1;

    MultiBusController controller;
    try {
        const int selected_leg = selected_id % 4;
        std::array<bool, 12> configured{};
        for (int bus_index = 0; bus_index < 4; ++bus_index) {
            if (single_bus && bus_index != selected_leg) continue;
            const auto& hw = kBuses[bus_index];
            ParallelBus& bus = controller.addBus(hw.chip, hw.line, hw.port);
            for (const auto id : hw.motors) {
                if (!bus.addMotor(id)) throw std::runtime_error("重复 motor ID");
                bus.setDamping(id, 0.005f);
                configured[id] = true;
            }
        }
        // 诊断只需 50 Hz；降低总线负载，便于观察通信质量和手动转动。
        controller.startAll(50);

        std::array<float, 12> initial_q{};
        std::array<bool, 12> baseline_ready{};
        uint64_t start_ns = monotonicNowNs();
        uint64_t last_print_ns = 0;

        while (running.load(std::memory_order_acquire)) {
            pollfd pfd{STDIN_FILENO, POLLIN, 0};
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                std::string line;
                if (!std::getline(std::cin, line) || line == "s" || line == "S") break;
            }

            const uint64_t now = monotonicNowNs();
            if (now - last_print_ns < 100'000'000ULL) { usleep(5000); continue; }
            last_print_ns = now;

            bool all_communicating = true;
            int configured_count = 0;
            for (int id = 0; id < 12; ++id) {
                if (!configured[id]) continue;
                ++configured_count;
                ParallelBus* bus = findBus(controller, id);
                const MotorState state = bus ? bus->getState(id) : MotorState{};
                if (!state.correct || state.merror != 0 || state.feedback_timestamp_ns == 0)
                    all_communicating = false;
                else if (!baseline_ready[id]) {
                    initial_q[id] = state.q;
                    baseline_ready[id] = true;
                }
            }

            ParallelBus* selected_bus = findBus(controller, selected_id);
            const MotorState state = selected_bus
                ? selected_bus->getState(selected_id) : MotorState{};
            const float delta_urdf = baseline_ready[selected_id]
                ? direction[selected_id] * (state.q - initial_q[selected_id]) : 0.0f;
            const float dq_urdf = direction[selected_id] * state.dq;
            const float tau_urdf = direction[selected_id] * state.tau;
            const double age_ms = state.feedback_timestamp_ns > 0 && now >= state.feedback_timestamp_ns
                ? (now - state.feedback_timestamp_ns) * 1e-6 : -1.0;

            std::cout << "\rID=" << std::setw(2) << selected_id << " "
                      << std::setw(9) << kJointNames[selected_id]
                      << " sign=" << std::showpos << direction[selected_id] << std::noshowpos
                      << std::fixed << std::setprecision(4)
                      << " q_motor=" << std::setw(8) << state.q
                      << " delta_urdf=" << std::setw(8) << delta_urdf
                      << " dq_urdf=" << std::setw(8) << dq_urdf
                      << " tau_urdf=" << std::setw(8) << tau_urdf
                      << " crc=" << state.correct
                      << " err=" << state.merror
                      << " age=" << std::setw(6) << std::setprecision(1) << age_ms << "ms"
                      << " fails=" << state.consecutive_failures
                      << " all" << configured_count << "="
                      << (all_communicating ? "OK" : "WAIT/FAIL")
                      << " elapsed=" << std::setprecision(1) << (now - start_ns) * 1e-9 << "s   "
                      << std::flush;
        }

        // 先锁存全部12电机阻尼并维持300 ms，再停止通信线程。
        controller.enterEmergencyDampingAll(0.02f);
        usleep(300000);
        controller.stopAll();
        requestEmergencyStop();
        std::cout << "\n[SAFE] 已配置的全部电机已发送阻尼并停止总线线程\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "\n[FAIL] " << error.what() << "，执行统一阻尼退出\n";
        if (controller.busCount() > 0) {
            controller.enterEmergencyDampingAll(0.02f);
            usleep(300000);
            controller.stopAll();
        }
        requestEmergencyStop();
        return 1;
    }
}
