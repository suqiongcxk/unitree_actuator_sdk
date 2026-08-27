#include "motor_controller.h"
#include "ZeroPointCalibration.h"
#include "emergency_stop.h"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
std::atomic<bool> running{true};
void signalHandler(int) { running.store(false); }

constexpr int kGlobalGpio[4] = {39, 63, 35, 133};
constexpr const char* kPort[4] = {"/dev/ttyS6", "/dev/ttyS4", "/dev/ttyS7", "/dev/ttyS0"};
constexpr unsigned short kMotors[4][3] = {
    {0, 4, 8}, {1, 5, 9}, {2, 6, 10}, {3, 7, 11}
};
constexpr const char* kNames[12] = {
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf"
};
}

int main(int argc, char** argv)
{
    if (argc < 3 || std::string(argv[1]) != "--motor") {
        std::cerr << "用法: sudo " << argv[0]
                  << " --motor <0..11> [--single-motor]\n";
        return 1;
    }
    const int selected = std::atoi(argv[2]);
    if (selected < 0 || selected >= 12) return 1;
    bool single_motor = false;
    for (int i = 3; i < argc; ++i) {
        if (std::string(argv[i]) == "--single-motor") single_motor = true;
        else return 1;
    }
    const int leg = selected % 4;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    resetEmergencyStop();

    const JointCalibConfig* configs = getCalibrationConfigs();
    int direction[12] = {0};
    for (int i = 0; i < 12; ++i) direction[configs[i].motor_id] = configs[i].motor_direction;

    std::cout << "Step 4 同步 MotorBus 对照诊断\n"
              << "目标 ID=" << selected << " " << kNames[selected]
              << "，总线=" << kPort[leg] << "，模式="
              << (single_motor ? "严格单电机" : "本腿3电机") << "\n"
              << "只发送 kp=0,kd=0.005,dq=0,tau=0；不发送位置目标。\n"
              << "本轮只观察通信，不要转动关节。确认机身悬空后按 Enter。" << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return 1;

    try {
        MotorBus bus(kGlobalGpio[leg], kPort[leg]);
        std::vector<unsigned short> active_ids;
        if (single_motor) active_ids.push_back(static_cast<unsigned short>(selected));
        else active_ids.assign(kMotors[leg], kMotors[leg] + 3);
        for (const auto id : active_ids) {
            bus.addMotor(id);
            bus.setDamping(id, 0.005f);
        }

        float initial_q = 0.0f;
        bool baseline = false;
        std::vector<unsigned> failures(active_ids.size(), 0);
        int print_divider = 0;
        while (running.load()) {
            pollfd pfd{STDIN_FILENO, POLLIN, 0};
            if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                if (!std::getline(std::cin, line) || line == "s" || line == "S") break;
            }

            bus.sendRecv();
            bool all_ok = true;
            MotorState selected_state;
            for (std::size_t i = 0; i < active_ids.size(); ++i) {
                const MotorState state = bus.getState(active_ids[i]);
                const bool ok = state.correct && state.merror == 0;
                failures[i] = ok ? 0 : failures[i] + 1;
                all_ok = all_ok && ok;
                if (active_ids[i] == selected) selected_state = state;
            }
            if (selected_state.correct && !baseline) {
                initial_q = selected_state.q;
                baseline = true;
            }
            const float delta = baseline
                ? direction[selected] * (selected_state.q - initial_q) : 0.0f;
            if (++print_divider % 10 != 0) { usleep(20000); continue; }
            std::cout << "ID=" << selected << " " << std::setw(9) << kNames[selected]
                      << " q_motor=" << std::fixed << std::setprecision(4) << selected_state.q
                      << " delta_urdf=" << delta
                      << " dq_urdf=" << direction[selected] * selected_state.dq
                      << " crc=" << selected_state.correct
                      << " err=" << selected_state.merror
                      << " fails={";
            for (std::size_t i = 0; i < failures.size(); ++i) {
                if (i) std::cout << ",";
                std::cout << failures[i];
            }
            std::cout << "} all" << active_ids.size() << "="
                      << (all_ok ? "OK" : "WAIT/FAIL") << std::endl;
            usleep(20000); // 明确保留20 ms周期间隔，复用标定路径的同步节奏。
        }

        for (const auto id : active_ids) bus.setDamping(id, 0.02f);
        for (int i = 0; i < 10; ++i) { bus.sendRecv(); usleep(20000); }
        requestEmergencyStop();
        std::cout << "\n[SAFE] 已配置电机持续发送阻尼后退出\n";
        return 0;
    } catch (const std::exception& error) {
        requestEmergencyStop();
        std::cerr << "\n[FAIL] " << error.what() << "\n";
        return 1;
    }
}
