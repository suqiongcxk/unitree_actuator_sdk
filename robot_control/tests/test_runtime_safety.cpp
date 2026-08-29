#include "runtime_safety.h"

#include <iostream>
#include <string>
#include <thread>
#include <vector>

int main()
{
    if (RuntimeSafetySupervisor::heartbeatStale(100, 0, 50)
            || RuntimeSafetySupervisor::heartbeatStale(100, 50, 50)
            || !RuntimeSafetySupervisor::heartbeatStale(101, 50, 50)
            || RuntimeSafetySupervisor::heartbeatStale(50, 100, 10)) {
        std::cerr << "[FAIL] 心跳陈旧边界判断错误" << std::endl;
        return 1;
    }
    if (std::string(RuntimeSafetySupervisor::name(
            RuntimeSafetyFault::MOTOR_OVERTEMPERATURE))
            != "MOTOR_OVERTEMPERATURE"
            || std::string(RuntimeSafetySupervisor::name(
                RuntimeSafetyFault::NN_COMMAND_INVALID))
                != "NN_COMMAND_INVALID"
            || std::string(RuntimeSafetySupervisor::name(
                RuntimeSafetyFault::JOINT_TARGET_RATE_EXCEEDED))
                != "JOINT_TARGET_RATE_EXCEEDED"
            || std::string(RuntimeSafetySupervisor::name(
                RuntimeSafetyFault::JOINT_FEEDBACK_VELOCITY_EXCEEDED))
                != "JOINT_FEEDBACK_VELOCITY_EXCEEDED"
            || std::string(RuntimeSafetySupervisor::name(
                RuntimeSafetyFault::MULTI_JOINT_TARGET_CHANGE))
                != "MULTI_JOINT_TARGET_CHANGE") {
        std::cerr << "[FAIL] 温度故障名称映射错误" << std::endl;
        return 1;
    }
    if (RuntimeSafetySupervisor::classifyMotorFeedback(89, 0, 90)
                != RuntimeSafetyFault::NONE
            || RuntimeSafetySupervisor::classifyMotorFeedback(90, 0, 90)
                != RuntimeSafetyFault::MOTOR_OVERTEMPERATURE
            || RuntimeSafetySupervisor::classifyMotorFeedback(50, 2, 90)
                != RuntimeSafetyFault::MOTOR_ERROR
            || RuntimeSafetySupervisor::classifyMotorFeedback(95, 2, 90)
                != RuntimeSafetyFault::MOTOR_OVERTEMPERATURE) {
        std::cerr << "[FAIL] 电机温度/MError分类边界错误" << std::endl;
        return 1;
    }

    RuntimeSafetySupervisor supervisor;
    if (supervisor.status().latched()) return 1;

    std::vector<std::thread> writers;
    for (int i = 0; i < 8; ++i) {
        writers.emplace_back([&supervisor, i]() {
            supervisor.latch(RuntimeSafetyFault::STATE_INVALID,
                             1000 + static_cast<uint64_t>(i), i);
        });
    }
    for (auto& writer : writers) writer.join();

    const RuntimeSafetyStatus first = supervisor.status();
    if (first.fault != RuntimeSafetyFault::STATE_INVALID
            || first.timestamp_ns < 1000 || first.timestamp_ns > 1007
            || first.detail < 0 || first.detail > 7) {
        std::cerr << "[FAIL] 首故障锁存结果不一致" << std::endl;
        return 1;
    }
    supervisor.latch(RuntimeSafetyFault::NN_THREAD_STALE, 9999, 99);
    const RuntimeSafetyStatus after = supervisor.status();
    if (after.fault != first.fault || after.timestamp_ns != first.timestamp_ns
            || after.detail != first.detail) {
        std::cerr << "[FAIL] 后续故障覆盖了第一根因" << std::endl;
        return 1;
    }

    std::cout << "[PASS] runtime safety first-fault latch test" << std::endl;
    return 0;
}
