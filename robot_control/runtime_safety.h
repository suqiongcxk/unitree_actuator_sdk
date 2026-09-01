#ifndef __ROBOT_CONTROL_RUNTIME_SAFETY_H
#define __ROBOT_CONTROL_RUNTIME_SAFETY_H

#include <cstdint>
#include <mutex>

// 运行期安全故障只锁存第一个根因，避免关闭过程的派生错误覆盖原始证据。
enum class RuntimeSafetyFault : int {
    NONE = 0,
    IMU_THREAD_STALE,
    ESTIMATION_THREAD_STALE,
    NN_THREAD_STALE,
    ESTIMATOR_UNAVAILABLE,
    STATE_STREAM_TIMEOUT,
    STATE_INVALID,
    MOTOR_FEEDBACK_INVALID,
    MOTOR_ERROR,
    MOTOR_OVERTEMPERATURE,
    NN_COMMAND_INVALID,
    JOINT_TARGET_RATE_EXCEEDED,
    JOINT_FEEDBACK_VELOCITY_EXCEEDED,
    AGGREGATE_TARGET_CHANGE,
};

struct RuntimeSafetyStatus {
    RuntimeSafetyFault fault = RuntimeSafetyFault::NONE;
    uint64_t timestamp_ns = 0;
    int detail = -1;  // 例如首个无效motor ID；-1表示无额外索引。

    bool latched() const { return fault != RuntimeSafetyFault::NONE; }
};

class RuntimeSafetySupervisor {
public:
    bool latch(RuntimeSafetyFault fault, uint64_t timestamp_ns,
               int detail = -1)
    {
        if (fault == RuntimeSafetyFault::NONE) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.latched()) return false;
        status_.fault = fault;
        status_.timestamp_ns = timestamp_ns;
        status_.detail = detail;
        return true;
    }

    RuntimeSafetyStatus status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

    static const char* name(RuntimeSafetyFault fault) noexcept
    {
        switch (fault) {
        case RuntimeSafetyFault::NONE: return "NONE";
        case RuntimeSafetyFault::IMU_THREAD_STALE: return "IMU_THREAD_STALE";
        case RuntimeSafetyFault::ESTIMATION_THREAD_STALE:
            return "ESTIMATION_THREAD_STALE";
        case RuntimeSafetyFault::NN_THREAD_STALE: return "NN_THREAD_STALE";
        case RuntimeSafetyFault::ESTIMATOR_UNAVAILABLE:
            return "ESTIMATOR_UNAVAILABLE";
        case RuntimeSafetyFault::STATE_STREAM_TIMEOUT: return "STATE_STREAM_TIMEOUT";
        case RuntimeSafetyFault::STATE_INVALID: return "STATE_INVALID";
        case RuntimeSafetyFault::MOTOR_FEEDBACK_INVALID:
            return "MOTOR_FEEDBACK_INVALID";
        case RuntimeSafetyFault::MOTOR_ERROR: return "MOTOR_ERROR";
        case RuntimeSafetyFault::MOTOR_OVERTEMPERATURE:
            return "MOTOR_OVERTEMPERATURE";
        case RuntimeSafetyFault::NN_COMMAND_INVALID:
            return "NN_COMMAND_INVALID";
        case RuntimeSafetyFault::JOINT_TARGET_RATE_EXCEEDED:
            return "JOINT_TARGET_RATE_EXCEEDED";
        case RuntimeSafetyFault::JOINT_FEEDBACK_VELOCITY_EXCEEDED:
            return "JOINT_FEEDBACK_VELOCITY_EXCEEDED";
        case RuntimeSafetyFault::AGGREGATE_TARGET_CHANGE:
            return "AGGREGATE_TARGET_CHANGE";
        }
        return "UNKNOWN";
    }

    static bool heartbeatStale(uint64_t now_ns, uint64_t last_ns,
                               uint64_t timeout_ns) noexcept
    {
        return last_ns > 0 && now_ns >= last_ns
            && now_ns - last_ns > timeout_ns;
    }

    static RuntimeSafetyFault classifyMotorFeedback(
        int temperature_c, int merror, int shutdown_temperature_c) noexcept
    {
        if (temperature_c >= shutdown_temperature_c)
            return RuntimeSafetyFault::MOTOR_OVERTEMPERATURE;
        if (merror != 0) return RuntimeSafetyFault::MOTOR_ERROR;
        return RuntimeSafetyFault::NONE;
    }

private:
    mutable std::mutex mutex_;
    RuntimeSafetyStatus status_;
};

#endif  // __ROBOT_CONTROL_RUNTIME_SAFETY_H
