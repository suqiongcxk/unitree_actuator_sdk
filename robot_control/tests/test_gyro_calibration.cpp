#include "state_estimator.h"
#include "jy901s.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <unistd.h>

namespace {
std::atomic<bool> running{true};
void signalHandler(int) { running.store(false); }

uint64_t monotonicNowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

struct AxisStats {
    int count = 0;
    double mean[3] = {0.0, 0.0, 0.0};
    double m2[3] = {0.0, 0.0, 0.0};
    double max_abs[3] = {0.0, 0.0, 0.0};

    void add(const float value[3])
    {
        ++count;
        for (int i = 0; i < 3; ++i) {
            const double delta = value[i] - mean[i];
            mean[i] += delta / count;
            m2[i] += delta * (value[i] - mean[i]);
            max_abs[i] = std::max(max_abs[i], std::abs(static_cast<double>(value[i])));
        }
    }

    double stddev(int axis) const
    {
        return count > 1 ? std::sqrt(m2[axis] / (count - 1)) : 0.0;
    }

    void reset() { *this = AxisStats{}; }
};
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout
        << "Step 2 陀螺仪零偏与低通实机诊断（仅 I2C，不启动电机）\n"
        << "阶段 A：启动后可先缓慢移动 IMU 约 2 秒，确认校准不会完成。\n"
        << "阶段 B：随后完全静止，正常应在约 1 秒后完成 50 个样本的校准。\n"
        << "阶段 C：继续静止 10 秒，观察每秒统计；再分别缓慢绕 X/Y/Z 转动。\n"
        << "按 Ctrl+C 退出。\n\n";

    JY901S imu("/dev/i2c-1");
    if (imu.init(200) != JY901S_Status::OK) {
        std::cerr << "JY901S 初始化失败\n";
        return 1;
    }

    ComplementaryStateEstimator estimator;
    float joint_q[12] = {0};
    float joint_dq[12] = {0};
    float joint_tau[12] = {0};
    int joint_error[12] = {0};
    bool joint_valid[12];
    uint64_t joint_age_ns[12] = {0};
    uint32_t joint_failure_count[12] = {0};
    for (bool& valid : joint_valid) valid = true;

    bool announced_calibration = false;
    uint64_t calibration_start_ns = monotonicNowNs();
    uint64_t stats_start_ns = 0;
    AxisStats corrected_stats;
    AxisStats filtered_stats;

    std::cout << std::fixed << std::setprecision(5);
    while (running.load()) {
        IMURawData raw{};
        const JY901S_Status data_status = imu.readAll(raw.angles, raw.acc, raw.gyro);
        const JY901S_Status quat_status = data_status == JY901S_Status::OK
            ? imu.readQuaternion(raw.quat) : JY901S_Status::ERROR_I2C_READ;
        raw.timestamp_ns = monotonicNowNs();
        raw.valid = data_status == JY901S_Status::OK
                 && quat_status == JY901S_Status::OK;

        const EstimatedState state = estimator.update(
            raw, joint_q, joint_dq, joint_tau, joint_error, joint_valid,
            joint_age_ns, joint_failure_count, raw.timestamp_ns);

        if (!raw.valid) {
            std::cout << "\r[INVALID] IMU读取失败，status=" << state.status_code
                      << "                              " << std::flush;
            usleep(20000);
            continue;
        }

        if (!state.gyro_calibrated) {
            const double elapsed = (raw.timestamp_ns - calibration_start_ns) * 1e-9;
            const float raw_norm = std::sqrt(raw.gyro.x * raw.gyro.x
                                           + raw.gyro.y * raw.gyro.y
                                           + raw.gyro.z * raw.gyro.z);
            std::cout << "\r[CALIBRATING] elapsed=" << std::setw(6) << elapsed
                      << " s  |gyro|=" << std::setw(8) << raw_norm
                      << " rad/s；移动会重置静止样本          " << std::flush;
            usleep(20000);
            continue;
        }

        if (!announced_calibration) {
            announced_calibration = true;
            stats_start_ns = raw.timestamp_ns;
            std::cout << "\n[CALIBRATED] bias=("
                      << state.gyro_bias[0] << ", " << state.gyro_bias[1] << ", "
                      << state.gyro_bias[2] << ") rad/s\n";
        }

        if (!state.valid || !state.gyro_valid) {
            std::cout << "\r[INVALID] state status=" << state.status_code
                      << "                              " << std::flush;
            usleep(20000);
            continue;
        }

        const float corrected[3] = {
            raw.gyro.x - state.gyro_bias[0],
            raw.gyro.y - state.gyro_bias[1],
            raw.gyro.z - state.gyro_bias[2]
        };
        corrected_stats.add(corrected);
        filtered_stats.add(state.angular_velocity);

        if (raw.timestamp_ns - stats_start_ns >= 1'000'000'000ULL) {
            std::cout << "[1s] corrected mean/std=(";
            for (int i = 0; i < 3; ++i) {
                if (i) std::cout << ", ";
                std::cout << corrected_stats.mean[i] << "/" << corrected_stats.stddev(i);
            }
            std::cout << ")  filtered mean/std/max=(";
            for (int i = 0; i < 3; ++i) {
                if (i) std::cout << ", ";
                std::cout << filtered_stats.mean[i] << "/"
                          << filtered_stats.stddev(i) << "/"
                          << filtered_stats.max_abs[i];
            }
            std::cout << ") rad/s\n";
            corrected_stats.reset();
            filtered_stats.reset();
            stats_start_ns = raw.timestamp_ns;
        }

        usleep(20000);  // 与状态估计线程一致：50 Hz
    }

    std::cout << "\nStep 2 诊断结束\n";
    return 0;
}
