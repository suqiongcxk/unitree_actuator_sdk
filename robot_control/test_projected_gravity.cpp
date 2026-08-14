#include "state_estimator.h"
#include "jy901s.h"

#include <atomic>
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
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "Step 3 投影重力测试（仅 I2C，不启动电机）\n"
              << "测试顺序：水平 → 头部下沉 → 左侧下沉 → 水平原地偏航\n"
              << "按 Ctrl+C 退出。开始后先保持 IMU 静止约 1 秒。\n\n";

    JY901S imu("/dev/i2c-1");
    if (imu.init(200) != JY901S_Status::OK) {
        std::cerr << "JY901S 初始化失败\n";
        return 1;
    }

    PassthroughEstimator estimator;
    float joint_q[12] = {0};
    float joint_dq[12] = {0};
    float joint_tau[12] = {0};
    int joint_error[12] = {0};
    bool joint_valid[12];
    uint64_t joint_age_ns[12] = {0};
    uint32_t joint_failure_count[12] = {0};
    for (bool& valid : joint_valid) valid = true;
    int print_divider = 0;

    std::cout << std::fixed << std::setprecision(3);
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

        if (++print_divider % 5 == 0) {
            if (!state.gyro_calibrated) {
                std::cout << "\r正在静止校准陀螺仪，status=" << state.status_code
                          << "                    " << std::flush;
            } else if (!state.valid || !state.projected_gravity_valid) {
                std::cout << "\r状态无效，status=" << state.status_code
                          << "                    " << std::flush;
            } else {
                std::cout << "\rRPY=(" << std::setw(7) << raw.angles.roll << ","
                          << std::setw(7) << raw.angles.pitch << ","
                          << std::setw(7) << raw.angles.yaw << ") deg  g_body=("
                          << std::setw(7) << state.projected_gravity[0] << ","
                          << std::setw(7) << state.projected_gravity[1] << ","
                          << std::setw(7) << state.projected_gravity[2] << ")"
                          << std::flush;
            }
        }
        usleep(20000);  // 状态估计频率 50 Hz
    }

    std::cout << "\n测试结束\n";
    return 0;
}
