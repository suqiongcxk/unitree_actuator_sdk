#include "state_estimator.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << "[FAIL] " << message << std::endl;
    return condition;
}

EstimatedState update(PassthroughEstimator& estimator,
                      const JY901S_Quaternion& quat,
                      uint64_t imu_timestamp_ns,
                      uint64_t now_ns,
                      bool imu_valid = true,
                      JY901S_GyroData gyro = {0.0f, 0.0f, 0.0f},
                      JY901S_ACCData acc = {0.0f, 0.0f, 9.80665f})
{
    IMURawData imu{};
    imu.quat = quat;
    imu.gyro = gyro;
    imu.acc = acc;
    imu.timestamp_ns = imu_timestamp_ns;
    imu.valid = imu_valid;

    float q[12] = {0};
    float dq[12] = {0};
    float tau[12] = {0};
    int error[12] = {0};
    bool joint_valid[12];
    uint64_t joint_age_ns[12] = {0};
    uint32_t joint_failure_count[12] = {0};
    for (bool& valid : joint_valid) valid = true;
    return estimator.update(imu, q, dq, tau, error, joint_valid,
                            joint_age_ns, joint_failure_count, now_ns);
}

float quaternionNorm(const EstimatedState& state)
{
    return std::sqrt(state.orientation[0] * state.orientation[0]
                   + state.orientation[1] * state.orientation[1]
                   + state.orientation[2] * state.orientation[2]
                   + state.orientation[3] * state.orientation[3]);
}

}  // namespace

int main()
{
    bool ok = true;
    constexpr uint64_t now = 1'000'000'000ULL;
    GyroEstimatorConfig no_calibration;
    no_calibration.calibration_samples = 0;
    PassthroughEstimator estimator(100'000'000ULL, no_calibration);

    // 非单位输入应被归一化，而不是直接传给 NN。
    EstimatedState normalized = update(estimator, {1.2f, 0.0f, 0.0f, 0.0f}, now, now);
    ok &= expect(normalized.valid, "可归一化四元数应有效");
    ok &= expect(std::abs(quaternionNorm(normalized) - 1.0f) < 1e-6f,
                 "输出四元数模长应为 1");
    ok &= expect(normalized.projected_gravity_valid &&
                 std::abs(normalized.projected_gravity[0]) < 1e-6f &&
                 std::abs(normalized.projected_gravity[1]) < 1e-6f &&
                 std::abs(normalized.projected_gravity[2] + 1.0f) < 1e-6f,
                 "水平姿态的投影重力应为 [0,0,-1]");

    // q 和 -q 表示同一姿态，输出必须保持符号连续。
    EstimatedState same_rotation = update(estimator, {-1.0f, 0.0f, 0.0f, 0.0f},
                                          now + 20'000'000ULL, now + 20'000'000ULL);
    ok &= expect(same_rotation.valid && same_rotation.orientation[0] > 0.0f,
                 "q/-q 不应造成符号跳变");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    EstimatedState nonfinite = update(estimator, {nan, 0.0f, 0.0f, 1.0f}, now, now);
    ok &= expect(!nonfinite.valid &&
                 nonfinite.status_code == static_cast<int>(EstimateStatus::QUATERNION_NONFINITE),
                 "NaN 四元数必须被拒绝");

    EstimatedState zero = update(estimator, {0.0f, 0.0f, 0.0f, 0.0f}, now, now);
    ok &= expect(!zero.valid &&
                 zero.status_code == static_cast<int>(EstimateStatus::QUATERNION_NORM_INVALID),
                 "零四元数必须被拒绝");

    EstimatedState stale = update(estimator, {1.0f, 0.0f, 0.0f, 0.0f},
                                  now - 100'000'001ULL, now);
    ok &= expect(!stale.valid &&
                 stale.status_code == static_cast<int>(EstimateStatus::IMU_TIMEOUT),
                 "超时 IMU 数据必须被拒绝");

    EstimatedState future = update(estimator, {1.0f, 0.0f, 0.0f, 0.0f}, now + 1, now);
    ok &= expect(!future.valid &&
                 future.status_code == static_cast<int>(EstimateStatus::IMU_TIMESTAMP_INVALID),
                 "未来时间戳必须被拒绝");

    EstimatedState read_failure = update(estimator, {1.0f, 0.0f, 0.0f, 0.0f}, now, now, false);
    ok &= expect(!read_failure.valid &&
                 read_failure.status_code == static_cast<int>(EstimateStatus::IMU_READ_FAILED),
                 "I2C 读取失败必须传播到估计状态");

    constexpr float sqrt_half = 0.70710678118f;
    PassthroughEstimator gravity_estimator(100'000'000ULL, no_calibration);
    EstimatedState pitch_down = update(gravity_estimator,
        {sqrt_half, 0.0f, sqrt_half, 0.0f}, now, now);
    ok &= expect(pitch_down.projected_gravity[0] > 0.999f &&
                 std::abs(pitch_down.projected_gravity[1]) < 1e-5f &&
                 std::abs(pitch_down.projected_gravity[2]) < 1e-5f,
                 "头部下沉 90 度时重力应指向机体 +X");

    PassthroughEstimator roll_estimator(100'000'000ULL, no_calibration);
    EstimatedState left_down = update(roll_estimator,
        {sqrt_half, -sqrt_half, 0.0f, 0.0f}, now, now);
    ok &= expect(left_down.projected_gravity[1] > 0.999f &&
                 std::abs(left_down.projected_gravity[0]) < 1e-5f &&
                 std::abs(left_down.projected_gravity[2]) < 1e-5f,
                 "左侧下沉 90 度时重力应指向机体 +Y");

    PassthroughEstimator yaw_estimator(100'000'000ULL, no_calibration);
    EstimatedState yaw_only = update(yaw_estimator,
        {sqrt_half, 0.0f, 0.0f, sqrt_half}, now, now);
    ok &= expect(std::abs(yaw_only.projected_gravity[0]) < 1e-5f &&
                 std::abs(yaw_only.projected_gravity[1]) < 1e-5f &&
                 yaw_only.projected_gravity[2] < -0.999f,
                 "纯 Yaw 转动不应改变投影重力");

    // 静止样本达到要求后，应估出残余零偏并将静止角速度压到零附近。
    GyroEstimatorConfig gyro_config;
    gyro_config.calibration_samples = 4;
    PassthroughEstimator gyro_estimator(100'000'000ULL, gyro_config);
    EstimatedState calibrated;
    for (int i = 0; i < 4; ++i) {
        const uint64_t timestamp = now + static_cast<uint64_t>(i) * 20'000'000ULL;
        calibrated = update(gyro_estimator, {1.0f, 0.0f, 0.0f, 0.0f},
                            timestamp, timestamp, true,
                            {0.010f, -0.020f, 0.005f});
    }
    ok &= expect(calibrated.valid && calibrated.gyro_valid && calibrated.gyro_calibrated,
                 "连续静止样本应完成陀螺仪校准");
    ok &= expect(std::abs(calibrated.gyro_bias[0] - 0.010f) < 1e-6f &&
                 std::abs(calibrated.gyro_bias[1] + 0.020f) < 1e-6f &&
                 std::abs(calibrated.gyro_bias[2] - 0.005f) < 1e-6f,
                 "校准零偏应等于静止样本均值");
    ok &= expect(std::abs(calibrated.angular_velocity[0]) < 1e-6f &&
                 std::abs(calibrated.angular_velocity[1]) < 1e-6f &&
                 std::abs(calibrated.angular_velocity[2]) < 1e-6f,
                 "校准后静止角速度应接近零");

    // 校准中发生运动必须清空进度，不能把运动速度保存成零偏。
    PassthroughEstimator moving_estimator(100'000'000ULL, gyro_config);
    for (int i = 0; i < 2; ++i) {
        const uint64_t timestamp = now + static_cast<uint64_t>(i) * 20'000'000ULL;
        update(moving_estimator, {1.0f, 0.0f, 0.0f, 0.0f}, timestamp, timestamp,
               true, {0.01f, 0.0f, 0.0f});
    }
    update(moving_estimator, {1.0f, 0.0f, 0.0f, 0.0f}, now + 40'000'000ULL,
           now + 40'000'000ULL, true, {0.20f, 0.0f, 0.0f});
    EstimatedState after_three_new_samples;
    for (int i = 0; i < 3; ++i) {
        const uint64_t timestamp = now + static_cast<uint64_t>(i + 3) * 20'000'000ULL;
        after_three_new_samples = update(moving_estimator,
            {1.0f, 0.0f, 0.0f, 0.0f}, timestamp, timestamp,
            true, {0.01f, 0.0f, 0.0f});
    }
    ok &= expect(!after_three_new_samples.gyro_calibrated &&
                 after_three_new_samples.status_code ==
                     static_cast<int>(EstimateStatus::GYRO_CALIBRATING),
                 "校准期间运动必须重置静止采样进度");

    // 软件低通必须降低单帧阶跃，同时保持方向正确。
    GyroEstimatorConfig filter_config = no_calibration;
    filter_config.lowpass_cutoff_hz = 5.0f;
    PassthroughEstimator filter_estimator(100'000'000ULL, filter_config);
    update(filter_estimator, {1.0f, 0.0f, 0.0f, 0.0f}, now, now);
    EstimatedState filtered = update(filter_estimator,
        {1.0f, 0.0f, 0.0f, 0.0f}, now + 20'000'000ULL, now + 20'000'000ULL,
        true, {1.0f, 0.0f, 0.0f});
    ok &= expect(filtered.angular_velocity[0] > 0.0f && filtered.angular_velocity[0] < 1.0f,
                 "低通滤波应平滑角速度阶跃且保持符号");

    PassthroughEstimator bad_gyro_estimator(100'000'000ULL, no_calibration);
    EstimatedState bad_gyro = update(bad_gyro_estimator,
        {1.0f, 0.0f, 0.0f, 0.0f}, now, now, true, {nan, 0.0f, 0.0f});
    ok &= expect(!bad_gyro.valid &&
                 bad_gyro.status_code == static_cast<int>(EstimateStatus::GYRO_NONFINITE),
                 "NaN 角速度必须被拒绝");

    PassthroughEstimator range_estimator(100'000'000ULL, no_calibration);
    EstimatedState out_of_range = update(range_estimator,
        {1.0f, 0.0f, 0.0f, 0.0f}, now, now, true, {36.0f, 0.0f, 0.0f});
    ok &= expect(!out_of_range.valid &&
                 out_of_range.status_code == static_cast<int>(EstimateStatus::GYRO_RANGE_INVALID),
                 "超量程角速度必须被拒绝");

    PassthroughEstimator jump_estimator(100'000'000ULL, no_calibration);
    update(jump_estimator, {1.0f, 0.0f, 0.0f, 0.0f}, now, now);
    EstimatedState jump = update(jump_estimator,
        {1.0f, 0.0f, 0.0f, 0.0f}, now + 20'000'000ULL, now + 20'000'000ULL,
        true, {11.0f, 0.0f, 0.0f});
    ok &= expect(!jump.valid &&
                 jump.status_code == static_cast<int>(EstimateStatus::GYRO_JUMP_INVALID),
                 "异常角速度突跳必须被拒绝");

    // 12 个唯一标记必须保持 motor-ID 下标，且任一无效反馈会使整帧失效。
    IMURawData mapping_imu{};
    mapping_imu.quat = {1.0f, 0.0f, 0.0f, 0.0f};
    mapping_imu.acc = {0.0f, 0.0f, 9.80665f};
    mapping_imu.timestamp_ns = now;
    mapping_imu.valid = true;
    float unique_q[12], unique_dq[12], unique_tau[12];
    int unique_error[12] = {0};
    bool unique_valid[12];
    uint64_t unique_age[12] = {0};
    uint32_t unique_failures[12] = {0};
    for (int i = 0; i < 12; ++i) {
        unique_q[i] = 100.0f + i;
        unique_dq[i] = 200.0f + i;
        unique_tau[i] = 300.0f + i;
        unique_valid[i] = true;
    }
    PassthroughEstimator mapping_estimator(100'000'000ULL, no_calibration);
    EstimatedState mapped = mapping_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now);
    bool mapping_ok = mapped.valid && mapped.joint_feedback_valid;
    for (int i = 0; i < 12; ++i) {
        mapping_ok = mapping_ok && mapped.joint_position[i] == unique_q[i]
                                && mapped.joint_velocity[i] == unique_dq[i]
                                && mapped.joint_torque[i] == unique_tau[i];
    }
    ok &= expect(mapping_ok, "12 个关节唯一标记必须保持 motor-ID 下标");

    unique_valid[7] = false;
    EstimatedState missing_joint = mapping_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now);
    ok &= expect(!missing_joint.valid && !missing_joint.joint_feedback_valid &&
                 !missing_joint.joint_valid[7] &&
                 missing_joint.status_code ==
                     static_cast<int>(EstimateStatus::JOINT_FEEDBACK_INVALID),
                 "任一关节 CRC/反馈无效时整帧必须失效");

    // 电机 CRC 即使仍为 true，超过 100ms 的旧反馈也必须失效。
    unique_valid[7] = true;
    unique_age[7] = 100'000'001ULL;
    PassthroughEstimator stale_motor_estimator(100'000'000ULL, no_calibration);
    EstimatedState stale_motor = stale_motor_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now);
    ok &= expect(!stale_motor.valid && !stale_motor.joint_valid[7] &&
                 stale_motor.status_code ==
                     static_cast<int>(EstimateStatus::MOTOR_FEEDBACK_TIMEOUT),
                 "超过 100ms 的电机反馈必须被拒绝");

    // 估计时钟倒退和异常 dt 必须被识别，而且下一帧可以恢复。
    unique_age[7] = 0;
    PassthroughEstimator time_estimator(100'000'000ULL, no_calibration);
    EstimatedState time_first = time_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now);
    mapping_imu.timestamp_ns = now - 20'000'000ULL;
    EstimatedState backwards = time_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now - 20'000'000ULL);
    ok &= expect(time_first.valid && !backwards.valid &&
                 backwards.status_code ==
                     static_cast<int>(EstimateStatus::ESTIMATION_TIMESTAMP_INVALID) &&
                 backwards.consecutive_invalid_count == 1,
                 "估计时间戳倒退必须被拒绝");

    mapping_imu.timestamp_ns = now + 180'000'000ULL;
    EstimatedState large_dt = time_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now + 180'000'000ULL);
    ok &= expect(!large_dt.valid &&
                 large_dt.status_code == static_cast<int>(EstimateStatus::ESTIMATION_DT_INVALID),
                 "超过 100ms 的估计 dt 必须被拒绝");

    mapping_imu.timestamp_ns = now + 200'000'000ULL;
    EstimatedState recovered_time = time_estimator.update(
        mapping_imu, unique_q, unique_dq, unique_tau, unique_error, unique_valid,
        unique_age, unique_failures, now + 200'000'000ULL);
    ok &= expect(recovered_time.valid &&
                 std::abs(recovered_time.dt_sec - 0.02f) < 1e-5f &&
                 recovered_time.consecutive_invalid_count == 0,
                 "异常 dt 后下一帧应能恢复，且 dt 为 20ms");

    if (!ok) return 1;
    std::cout << "[PASS] State estimator quaternion and gyro safety tests" << std::endl;
    return 0;
}
