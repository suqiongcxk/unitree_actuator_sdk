#ifndef __ROBOT_CONTROL_STATE_ESTIMATOR_H
#define __ROBOT_CONTROL_STATE_ESTIMATOR_H

#include "shared_data.h"
#include "legged_odometry.h"

enum class EstimateStatus {
    OK = 0,
    IMU_READ_FAILED,
    IMU_TIMESTAMP_INVALID,
    IMU_TIMEOUT,
    QUATERNION_NONFINITE,
    QUATERNION_NORM_INVALID,
    GYRO_CALIBRATING,
    GYRO_NONFINITE,
    GYRO_RANGE_INVALID,
    GYRO_JUMP_INVALID,
    JOINT_FEEDBACK_INVALID,
    MOTOR_FEEDBACK_TIMEOUT,
    ESTIMATION_TIMESTAMP_INVALID,
    ESTIMATION_DT_INVALID,
    LINEAR_KF_INVALID,
};

struct GyroEstimatorConfig {
    int calibration_samples = 50;                 // 1 s @ 50 Hz
    float stationary_gyro_threshold = 0.08f;      // rad/s
    float stationary_acc_tolerance = 0.8f;        // |norm(acc)-g|, m/s^2
    float max_calibration_stddev = 0.015f;         // rad/s
    float lowpass_cutoff_hz = 15.0f;
    float max_abs_gyro = 35.0f;                    // 略高于 ±2000 deg/s
    float max_gyro_jump = 10.0f;                   // 相邻有效帧 rad/s
};

// ═══════════════════════════════════════════════════════════════════════════════
//  StateEstimator — 状态估计抽象接口
// ═══════════════════════════════════════════════════════════════════════════════
//
//  输入:  IMU原始数据 + 12电机关节反馈
//  输出:  EstimatedState (机体姿态速度 + 关节状态)
//
//  后续替换为真正的 Kalman / Complementary / EKF 时，只需实现此接口。

class StateEstimator {
public:
    virtual ~StateEstimator() = default;

    /// 执行一次状态估计更新
    /// @param imu              最新 IMU 数据
    /// @param joint_position   12 电机输出端位置 (rad)
    /// @param joint_velocity   12 电机输出端速度 (rad/s)
    /// @param joint_torque     12 电机输出端力矩 (N·m)
    /// @param joint_error      12 电机错误码
    /// @param now_ns           当前时间戳 (CLOCK_MONOTONIC, ns)
    /// @return                 估计状态
    virtual EstimatedState update(
        const IMURawData& imu,
        const float* joint_position,
        const float* joint_velocity,
        const float* joint_torque,
        const int*   joint_error,
        const bool*  joint_valid,
        const uint64_t* joint_age_ns,
        const uint32_t* joint_failure_count,
        uint64_t now_ns
    ) = 0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  PassthroughEstimator — 占位实现 (直通拷贝，不做滤波)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  PLACEHOLDER — 替换为真正的状态估计算法:
//    - Kalman Filter (线性)
//    - Extended Kalman Filter (非线性)
//    - Complementary Filter (简单高效)
//    - Mahony/Madgwick Filter (姿态专用)

class PassthroughEstimator : public StateEstimator {
public:
    explicit PassthroughEstimator(uint64_t imu_timeout_ns = 100'000'000ULL);
    PassthroughEstimator(uint64_t imu_timeout_ns, const GyroEstimatorConfig& gyro_config);

    EstimatedState update(
        const IMURawData& imu,
        const float* joint_position,
        const float* joint_velocity,
        const float* joint_torque,
        const int*   joint_error,
        const bool*  joint_valid,
        const uint64_t* joint_age_ns,
        const uint32_t* joint_failure_count,
        uint64_t now_ns
    ) override;

private:
    void resetGyroCalibration();

    uint64_t imu_timeout_ns_;
    GyroEstimatorConfig gyro_config_;
    float previous_quaternion_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bool has_previous_quaternion_ = false;
    float gyro_bias_[3] = {0};
    float gyro_cal_mean_[3] = {0};
    float gyro_cal_m2_[3] = {0};
    int gyro_cal_samples_ = 0;
    bool gyro_calibrated_ = false;
    float filtered_gyro_[3] = {0};
    float previous_raw_gyro_[3] = {0};
    bool has_filtered_gyro_ = false;
    bool has_previous_raw_gyro_ = false;
    uint64_t previous_gyro_timestamp_ns_ = 0;
    uint64_t previous_estimation_timestamp_ns_ = 0;
    uint32_t consecutive_invalid_count_ = 0;
    LeggedOdometry legged_odometry_;
};

// 当前 Step 1~7 的 IMU/腿运动学互补估计后端。
using ComplementaryStateEstimator = PassthroughEstimator;

#endif  // __ROBOT_CONTROL_STATE_ESTIMATOR_H
