#ifndef __ROBOT_CONTROL_STATE_ESTIMATOR_H
#define __ROBOT_CONTROL_STATE_ESTIMATOR_H

#include "shared_data.h"

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
    EstimatedState update(
        const IMURawData& imu,
        const float* joint_position,
        const float* joint_velocity,
        const float* joint_torque,
        const int*   joint_error,
        uint64_t now_ns
    ) override;
};

#endif  // __ROBOT_CONTROL_STATE_ESTIMATOR_H
