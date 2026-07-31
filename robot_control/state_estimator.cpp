#include "state_estimator.h"
#include <cstring>

// ═══════════════════════════════════════════════════════════════════════════════
//  PassthroughEstimator — PLACEHOLDER
// ═══════════════════════════════════════════════════════════════════════════════
//
//  不做任何滤波，仅将输入数据直通拷贝到 EstimatedState。
//  后续替换为真正的 Kalman / Complementary Filter。

EstimatedState PassthroughEstimator::update(
    const IMURawData& imu,
    const float* joint_position,
    const float* joint_velocity,
    const float* joint_torque,
    const int*   joint_error,
    uint64_t now_ns)
{
    EstimatedState est;

    // ── 机体状态 ──
    // 位置: 无外部参考，保持为0
    est.position[0] = 0.0f;
    est.position[1] = 0.0f;
    est.position[2] = 0.0f;

    // 姿态: 使用 JY901S 硬件解算的四元数
    est.orientation[0] = imu.quat.q0;  // w
    est.orientation[1] = imu.quat.q1;  // x
    est.orientation[2] = imu.quat.q2;  // y
    est.orientation[3] = imu.quat.q3;  // z

    // 线速度: 未估计，保持为0
    // TODO: 通过加速度积分 + 运动学约束估计
    est.linear_velocity[0] = 0.0f;
    est.linear_velocity[1] = 0.0f;
    est.linear_velocity[2] = 0.0f;

    // 角速度: 直接使用陀螺仪数据
    est.angular_velocity[0] = imu.gyro.x;
    est.angular_velocity[1] = imu.gyro.y;
    est.angular_velocity[2] = imu.gyro.z;

    // ── 关节状态 (12 电机) ──
    std::memcpy(est.joint_position, joint_position, 12 * sizeof(float));
    std::memcpy(est.joint_velocity, joint_velocity, 12 * sizeof(float));
    std::memcpy(est.joint_torque,   joint_torque,   12 * sizeof(float));
    std::memcpy(est.joint_error,    joint_error,    12 * sizeof(int));

    // ── 接触状态 ──
    // TODO: 通过足端力传感器或运动学判断
    est.contact[0] = false;
    est.contact[1] = false;
    est.contact[2] = false;
    est.contact[3] = false;

    est.timestamp_ns = now_ns;
    return est;
}
