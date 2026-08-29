#ifndef __ROBOT_CONTROL_SHARED_DATA_H
#define __ROBOT_CONTROL_SHARED_DATA_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include "jy901s.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  SnapshotBuffer — 单写者 + 多读者的线程安全快照交换
// ═══════════════════════════════════════════════════════════════════════════════
//
//  写者只修改writer_slot_；commitWrite()在短临界区内发布一个值副本。
//  读者同样在短临界区内取得自己的值副本，离开锁后可任意时长使用。
//  每个读者持有独立Sequence，因此启动等待、控制线程和监控读者互不干扰。

template <typename T>
class DoubleBuffer {
public:
    using Sequence = uint64_t;

    // ── Writer API (只有生产者线程调用) ──────────────────────────────────────

    /// 获取写槽的引用。直接写入槽内内存，不拷贝。
    T& acquireWriteSlot() {
        return writer_slot_;
    }

    /// 提交写入。锁只覆盖一次结构体赋值与序列号递增。
    void commitWrite() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        published_ = writer_slot_;
        ++sequence_;
    }

    // ── Reader API (支持多个消费者线程) ──────────────────────────────────────

    /// 若存在相对该读者更新的帧，则复制到out并更新reader_sequence。
    bool tryRead(T& out, Sequence& reader_sequence) const {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (sequence_ == 0 || sequence_ == reader_sequence) return false;
        out = published_;
        reader_sequence = sequence_;
        return true;
    }

private:
    T writer_slot_{};                   // 仅生产者访问
    T published_{};                     // 只在snapshot_mutex_内访问
    mutable std::mutex snapshot_mutex_;
    Sequence sequence_ = 0;             // 只在snapshot_mutex_内访问
};

// ═══════════════════════════════════════════════════════════════════════════════
//  IMU 原始数据 (IMU Thread → State Estimation Thread)
// ═══════════════════════════════════════════════════════════════════════════════

struct IMURawData {
    JY901S_AngleData    angles;         // roll, pitch, yaw (度)
    JY901S_ACCData      acc;            // x, y, z (m/s²)
    JY901S_GyroData     gyro;           // x, y, z (rad/s)
    JY901S_Quaternion   quat;           // w, x, y, z
    uint64_t            timestamp_ns = 0; // CLOCK_MONOTONIC
    bool                valid = false;  // 本周期所有必需 IMU 数据均读取成功
};

using IMUBuffer = DoubleBuffer<IMURawData>;

// ═══════════════════════════════════════════════════════════════════════════════
//  估计状态 (State Estimation Thread → NN Thread)
// ═══════════════════════════════════════════════════════════════════════════════

struct EstimatedState {
    // ── 机体状态 (world frame) ──
    float position[3]         = {0};   // x, y, z (m)
    float orientation[4]      = {0};   // quaternion w, x, y, z
    float linear_velocity[3]  = {0};   // dx, dy, dz (m/s)
    float body_linear_velocity[3] = {0}; // body frame, m/s
    float angular_velocity[3] = {0};   // body-frame (rad/s)
    float projected_gravity[3] = {0};  // 单位重力在机体坐标系中的方向

    // ── 12 电机关节状态 (下标严格等于 motor ID: 0..11) ──
    // q/dq为关节输出端；tau保持GO-M8010-6 SDK回传的转子侧N·m，
    // 进入足端力估计前再乘6.333，避免影响既有控制接口和日志兼容性。
    float joint_position[12]  = {0};
    float joint_velocity[12]  = {0};
    float joint_torque[12]    = {0};
    int   joint_error[12]     = {0};
    bool  joint_valid[12]     = {false}; // 最近成功反馈未超时、数值有限且 merror=0
    bool  joint_feedback_valid = false;  // 12 个关节本周期全部有效
    uint64_t joint_age_ns[12] = {0};     // 各关节最近成功反馈距本帧的年龄
    uint32_t joint_failure_count[12] = {0}; // 各关节连续通信失败次数
    uint64_t max_joint_age_ns = 0;

    // ── 接触状态 ──
    bool  contact[4]          = {false};
    float contact_confidence[4] = {0};   // FL,FR,RL,RR，0..1
    float foot_position[4][3] = {{0}};   // 相对 base，base frame，m
    float foot_velocity[4][3] = {{0}};   // 关节运动造成的相对速度，m/s
    float foot_force_body[4][3] = {{0}}; // 地面对足端的估算力，base frame，N
    float normal_force[4] = {0};         // +Z法向支撑力，N
    float foot_force_residual[4] = {0};  // 力矩重构残差，N·m
    bool  foot_force_valid[4] = {false};
    bool  contact_used_force[4] = {false}; // false表示求解无效并回退力矩范数
    float body_height         = 0.0f;    // base 原点到足底接触平面的高度，m
    float linear_velocity_confidence = 0.0f;
    bool  leg_odometry_valid = false;
    bool  airborne           = true;
    bool  slipping           = false;
    bool  landing_impact     = false;
    // 仅 Linear KF 后端填充；排列为 [base p, base v, 4×foot p]。
    float state_covariance[18][18] = {{0}};
    bool  covariance_valid = false;
    int   estimator_backend = 0;  // 0=Complementary, 1=LinearKF

    // Step 1: 姿态健康状态。无效状态不得进入 NN 控制。
    bool     orientation_valid   = false;
    bool     gyro_valid          = false;
    bool     projected_gravity_valid = false;
    bool     gyro_calibrated     = false;
    bool     valid               = false;
    float    quaternion_raw_norm = 0.0f;
    float    gyro_bias[3]        = {0};
    uint64_t imu_age_ns          = 0;
    float    dt_sec              = 0.0f;
    uint32_t consecutive_invalid_count = 0;
    int      status_code         = 0;

    uint64_t timestamp_ns     = 0;
};

using EstimatedStateBuffer = DoubleBuffer<EstimatedState>;

// ═══════════════════════════════════════════════════════════════════════════════
//  NN 输出指令 (NN Thread → Motor Bus Threads)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  注意: 不需要 DoubleBuffer。NN 线程直接调用 ParallelBus::setPosition(),
//  内部已有 mutex 保护。此结构体仅用于 NN 内部组织指令。

struct NNCommandSet {
    float joint_position_target[12] = {0};  // 目标输出位置 (rad)
    float kp[12]                    = {0};
    float kd[12]                    = {0};
    bool  valid                     = false;
};

#endif  // __ROBOT_CONTROL_SHARED_DATA_H
