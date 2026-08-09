#ifndef __ROBOT_CONTROL_SHARED_DATA_H
#define __ROBOT_CONTROL_SHARED_DATA_H

#include <atomic>
#include <cstdint>
#include "jy901s.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  Lock-Free Double-Buffer — 单写者 + 单读者 的无锁数据交换
// ═══════════════════════════════════════════════════════════════════════════════
//
//  内存模型:
//    写者 ──写入──▶ slots_[write_idx_]  ──commitWrite()──▶ 原子翻转索引
//    读者 ──tryAcquireRead()──▶ slots_[1-write_idx_] ◀── 只读
//
//  两个槽永远不会同时被读写。无需互斥锁。

template <typename T>
class DoubleBuffer {
public:
    DoubleBuffer() {
        write_idx_.store(0, std::memory_order_relaxed);
        seq_.store(0, std::memory_order_relaxed);
    }

    // ── Writer API (只有生产者线程调用) ──────────────────────────────────────

    /// 获取写槽的引用。直接写入槽内内存，不拷贝。
    T& acquireWriteSlot() {
        return slots_[write_idx_.load(std::memory_order_relaxed)];
    }

    /// 提交写入: 原子翻转 write_idx_ + 递增序列号。
    /// release 屏障确保所有数据写入在索引翻转之前对所有线程可见。
    void commitWrite() {
        int new_write = 1 - write_idx_.load(std::memory_order_relaxed);
        write_idx_.store(new_write, std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);
    }

    // ── Reader API (只有消费者线程调用) ──────────────────────────────────────

    /// 尝试获取最新数据的只读指针。
    /// @return 指向最新完成数据的指针，或 nullptr（无新数据）。
    /// acquire 屏障确保读取在索引翻转之后进行。
    const T* tryAcquireRead() {
        uint32_t current_seq = seq_.load(std::memory_order_acquire);
        if (current_seq == last_read_seq_) {
            return nullptr;
        }
        last_read_seq_ = current_seq;
        int read_slot = 1 - write_idx_.load(std::memory_order_acquire);
        return &slots_[read_slot];
    }

private:
    T slots_[2];
    std::atomic<int>      write_idx_{0};
    std::atomic<uint32_t> seq_{0};
    uint32_t              last_read_seq_{0};  // 仅消费者线程访问，无需 atomic
};

// ═══════════════════════════════════════════════════════════════════════════════
//  IMU 原始数据 (IMU Thread → State Estimation Thread)
// ═══════════════════════════════════════════════════════════════════════════════

struct IMURawData {
    JY901S_AngleData    angles;         // roll, pitch, yaw (度)
    JY901S_ACCData      acc;            // x, y, z (m/s²)
    JY901S_GyroData     gyro;           // x, y, z (rad/s)
    JY901S_Quaternion   quat;           // w, x, y, z
    uint64_t            timestamp_ns;   // CLOCK_MONOTONIC
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
    float angular_velocity[3] = {0};   // body-frame (rad/s)

    // ── 12 电机关节状态 (下标严格等于 motor ID: 0..11) ──
    // 输出端量纲: rad, rad/s, N·m
    float joint_position[12]  = {0};
    float joint_velocity[12]  = {0};
    float joint_torque[12]    = {0};
    int   joint_error[12]     = {0};

    // ── 接触状态 ──
    bool  contact[4]          = {false};

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
