#ifndef __ROBOT_CONTROL_ROBOT_CONTROLLER_H
#define __ROBOT_CONTROL_ROBOT_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include "shared_data.h"
#include "nn_validation.h"

// 前向声明 — 避免在此头文件中引入重型依赖
class JY901S;
class MultiBusController;
class NNPolicy;

// ═══════════════════════════════════════════════════════════════════════════════
//  BusConfig — 单路 RS-485 总线配置
// ═══════════════════════════════════════════════════════════════════════════════

struct BusConfig {
    int gpio_chip;                              // GPIO 芯片编号 (0, 1, 2...)
    int gpio_line;                              // GPIO 行偏移
    std::string serial_port;                    // 串口路径, e.g. "/dev/ttyS0"
    std::vector<unsigned short> motor_ids;      // 此总线上的电机 ID 列表
};

// ═══════════════════════════════════════════════════════════════════════════════
//  RobotControlConfig — 完整机器人控制配置
// ═══════════════════════════════════════════════════════════════════════════════

struct RobotControlConfig {
    // ── IMU ──
    std::string imu_device = "/dev/i2c-2";
    int imu_hz = 200;

    // ── 状态估计 ──
    int estimation_hz = 50;

    // ── 电机总线 ──
    int motor_hz = 500;                     // 每路总线控制频率
    std::vector<BusConfig> buses;

    // ── 神经网络 ──
    int nn_hz = 50;

    /// ONNX 模型文件路径 (空字符串 = 使用 StandingPolicy)
    std::string onnx_model_path;

    /// ONNX 模型输出缩放因子 (训练时使用的 action_scale)
    float action_scale = 0.25f;

    // ── NN 验证控制 ──
    NNControlFlags nn_flags;  // 干运行 / 记录 / 验证 / 对比

    // ── 默认 PD 增益 ──
    float default_kp = 0.625f;
    float default_kd = 0.0125f;

    // ── 默认站立姿态 (12 关节, 输出端 rad, Z字排序) ──
    // [0..3] = hip(4条腿), [4..7] = thigh(4条腿), [8..11] = lower_leg(4条腿)
    // 与 ZeroPointCalibration.cpp 的 default_joint_pos 完全一致
    float default_standing_pose[12] = {
        0.1f, -0.1f, 0.1f, -0.1f,     // hip: Leg1,Leg2,Leg3,Leg4
        0.8f,  0.8f, 1.0f,  1.0f,     // thigh
       -1.5f, -1.5f, -1.5f, -1.5f    // lower_leg
    };
};

// ═══════════════════════════════════════════════════════════════════════════════
//  RobotController — 多线程机器人控制器
// ═══════════════════════════════════════════════════════════════════════════════
//
//  管理 7 个线程:
//    1 x IMU 读取    (200 Hz)
//    4 x 电机总线    (500 Hz, 通过 MultiBusController 管理)
//    1 x 状态估计    ( 50 Hz)
//    1 x NN 推理     ( 50 Hz)
//
//  数据流:
//    IMU  ──DoubleBuffer──▶ 估计  ──DoubleBuffer──▶  NN
//    电机 ──mutex+copy──▶ 估计
//    NN   ──mutex+copy──▶ 电机

class RobotController {
public:
    explicit RobotController(const RobotControlConfig& config);
    ~RobotController();

    // ── 生命周期 ──────────────────────────────────────────────────────────

    /// 初始化硬件: IMU + 电机总线
    /// @return true 成功
    bool initialize();

    /// 启动所有线程 (按依赖顺序)
    /// @return true 成功
    bool start();

    /// 优雅关闭 (按依赖逆序)
    void stop();

    bool isRunning() const { return running_.load(); }

    // ── 监控接口 (线程安全) ──────────────────────────────────────────────

    const IMURawData*    getLatestIMUData();
    const EstimatedState* getLatestEstimatedState();
    int getIMUHz()       const { return config_.imu_hz; }
    int getEstimationHz() const { return config_.estimation_hz; }
    int getMotorHz()     const { return config_.motor_hz; }
    int getNNHz()        const { return config_.nn_hz; }

    // ── 工厂方法 ─────────────────────────────────────────────────────────

    /// 获取默认的 4 腿 12 电机硬件配置
    static RobotControlConfig getDefaultConfig();

private:
    RobotControlConfig config_;

    // ── 硬件 ─────────────────────────────────────────────────────────────

    std::unique_ptr<JY901S>             imu_;
    std::unique_ptr<MultiBusController> motor_ctrl_;

    // ── 共享数据缓冲 ─────────────────────────────────────────────────────

    IMUBuffer            imu_buffer_;
    EstimatedStateBuffer est_buffer_;

    // ── NN 推理与验证 ────────────────────────────────────────────────────

    std::unique_ptr<NNPolicy> nn_policy_;
    std::unique_ptr<NNInferenceLogger> nn_logger_;

    // ── 线程 ─────────────────────────────────────────────────────────────

    std::thread imu_thread_;
    std::thread est_thread_;
    std::thread nn_thread_;
    // 4 路电机总线线程由 MultiBusController 内部管理

    std::atomic<bool> running_{false};

    // ── 线程主循环 ───────────────────────────────────────────────────────

    void imuLoop();
    void estimationLoop();
    void nnLoop();

    // ── 工具 ─────────────────────────────────────────────────────────────

    /// 启动时等待 IMU 产生初始数据
    bool waitForIMUData(int timeout_ms = 500);

    /// 启动时等待电机总线就绪
    bool waitForMotorReady(int timeout_ms = 1000);
};

#endif  // __ROBOT_CONTROL_ROBOT_CONTROLLER_H
