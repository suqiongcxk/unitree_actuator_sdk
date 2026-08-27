#ifndef __ROBOT_CONTROL_ROBOT_CONTROLLER_H
#define __ROBOT_CONTROL_ROBOT_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <array>
#include <mutex>
#include "shared_data.h"
#include "nn_validation.h"
#include "../motor_lib/ZeroPointCalibration.h"

// 前向声明 — 避免在此头文件中引入重型依赖
class JY901S;
class MultiBusController;
class NNPolicy;

enum class StateEstimatorBackend {
    COMPLEMENTARY = 0,
    LINEAR_KF = 1,
};

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
    std::string imu_device = "/dev/i2c-1";
    int imu_hz = 200;

    // ── 状态估计 ──
    int estimation_hz = 50;
    StateEstimatorBackend estimator_backend = StateEstimatorBackend::COMPLEMENTARY;
    // [vx, vy, yaw_rate]，base frame；默认无遥控命令。
    std::array<float, 3> velocity_command{{0.0f, 0.0f, 0.0f}};

    // ── 电机总线 ──
    int motor_hz = 500;                     // 每路总线控制频率
    std::vector<BusConfig> buses;
    // Step 5 实机安全诊断：跳过机械标定/站立，电机始终只发送阻尼。
    bool monitor_only = false;
    // Step 7 实机诊断：正常执行机械标定/缓慢站立，但禁止 NN 下发并输出详细里程计。
    bool step7_diagnostics = false;
    // 仅 Step 7 诊断：延迟后只在估计器输入副本中注入单腿异常速度。
    int simulated_leg_slip = -1;             // -1=关闭，0..3=FL/FR/RL/RR
    int simulated_leg_slip_delay_ms = 8000;  // 留出站立过渡和稳定时间
    // 仅 monitor_only：启动估计线程 3 秒后模拟指定电机反馈停止刷新。
    int simulated_motor_timeout_id = -1;
    int simulated_fault_delay_ms = 3000;
    bool simulated_imu_stream_loss = false;

    // ── 神经网络 ──
    int nn_hz = 50;

    /// ONNX 模型文件路径 (空字符串 = 使用 StandingPolicy)
    std::string onnx_model_path;

    /// ONNX 模型输出缩放因子 (训练时使用的 action_scale)
    float action_scale = 0.25f;

    /// 开环站立指令平滑过渡到 ONNX 指令的时间；仅正常 ONNX 模式启用。
    float nn_takeover_duration_sec = 2.0f;

    // ── NN 验证控制 ──
    NNControlFlags nn_flags;  // 干运行 / 记录 / 验证 / 对比

    // ── 默认 PD 增益 ──
    float default_kp = 0.625f;
    float default_kd = 0.0125f;

    // ── 安全退出阻尼 ──
    // 仅用于急停/正常退出后的 12 电机阻尼锁存，不改变正常控制 PD 增益。
    float emergency_damping_kd = 0.136f;  // 当前 0.068 的 2 倍

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

    /// 请求急停；只发布停止状态，不在调用线程中执行通信清理。
    void requestEmergencyStop() noexcept;

    /// 幂等安全退出：停止上层控制、全电机阻尼、回收所有线程。
    void safeShutdown();

    bool isRunning() const;
    /// 线程安全更新 NN 速度命令 [vx,vy,yaw_rate]。拒绝 NaN/Inf。
    bool setVelocityCommand(float vx, float vy, float yaw_rate);

    // ── 监控接口 (线程安全) ──────────────────────────────────────────────

    bool getLatestIMUData(IMURawData& out) const;
    bool getLatestEstimatedState(EstimatedState& out) const;
    int getIMUHz()       const { return config_.imu_hz; }
    int getEstimationHz() const { return config_.estimation_hz; }
    int getMotorHz()     const { return config_.motor_hz; }
    int getNNHz()        const { return config_.nn_hz; }

    // ── 工厂方法 ─────────────────────────────────────────────────────────

    /// 获取默认的 4 腿 12 电机硬件配置
    static RobotControlConfig getDefaultConfig();

    // ── 标定与启动状态 ─────────────────────────────────────────────────────

    /// 获取当前启动阶段
    StartupPhase getStartupPhase() const { return startup_phase_; }

    /// 获取标定结果
    const JointCalibResult* getCalibrationResults() const { return calib_results_; }

    /// 标定是否已完成
    bool isCalibrationComplete() const { return calibration_completed_; }

    /// 标定成功关节数
    int getCalibrationOKCount() const { return calibration_ok_count_; }

private:
    RobotControlConfig config_;

    // ── 硬件 ─────────────────────────────────────────────────────────────

    std::unique_ptr<JY901S>             imu_;
    std::unique_ptr<MultiBusController> motor_ctrl_;

    // ── 共享数据缓冲 ─────────────────────────────────────────────────────

    IMUBuffer            imu_buffer_;
    EstimatedStateBuffer est_buffer_;

    // 终端监控使用独立值快照，避免成为无锁双缓冲的第二个消费者。
    mutable std::mutex monitor_imu_mutex_;
    mutable std::mutex monitor_est_mutex_;
    IMURawData monitor_imu_snapshot_{};
    EstimatedState monitor_est_snapshot_{};
    bool monitor_imu_available_ = false;
    bool monitor_est_available_ = false;

    // ── NN 推理与验证 ────────────────────────────────────────────────────

    std::unique_ptr<NNPolicy> nn_policy_;
    std::unique_ptr<NNInferenceLogger> nn_logger_;
    std::array<float, 3> velocity_command_{{0.0f, 0.0f, 0.0f}};
    std::mutex velocity_command_mutex_;

    // ── 线程 ─────────────────────────────────────────────────────────────

    std::thread imu_thread_;
    std::thread est_thread_;
    std::thread nn_thread_;
    // 4 路电机总线线程由 MultiBusController 内部管理

    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_started_{false};

    // ── 标定与启动 ───────────────────────────────────────────────────────

    StartupPhase      startup_phase_ = StartupPhase::INIT_COMM;
    JointCalibResult  calib_results_[12] = {};
    float             post_calib_position_[12] = {};
    bool              calibrated_by_motor_id_[12] = {};
    bool              calibration_completed_ = false;
    int               calibration_ok_count_ = 0;

    /// 执行完整标定序列 (初始化阶段调用)
    bool runCalibrationSequence();

    /// 复用已实机验证的站立前大腿安全预定位。
    bool prepositionThighsForStanding();

    /// 从标定位姿平滑过渡到站立姿态 (start 阶段调用)
    bool transitionToStandingInternal(float transition_time_sec = 3.0f);

    // ── 线程主循环 ───────────────────────────────────────────────────────

    void imuLoop();
    void estimationLoop();
    void nnLoop();

    // ── 工具 ─────────────────────────────────────────────────────────────

    /// 启动时等待 IMU 产生初始数据
    bool waitForIMUData(int timeout_ms = 500);

    /// 等待静止陀螺仪零偏校准完成；必须在电机总线启动前调用。
    bool waitForEstimatorReady(int timeout_ms = 3000);

    /// 启动时等待电机总线就绪
    bool waitForMotorReady(int timeout_ms = 1000);
};

#endif  // __ROBOT_CONTROL_ROBOT_CONTROLLER_H
