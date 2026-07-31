#include "robot_controller.h"
#include "state_estimator.h"
#include "nn_policy.h"
#include "jy901s.h"
#include "parallel_bus.h"
#include "../motor_lib/motor_controller.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <ctime>
#include <cstring>
#include <chrono>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════════

RobotController::RobotController(const RobotControlConfig& config)
    : config_(config)
{}

RobotController::~RobotController()
{
    stop();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  默认配置 — 4 腿 12 电机
// ═══════════════════════════════════════════════════════════════════════════════

RobotControlConfig RobotController::getDefaultConfig()
{
    RobotControlConfig c;
    c.imu_device    = "/dev/i2c-2";
    c.imu_hz        = 200;
    c.estimation_hz = 50;
    c.motor_hz      = 500;
    c.nn_hz         = 50;
    c.default_kp    = 0.625f;
    c.default_kd    = 0.0125f;

    // 4 路 RS-485 总线, 每路 3 个电机 (hip / thigh / lower_leg)
    c.buses = {
        {0, 133, "/dev/ttyS0", {0, 4, 8}},      // Leg1
        {0, 39,  "/dev/ttyS6", {1, 5, 9}},      // Leg2
        {0, 35,  "/dev/ttyS7", {2, 6, 10}},     // Leg3
        {0, 63,  "/dev/ttyS4", {3, 7, 11}},     // Leg4
    };

    return c;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  initialize() — 初始化硬件
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::initialize()
{
    std::cout << "[RobotController] 初始化硬件..." << std::endl;

    // ── 1. 初始化 IMU ──
    imu_ = std::make_unique<JY901S>(config_.imu_device);
    JY901S_Status imu_st = imu_->init(config_.imu_hz);
    if (imu_st != JY901S_Status::OK) {
        std::cerr << "[RobotController] 错误: IMU 初始化失败 (status="
                  << static_cast<int>(imu_st) << ")" << std::endl;
        return false;
    }
    std::cout << "[RobotController] JY901S IMU 初始化成功, "
              << config_.imu_hz << "Hz" << std::endl;

    // ── 2. 创建 4 路并行总线 ──
    motor_ctrl_ = std::make_unique<MultiBusController>();
    for (const auto& bus_cfg : config_.buses) {
        ParallelBus& bus = motor_ctrl_->addBus(
            bus_cfg.gpio_chip,
            bus_cfg.gpio_line,
            bus_cfg.serial_port);
        for (unsigned short id : bus_cfg.motor_ids) {
            if (!bus.addMotor(id)) {
                std::cerr << "[RobotController] 错误: 无法注册电机 ID="
                          << id << " 到总线 " << bus_cfg.serial_port << std::endl;
                return false;
            }
        }
        std::cout << "[RobotController] 总线 " << bus_cfg.serial_port
                  << " (GPIO" << bus_cfg.gpio_line << ") "
                  << bus_cfg.motor_ids.size() << " 电机" << std::endl;
    }

    std::cout << "[RobotController] 硬件初始化完成 — "
              << config_.buses.size() << " 路总线, "
              << "共 " << (config_.buses.size() * 3) << " 电机" << std::endl;

    // ── 2.5. [新增] 上电标定序列 ──
    startup_phase_ = StartupPhase::INIT_COMM;
    if (!runCalibrationSequence()) {
        std::cerr << "[RobotController] 错误: 标定失败, 禁止进入控制模式" << std::endl;
        startup_phase_ = StartupPhase::FAULT;
        return false;
    }

    // ── 3. 构建 NN 策略链 ──
    //   StandingPolicy → (可选) ValidatingPolicy → (可选) ComparingPolicy
    {
        std::unique_ptr<NNPolicy> policy;
        std::unique_ptr<NNPolicy> baseline;

        // 3a. 基础策略
        if (!config_.onnx_model_path.empty()) {
            // ──── ONNX 模式 ────
            auto onnx = std::make_unique<ONNXPolicy>(
                config_.onnx_model_path,
                config_.default_standing_pose,
                config_.action_scale,
                config_.default_kp,
                config_.default_kd);
            if (!onnx->initialize()) {
                std::cerr << "[RobotController] ONNX 模型加载失败, 回退到 StandingPolicy"
                          << std::endl;
                policy = std::make_unique<StandingPolicy>(
                    config_.default_standing_pose, config_.default_kp, config_.default_kd);
            } else {
                policy = std::move(onnx);
            }
        } else {
            // ──── StandingPolicy 模式 (默认) ────
            policy = std::make_unique<StandingPolicy>(
                config_.default_standing_pose, config_.default_kp, config_.default_kd);
            std::cout << "[RobotController] 使用 StandingPolicy (无 ONNX 模型)" << std::endl;
        }

        // 3b. 对比模式: 同时运行 StandingPolicy 作基线
        if (config_.nn_flags.compare) {
            baseline = std::make_unique<StandingPolicy>(
                config_.default_standing_pose, config_.default_kp, config_.default_kd);
            policy = std::make_unique<ComparingPolicy>(
                std::move(policy), std::move(baseline));
            std::cout << "[RobotController] NN 对比模式已启用" << std::endl;
        }

        // 3c. 验证包装
        if (config_.nn_flags.validate) {
            auto fallback = std::make_unique<StandingPolicy>(
                config_.default_standing_pose, config_.default_kp, config_.default_kd);
            policy = std::make_unique<ValidatingPolicy>(
                std::move(policy), std::move(fallback),
                ValidatingPolicy::FallbackMode::PREV_FRAME);
            std::cout << "[RobotController] NN 验证模式已启用 (越界→维持上帧)" << std::endl;
        }

        nn_policy_ = std::move(policy);
    }

    // ── 4. 初始化推理日志 ──
    if (config_.nn_flags.log_io) {
        nn_logger_ = std::make_unique<NNInferenceLogger>(config_.nn_flags.log_filepath);
    }
    if (config_.nn_flags.dry_run) {
        std::cout << "[RobotController] *** 干运行模式: NN 计算但不发送电机指令 ***" << std::endl;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  start() — 按依赖顺序启动所有线程
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::start()
{
    if (running_.load()) {
        std::cerr << "[RobotController] 已经在运行" << std::endl;
        return false;
    }

    std::cout << "[RobotController] 启动控制线程..." << std::endl;
    running_.store(true);

    // ── 1. 启动 IMU 线程 (必须先启动，估计线程依赖它) ──
    std::cout << "  [1/4] 启动 IMU 读取线程 (" << config_.imu_hz << "Hz)..." << std::endl;
    imu_thread_ = std::thread(&RobotController::imuLoop, this);

    // 等待首帧数据
    if (!waitForIMUData(500)) {
        std::cerr << "  [警告] IMU 首帧数据超时，继续启动..." << std::endl;
    } else {
        std::cout << "  [OK] IMU 数据就绪" << std::endl;
    }

    // ── 2. 启动状态估计线程 ──
    std::cout << "  [2/4] 启动状态估计线程 (" << config_.estimation_hz << "Hz)..." << std::endl;
    est_thread_ = std::thread(&RobotController::estimationLoop, this);

    // ── 3. 启动 4 路电机总线 (每个 bus 一个线程, 硬件并行) ──
    std::cout << "  [3/4] 启动 " << config_.buses.size() << " 路电机总线 ("
              << config_.motor_hz << "Hz)..." << std::endl;
    motor_ctrl_->startAll(config_.motor_hz);

    // 等待电机就绪
    if (!waitForMotorReady(1000)) {
        std::cerr << "  [警告] 电机总线就绪超时，继续启动..." << std::endl;
    } else {
        std::cout << "  [OK] 电机总线就绪" << std::endl;
    }

    // ── 3.5. [新增] 过渡到站立姿态 ──
    if (calibration_completed_) {
        startup_phase_ = StartupPhase::TRANSITION_STAND;
        std::cout << "  [3.5/4] 过渡到站立姿态..." << std::endl;
        transitionToStandingInternal(2.0f);

        // ── 3.6. [新增] 站立稳定保持 ──
        startup_phase_ = StartupPhase::HOLD_STANDING;
        std::cout << "  [3.6/4] 站立稳定保持 (1s)..." << std::endl;
        sleep(1);
    }

    // ── 4. 启动 NN 推理线程 (最后启动，需要估计和电机都在运行) ──
    std::cout << "  [4/4] 启动 NN 推理线程 (" << config_.nn_hz << "Hz)..." << std::endl;
    nn_thread_ = std::thread(&RobotController::nnLoop, this);
    startup_phase_ = StartupPhase::NN_ACTIVE;

    std::cout << "[RobotController] ✓ 全部 7 个线程已启动 ("
              << "1 IMU + 1 EST + " << config_.buses.size()
              << " BUS + 1 NN)" << std::endl;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  stop() — 按依赖逆序优雅关闭
// ═══════════════════════════════════════════════════════════════════════════════

void RobotController::stop()
{
    if (!running_.load()) return;  // 已停止

    std::cout << "[RobotController] 正在关闭..." << std::endl;

    // ── 1. 通知所有线程退出 ──
    running_.store(false);

    // ── 2. 先 Join NN 线程 (停止发送新指令) ──
    std::cout << "  [1/4] 停止 NN 线程..." << std::endl;
    if (nn_thread_.joinable()) {
        nn_thread_.join();
    }

    // ── 3. 刹车所有电机 ──
    std::cout << "  [2/4] 刹车所有电机..." << std::endl;
    for (size_t b = 0; b < motor_ctrl_->busCount(); ++b) {
        auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
        for (unsigned short id : motor_ids) {
            motor_ctrl_->bus(b).brake(id);
        }
    }
    // 给总线线程一个周期执行刹车指令
    usleep(100000);  // 100ms

    // ── 4. 停止电机总线 (Join 4 路总线线程) ──
    std::cout << "  [3/4] 停止电机总线..." << std::endl;
    motor_ctrl_->stopAll();

    // ── 5. Join 状态估计线程 ──
    std::cout << "  [4/4] 停止状态估计线程..." << std::endl;
    if (est_thread_.joinable()) {
        est_thread_.join();
    }

    // ── 6. Join IMU 线程 ──
    std::cout << "  [5/5] 停止 IMU 线程..." << std::endl;
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }

    // ── 7. 关闭 IMU 设备 ──
    if (imu_) {
        imu_->close();
    }

    std::cout << "[RobotController] ✓ 已关闭" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  监控接口
// ═══════════════════════════════════════════════════════════════════════════════

const IMURawData* RobotController::getLatestIMUData()
{
    return imu_buffer_.tryAcquireRead();
}

const EstimatedState* RobotController::getLatestEstimatedState()
{
    return est_buffer_.tryAcquireRead();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  启动辅助 — 等待 IMU 数据
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::waitForIMUData(int timeout_ms)
{
    auto start = std::chrono::steady_clock::now();
    while (running_.load()) {
        const IMURawData* data = imu_buffer_.tryAcquireRead();
        if (data && data->timestamp_ns > 0) {
            return true;
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;
        usleep(5000);  // 5ms
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  启动辅助 — 等待电机总线就绪
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::waitForMotorReady(int timeout_ms)
{
    auto start = std::chrono::steady_clock::now();
    while (running_.load()) {
        bool ready = true;
        for (size_t b = 0; b < motor_ctrl_->busCount(); ++b) {
            auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
            for (unsigned short id : motor_ids) {
                MotorState s = motor_ctrl_->bus(b).getState(id);
                if (!s.correct) {
                    ready = false;
                    break;
                }
            }
            if (!ready) break;
        }
        if (ready) return true;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;
        usleep(10000);  // 10ms
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  runCalibrationSequence — 完整上电标定序列
// ═══════════════════════════════════════════════════════════════════════════════
//
//  标定阶段使用 MotorBus（同步模式），与 ParallelBus 共享 GPIO/串口。
//  MotorBus 实例在此函数内创建和销毁，ParallelBus 线程尚未启动 → 无冲突。
//
//  流程:
//    1. calibrateAllJoints() — 12 关节机械限位标定
//    2. validateCalibrationResults() — 实测行程 vs URDF 预期
//    3. 计算从标定位姿到站立姿态的起始位置

bool RobotController::runCalibrationSequence()
{
    std::cout << "\n╔══════════════════════════════════════════════╗" << std::endl;
    std::cout <<   "║  机械限位标定 — 12 关节                       ║" << std::endl;
    std::cout <<   "╚══════════════════════════════════════════════╝" << std::endl;

    // ── 阶段 1: 12 关节标定 ──
    startup_phase_ = StartupPhase::CALIBRATING;

    int ok_count = calibrateAllJoints(calib_results_);
    calibration_ok_count_ = ok_count;

    if (ok_count < 8)
    {
        std::cerr << "[Calib] 致命错误: 标定成功数不足 ("
                  << ok_count << "/12), 至少需要 8 个" << std::endl;
        return false;
    }

    // ── 阶段 2: 验证标定结果 ──
    startup_phase_ = StartupPhase::VERIFY_RESULTS;

    const auto* configs = getCalibrationConfigs();
    bool ranges_ok = validateCalibrationResults(calib_results_, configs);

    if (!ranges_ok && ok_count < 12)
    {
        std::cerr << "[Calib] 错误: 标定行程验证失败, 禁止继续" << std::endl;
        return false;
    }
    if (!ranges_ok)
    {
        std::cout << "[Calib] 警告: 部分关节行程偏差较大, 继续启动 ("
                  << ok_count << "/12 标定成功)" << std::endl;
    }

    // ── 阶段 3: 记录标定后的当前位置 ──
    //   hip/thigh: 停在 MechLimitEnd
    //   calf:      停在 MechLimitStart
    for (int i = 0; i < 12; i++)
    {
        if (calib_results_[i].success)
        {
            post_calib_position_[i] = configs[i].hit_upper_first
                                    ? calib_results_[i].mech_limit_end
                                    : calib_results_[i].mech_limit_start;
        }
        else
        {
            post_calib_position_[i] = 0.0f;  // 标定失败的关节将跳过过渡
        }
    }

    calibration_completed_ = true;

    std::cout << "\n[Calib] ✓ 标定序列完成 ("
              << ok_count << "/12 关节 OK)" << std::endl;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  transitionToStandingInternal — 平滑过渡到站立姿态
// ═══════════════════════════════════════════════════════════════════════════════
//
//  在 ParallelBus 线程已启动后调用。
//  q_cmd = q_start + smoothstep(α) * (q_stand - q_start)
//
//  过渡期间使用较低 KP/KD 确保平滑运动，到位后由 NN 策略接管使用正常增益。

void RobotController::transitionToStandingInternal(float transition_time_sec)
{
    const int STEPS = 200;
    const float dt = transition_time_sec / STEPS;
    const float kp = 0.3f;   // 过渡阶段使用较低刚度
    const float kd = 0.02f;

    // 计算站立目标（URDF 关节角 → 电机输出端位置）
    float stand_target[12] = {0};
    for (int i = 0; i < 12; i++)
    {
        if (calibration_completed_ && calib_results_[i].success)
        {
            stand_target[i] = computeMotorTargetFromURDF(i, config_.default_standing_pose[i]);
        }
        else
        {
            // 未标定的关节直接使用默认姿态（适用于跳过标定的场景）
            stand_target[i] = config_.default_standing_pose[i];
        }
    }

    std::cout << "[Transition] " << transition_time_sec << "s, "
              << STEPS << " steps (smoothstep)" << std::endl;

    for (int step = 0; step <= STEPS; step++)
    {
        float alpha = static_cast<float>(step) / STEPS;
        // smoothstep: f(t) = t²(3 - 2t), 缓入缓出
        float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);

        for (size_t b = 0; b < motor_ctrl_->busCount(); b++)
        {
            auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
            for (size_t m = 0; m < motor_ids.size(); m++)
            {
                int mid = motor_ids[m];
                float q_cmd = post_calib_position_[mid]
                            + smooth_alpha * (stand_target[mid] - post_calib_position_[mid]);
                motor_ctrl_->bus(b).setPosition(mid, q_cmd, kp, kd);
            }
        }

        usleep(static_cast<int>(dt * 1e6));
    }

    std::cout << "[Transition] ✓ 到达站立姿态" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  IMU 读取线程 — 200Hz
// ═══════════════════════════════════════════════════════════════════════════════
//
//  读取 JY901S IMU 数据并写入 imu_buffer_ 双缓冲。
//  使用 clock_nanosleep(TIMER_ABSTIME) 避免累积漂移 (与 ParallelBus 相同模式)。

void RobotController::imuLoop()
{
    const long period_ns = 1'000'000'000L / config_.imu_hz;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::cout << "[IMU Thread] 启动, 周期=" << (period_ns / 1000) << "us" << std::endl;

    while (running_.load(std::memory_order_relaxed)) {
        // ── 获取写槽引用 ──
        IMURawData& data = imu_buffer_.acquireWriteSlot();

        // ── 读取 IMU ──
        JY901S_Status st = imu_->readAll(data.angles, data.acc, data.gyro);
        if (st == JY901S_Status::OK) {
            // 记录时间戳
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            data.timestamp_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
                              + static_cast<uint64_t>(ts.tv_nsec);

            // 额外读取四元数 (用于姿态估计)
            imu_->readQuaternion(data.quat);

            // ── 提交: 原子翻转双缓冲 ──
            imu_buffer_.commitWrite();
        } else {
            // I2C 读取失败 — 不提交, 估计线程会检测到无新数据
            // (错误率应该很低, 无需在热路径上打印)
        }

        // ── 绝对时间睡眠 (无累积漂移) ──
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1'000'000'000L) {
            next.tv_nsec -= 1'000'000'000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "[IMU Thread] 已退出" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  状态估计线程 — 50Hz
// ═══════════════════════════════════════════════════════════════════════════════
//
//  从 imu_buffer_ 读取最新 IMU 数据, 从 ParallelBus 读取电机反馈,
//  运行状态估计算法 (当前为 PassthroughEstimator 占位),
//  结果写入 est_buffer_ 双缓冲。

void RobotController::estimationLoop()
{
    PassthroughEstimator estimator;  // PLACEHOLDER — 替换为 Kalman/Complementary
    const long period_ns = 1'000'000'000L / config_.estimation_hz;
    const int bus_count = static_cast<int>(config_.buses.size());

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::cout << "[EST Thread] 启动, 周期=" << (period_ns / 1000000) << "ms" << std::endl;

    while (running_.load(std::memory_order_relaxed)) {
        // ── 1. 读取最新 IMU 数据 (双缓冲指针, 不拷贝) ──
        const IMURawData* imu = imu_buffer_.tryAcquireRead();
        if (!imu) {
            // 尚无新数据，跳过本周期
            usleep(1000);
            continue;
        }

        // ── 2. 读取 12 电机状态 (值拷贝, ParallelBus 内部 mutex 保护) ──
        float joint_q[12], joint_dq[12], joint_tau[12];
        int   joint_err[12];
        int global_idx = 0;
        for (int b = 0; b < bus_count; ++b) {
            auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
            for (size_t m = 0; m < motor_ids.size(); ++m) {
                MotorState s = motor_ctrl_->bus(b).getState(motor_ids[m]);
                joint_q[global_idx]   = s.q;
                joint_dq[global_idx]  = s.dq;
                joint_tau[global_idx] = s.tau;
                joint_err[global_idx] = s.merror;
                ++global_idx;
            }
        }

        // ── 3. 运行状态估计 ──
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
                         + static_cast<uint64_t>(ts.tv_nsec);

        EstimatedState& est = est_buffer_.acquireWriteSlot();
        est = estimator.update(*imu, joint_q, joint_dq, joint_tau, joint_err, now_ns);
        est_buffer_.commitWrite();

        // ── 绝对时间睡眠 ──
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1'000'000'000L) {
            next.tv_nsec -= 1'000'000'000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "[EST Thread] 已退出" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  NN 推理线程 — 50Hz
// ═══════════════════════════════════════════════════════════════════════════════
//
//  从 est_buffer_ 读取估计状态, 运行策略推理 (当前为 StandingPolicy 占位),
//  将电机指令写入 ParallelBus (内部 mutex 保护)。

void RobotController::nnLoop()
{
    const long period_ns = 1'000'000'000L / config_.nn_hz;
    const int bus_count = static_cast<int>(config_.buses.size());

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::cout << "[NN Thread] 启动, 策略=" << nn_policy_->name()
              << ", 周期=" << (period_ns / 1000000) << "ms"
              << (config_.nn_flags.dry_run ? " [DRY RUN]" : "")
              << (config_.nn_flags.log_io ? " [LOG]" : "")
              << (config_.nn_flags.compare ? " [COMPARE]" : "")
              << (config_.nn_flags.validate ? " [VALIDATE]" : "")
              << std::endl;

    while (running_.load(std::memory_order_relaxed)) {
        // ── 1. 读取估计状态 (双缓冲指针, 不拷贝) ──
        const EstimatedState* est = est_buffer_.tryAcquireRead();
        if (!est) {
            usleep(1000);
            continue;
        }

        // ── 2. 运行推理 (计时) ──
        auto t0 = std::chrono::high_resolution_clock::now();

        NNCommandSet cmds;
        bool inference_ok = nn_policy_->infer(*est, cmds);

        auto t1 = std::chrono::high_resolution_clock::now();
        int latency_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

        // ── 3. 日志记录 (可选) ──
        if (nn_logger_ && nn_logger_->isEnabled()) {
            nn_logger_->log(*est, cmds, inference_ok, latency_us);
        }

        // ── 4. 写入电机指令 (除非干运行模式) ──
        if (!config_.nn_flags.dry_run) {
            if (inference_ok && cmds.valid) {
                for (int b = 0; b < bus_count; ++b) {
                    auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
                    for (size_t m = 0; m < motor_ids.size(); ++m) {
                        // 使用 motor ID 作为模型输出索引
                        // 模型输出顺序: [0]=motor0(hip), [1]=motor1(hip), ...
                        //               [4]=motor4(thigh), ..., [8]=motor8(lower_leg)...
                        int mid = motor_ids[m];
                        motor_ctrl_->bus(b).setPosition(
                            mid,
                            cmds.joint_position_target[mid],
                            cmds.kp[mid],
                            cmds.kd[mid]);
                    }
                }
            }
        }

        // ── 绝对时间睡眠 ──
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1'000'000'000L) {
            next.tv_nsec -= 1'000'000'000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    std::cout << "[NN Thread] 已退出" << std::endl;
}
