#include "robot_controller.h"
#include "state_estimator.h"
#include "nn_policy.h"
#include "jy901s.h"
#include "parallel_bus.h"
#include "../motor_lib/motor_controller.h"
#include "emergency_stop.h"

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
    safeShutdown();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  默认配置 — 4 腿 12 电机
// ═══════════════════════════════════════════════════════════════════════════════

RobotControlConfig RobotController::getDefaultConfig()
{
    RobotControlConfig c;
    c.imu_device    = "/dev/i2c-1";
    c.imu_hz        = 200;
    c.estimation_hz = 50;
    c.motor_hz      = 500;
    c.nn_hz         = 50;
    c.default_kp    = 0.625f;
    c.default_kd    = 0.0125f;

    // 4 路 RS-485 总线, 每路 3 个电机 (hip / thigh / lower_leg)
    c.buses = {
        {1, 7,  "/dev/ttyS6", {0, 4, 8}},      // Leg1: global GPIO39
        {1, 31, "/dev/ttyS4", {1, 5, 9}},      // Leg2: global GPIO63
        {1, 3,  "/dev/ttyS7", {2, 6, 10}},     // Leg3: global GPIO35
        {4, 5,  "/dev/ttyS0", {3, 7, 11}},     // Leg4: global GPIO133
    };

    return c;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  initialize() — 初始化硬件
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::initialize()
{
    std::cout << "[RobotController] 初始化硬件..." << std::endl;

    // 关节状态和 NN 数组统一以 motor ID 为下标，启动前拒绝非法、重复或漏配 ID。
    bool motor_id_seen[12] = {false};
    int configured_motor_count = 0;
    for (const auto& bus_cfg : config_.buses) {
        for (unsigned short id : bus_cfg.motor_ids) {
            if (id >= 12 || motor_id_seen[id]) {
                std::cerr << "[RobotController] 错误: 非法或重复的电机 ID="
                          << id << std::endl;
                return false;
            }
            motor_id_seen[id] = true;
            ++configured_motor_count;
        }
    }
    if (configured_motor_count != 12) {
        std::cerr << "[RobotController] 错误: 必须且只能配置 motor ID 0..11，当前数量="
                  << configured_motor_count << std::endl;
        return false;
    }

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

    // ── 2. 上电标定序列 ──
    // 标定使用同步 MotorBus；并行总线必须在标定结束后创建，避免重复占用串口/GPIO。
    startup_phase_ = StartupPhase::INIT_COMM;
    if (!runCalibrationSequence()) {
        std::cerr << "[RobotController] 错误: 标定失败, 禁止进入控制模式" << std::endl;
        startup_phase_ = StartupPhase::FAULT;
        return false;
    }

    if (isEmergencyStopRequested()) return false;

    // ── 3. 创建 4 路并行总线 ──
    motor_ctrl_ = std::make_unique<MultiBusController>();
    for (const auto& bus_cfg : config_.buses) {
        ParallelBus& bus = motor_ctrl_->addBus(
            bus_cfg.gpio_chip, bus_cfg.gpio_line, bus_cfg.serial_port);
        for (unsigned short id : bus_cfg.motor_ids) {
            if (!bus.addMotor(id)) return false;
        }
        std::cout << "[RobotController] 总线 " << bus_cfg.serial_port
                  << " (GPIO" << bus_cfg.gpio_chip << ":" << bus_cfg.gpio_line << ") "
                  << bus_cfg.motor_ids.size() << " 电机" << std::endl;
    }

    // ── 4. 构建 NN 策略链 ──
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
    if (isEmergencyStopRequested()) return false;
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

    std::cout << "  [2.5/4] 保持机器人静止，校准陀螺仪残余零偏..." << std::endl;
    if (!waitForEstimatorReady(3000)) {
        std::cerr << "  [错误] 陀螺仪静止校准失败；请保持机器人静止后重试" << std::endl;
        requestEmergencyStop();
        return false;
    }
    std::cout << "  [OK] 陀螺仪零偏校准完成" << std::endl;

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
        if (isEmergencyStopRequested()) return false;

        // ── 3.6. [新增] 站立稳定保持 ──
        startup_phase_ = StartupPhase::HOLD_STANDING;
        std::cout << "  [3.6/4] 站立稳定保持 (1s)..." << std::endl;
        for (int i = 0; i < 20 && !isEmergencyStopRequested(); ++i) {
            usleep(50000);
        }
    }

    // ── 4. 启动 NN 推理线程 (最后启动，需要估计和电机都在运行) ──
    if (isEmergencyStopRequested()) return false;
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
    safeShutdown();
}

bool RobotController::isRunning() const
{
    return running_.load(std::memory_order_acquire)
        && !isEmergencyStopRequested();
}

void RobotController::requestEmergencyStop() noexcept
{
    ::requestEmergencyStop();
    running_.store(false, std::memory_order_release);
}

void RobotController::safeShutdown()
{
    if (shutdown_started_.exchange(true, std::memory_order_acq_rel)) return;

    std::cout << "[RobotController] 正在安全关闭..." << std::endl;
    requestEmergencyStop();

    // 先结束所有可能继续产生位置/速度/力矩指令的线程。
    if (nn_thread_.joinable()) nn_thread_.join();
    if (est_thread_.joinable()) est_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();

    if (motor_ctrl_ && motor_ctrl_->busCount() > 0) {
        const bool all_buses_ready = motor_ctrl_->busCount() == config_.buses.size();
        // 锁存阻尼后普通 setPosition/setVelocity/setTorque 将全部失效。
        motor_ctrl_->enterEmergencyDampingAll(0.02f);
        motor_ctrl_->startAll(100);
        usleep(200000);  // 保持周期发送 200 ms，再回收总线线程。
        motor_ctrl_->stopAll();
        if (!all_buses_ready) {
            // 初始化只完成部分总线时，释放占用后再覆盖全部 12 个电机。
            motor_ctrl_.reset();
            enterDampingModeForAllMotors(0.02f, 200);
        }
    } else {
        // 初始化/标定提前失败时，并行总线尚不存在，使用同步安全路径。
        enterDampingModeForAllMotors(0.02f, 200);
    }

    if (imu_) imu_->close();
    std::cout << "[RobotController] ✓ 12 电机阻尼指令已发送，线程已回收" << std::endl;
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
    while (running_.load() && !isEmergencyStopRequested()) {
        const IMURawData* data = imu_buffer_.tryAcquireRead();
        if (data && data->valid && data->timestamp_ns > 0) {
            return true;
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;
        usleep(5000);  // 5ms
    }
    return false;
}

bool RobotController::waitForEstimatorReady(int timeout_ms)
{
    auto start = std::chrono::steady_clock::now();
    while (running_.load() && !isEmergencyStopRequested()) {
        const EstimatedState* state = est_buffer_.tryAcquireRead();
        // 此时电机总线尚未启动，只等待 IMU 姿态和陀螺仪校准完成。
        if (state && state->orientation_valid
                  && state->gyro_valid && state->gyro_calibrated) {
            std::cout << "  [Gyro] bias=(" << state->gyro_bias[0] << ", "
                      << state->gyro_bias[1] << ", " << state->gyro_bias[2]
                      << ") rad/s" << std::endl;
            return true;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;
        usleep(5000);
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  启动辅助 — 等待电机总线就绪
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::waitForMotorReady(int timeout_ms)
{
    auto start = std::chrono::steady_clock::now();
    while (running_.load() && !isEmergencyStopRequested()) {
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

    if (isEmergencyStopRequested()) {
        std::cerr << "[Calib] 急停请求，终止标定序列" << std::endl;
        return false;
    }

    if (ok_count < 8)  // 至少 8/12 关节成功才允许继续
    {
        std::cerr << "[Calib] 致命错误: 标定成功数不足 ("
                  << ok_count << "/12)" << std::endl;
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
            // 校准后电机停在撞到的URDF限位处, 用转换函数得到准确的电机位置
            float q_urdf_hit = configs[i].hit_upper_first
                             ? configs[i].urdf_upper : configs[i].urdf_lower;
            post_calib_position_[i] = urdfToMotorPosition(configs[i].motor_id, q_urdf_hit);
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
    const float KP_START = 0.02f;   // 起步极低刚度, 避免初始大误差造成冲击电流
    const float KP_END   = 0.3f;    // 终点刚度
    const float KD       = 0.01f;   // 阻尼
    const float MAX_STEP_DELTA = 0.03f;  // 每步最大位置增量 (rad, ~1.7°)

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
            stand_target[i] = config_.default_standing_pose[i];
        }
    }

    // 记录上一帧已输出的位置, 用于增量限制
    float last_cmd[12];
    for (int i = 0; i < 12; i++)
        last_cmd[i] = post_calib_position_[i];

    std::cout << "[Transition] " << transition_time_sec << "s, "
              << STEPS << " steps (KP ramp " << KP_START
              << "→" << KP_END << ", max Δ=" << MAX_STEP_DELTA << " rad/step)"
              << std::endl;

    for (int step = 0; step <= STEPS && !isEmergencyStopRequested(); step++)
    {
        float alpha = static_cast<float>(step) / STEPS;
        float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);

        // KP 从极低逐步拉高
        float kp = KP_START + smooth_alpha * (KP_END - KP_START);

        for (size_t b = 0; b < motor_ctrl_->busCount(); b++)
        {
            auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
            for (size_t m = 0; m < motor_ids.size(); m++)
            {
                int mid = motor_ids[m];
                // 目标位置 (smoothstep 插值)
                float q_desired = post_calib_position_[mid]
                                + smooth_alpha * (stand_target[mid] - post_calib_position_[mid]);

                // 增量限制: 每步最多动 MAX_STEP_DELTA rad
                float delta = q_desired - last_cmd[mid];
                if (delta >  MAX_STEP_DELTA) delta =  MAX_STEP_DELTA;
                if (delta < -MAX_STEP_DELTA) delta = -MAX_STEP_DELTA;
                float q_cmd = last_cmd[mid] + delta;
                last_cmd[mid] = q_cmd;

                motor_ctrl_->bus(b).setPosition(mid, q_cmd, kp, KD);
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

    while (running_.load(std::memory_order_relaxed) && !isEmergencyStopRequested()) {
        // ── 获取写槽引用 ──
        IMURawData& data = imu_buffer_.acquireWriteSlot();

        // ── 读取 IMU ──
        JY901S_Status data_st = imu_->readAll(data.angles, data.acc, data.gyro);
        JY901S_Status quat_st = JY901S_Status::ERROR_I2C_READ;
        if (data_st == JY901S_Status::OK)
            quat_st = imu_->readQuaternion(data.quat);

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        data.timestamp_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
                          + static_cast<uint64_t>(ts.tv_nsec);
        data.valid = data_st == JY901S_Status::OK && quat_st == JY901S_Status::OK;

        // 成功与失败都提交，使估计线程能及时发现 IMU 断流，而不是沿用旧姿态。
        imu_buffer_.commitWrite();

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

    while (running_.load(std::memory_order_relaxed) && !isEmergencyStopRequested()) {
        // ── 1. 读取最新 IMU 数据 (双缓冲指针, 不拷贝) ──
        const IMURawData* imu = imu_buffer_.tryAcquireRead();
        if (!imu) {
            // 尚无新数据，跳过本周期
            usleep(1000);
            continue;
        }

        // ── 2. 读取 12 电机状态 (值拷贝, ParallelBus 内部 mutex 保护) ──
        // 数组下标严格等于 motor ID，与 EstimatedState 和 NN 输入输出一致。
        float joint_q[12] = {0};
        float joint_dq[12] = {0};
        float joint_tau[12] = {0};
        int   joint_err[12] = {0};
        bool  joint_valid[12] = {false};
        uint64_t joint_age_ns[12];
        uint32_t joint_failure_count[12] = {0};
        for (uint64_t& age : joint_age_ns) age = UINT64_MAX;

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_ns = static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
                         + static_cast<uint64_t>(ts.tv_nsec);

        for (int b = 0; b < bus_count; ++b) {
            auto motor_ids = motor_ctrl_->bus(b).getMotorIds();
            for (size_t m = 0; m < motor_ids.size(); ++m) {
                MotorState s = motor_ctrl_->bus(b).getState(motor_ids[m]);
                // 电机坐标系 → URDF 坐标系
                int mid = motor_ids[m];
                joint_q[mid]   = motorToUrdfPosition(mid, s.q);
                joint_dq[mid]  = motorToUrdfVelocity(mid, s.dq);
                joint_tau[mid] = motorToUrdfTorque(mid, s.tau);
                joint_err[mid] = s.merror;
                joint_valid[mid] = s.correct && s.merror == 0;
                joint_failure_count[mid] = s.consecutive_failures;
                if (s.feedback_timestamp_ns > 0 && s.feedback_timestamp_ns <= now_ns)
                    joint_age_ns[mid] = now_ns - s.feedback_timestamp_ns;
            }
        }

        // ── 3. 运行状态估计 ──
        EstimatedState& est = est_buffer_.acquireWriteSlot();
        est = estimator.update(
            *imu, joint_q, joint_dq, joint_tau, joint_err, joint_valid,
            joint_age_ns, joint_failure_count, now_ns);
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
    int consecutive_invalid_states = 0;
    auto last_estimate_received = std::chrono::steady_clock::now();

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::cout << "[NN Thread] 启动, 策略=" << nn_policy_->name()
              << ", 周期=" << (period_ns / 1000000) << "ms"
              << (config_.nn_flags.dry_run ? " [DRY RUN]" : "")
              << (config_.nn_flags.log_io ? " [LOG]" : "")
              << (config_.nn_flags.compare ? " [COMPARE]" : "")
              << (config_.nn_flags.validate ? " [VALIDATE]" : "")
              << std::endl;

    while (running_.load(std::memory_order_relaxed) && !isEmergencyStopRequested()) {
        // ── 1. 读取估计状态 (双缓冲指针, 不拷贝) ──
        const EstimatedState* est = est_buffer_.tryAcquireRead();
        if (!est) {
            const auto silence_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_estimate_received).count();
            if (silence_ms > 100) {
                std::cerr << "[NN Thread] 状态估计数据流超时 " << silence_ms
                          << "ms, 请求安全停机" << std::endl;
                requestEmergencyStop();
            }
            usleep(1000);
            continue;
        }
        last_estimate_received = std::chrono::steady_clock::now();

        if (!est->valid || !est->orientation_valid || !est->gyro_valid
                || !est->gyro_calibrated || !est->projected_gravity_valid
                || !est->joint_feedback_valid) {
            ++consecutive_invalid_states;
            // 无效帧绝不执行推理或下发；连续异常才停机，容忍单次 I2C 瞬态抖动。
            if (consecutive_invalid_states >= 3) {
                std::cerr << "[NN Thread] 状态估计连续无效, status=" << est->status_code
                          << ", 请求安全停机" << std::endl;
                requestEmergencyStop();
            }
            continue;
        }
        consecutive_invalid_states = 0;

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

        // dry-run 也提交“最终被验证接受”的动作，用于离线时序一致性；
        // 验证失败且没有安全回退命令时不更新动作历史。
        if (inference_ok && cmds.valid)
            nn_policy_->commitAcceptedCommand(cmds);

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
                        // URDF 目标 → 电机坐标系
                        float q_motor = urdfToMotorPosition(mid, cmds.joint_position_target[mid]);
                        motor_ctrl_->bus(b).setPosition(
                            mid, q_motor,
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
