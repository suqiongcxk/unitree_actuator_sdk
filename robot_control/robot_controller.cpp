#include "robot_controller.h"
#include "state_estimator.h"
#include "linear_kf_state_estimator_adapter.h"
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
#include <algorithm>

namespace {
void printBusTimingSnapshot(const MultiBusController& controller,
                            const char* prefix,
                            std::ostream& out)
{
    for (size_t index = 0; index < controller.busCount(); ++index) {
        const ParallelBus& bus = controller.bus(index);
        const BusTimingStats timing = bus.getTimingStats();
        out << prefix
            << " bus=" << static_cast<char>('A' + index)
            << " port=" << bus.getPort()
            << " actual_hz=" << std::fixed << std::setprecision(1)
            << bus.getActualHz()
            << " loops=" << timing.loop_count
            << " max_gap=" << std::setprecision(2)
            << static_cast<double>(timing.max_loop_gap_ns) / 1.0e6 << "ms"
            << " max_cycle="
            << static_cast<double>(timing.max_cycle_duration_ns) / 1.0e6 << "ms"
            << " gaps>{2,10,50,100}ms={"
            << timing.gap_over_2ms << ","
            << timing.gap_over_10ms << ","
            << timing.gap_over_50ms << ","
            << timing.gap_over_100ms << "}"
            << std::defaultfloat << std::endl;
    }
}
}

// ═══════════════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════════

RobotController::RobotController(const RobotControlConfig& config)
    : config_(config)
    , velocity_command_(config.velocity_command)
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

#ifndef LINEAR_KF_AVAILABLE
    // 必须在 IMU/电机初始化和机械标定前拒绝，不得静默回退。
    if (config_.estimator_backend == StateEstimatorBackend::LINEAR_KF) {
        std::cerr << "[RobotController] 错误: 本次构建未启用 Eigen/Linear KF" << std::endl;
        return false;
    }
#endif

    if (config_.simulated_motor_timeout_id >= 0
            && (!config_.monitor_only || config_.simulated_motor_timeout_id > 11)) {
        std::cerr << "[RobotController] 错误: 电机超时故障注入仅允许用于 "
                     "monitor-only，ID 必须为 0..11" << std::endl;
        return false;
    }
    if (config_.simulated_imu_stream_loss && !config_.monitor_only) {
        std::cerr << "[RobotController] 错误: IMU 数据流故障注入仅允许用于 "
                     "monitor-only" << std::endl;
        return false;
    }
    if (config_.simulated_leg_slip >= 0
            && (!config_.step7_diagnostics || config_.simulated_leg_slip > 3)) {
        std::cerr << "[RobotController] 错误: 单腿打滑注入仅允许用于 "
                     "step7-diagnostics，腿下标必须为 0..3" << std::endl;
        return false;
    }

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
    if (config_.monitor_only) {
        // 只监控模式专用于时间戳/看门狗/急停实机验收，
        // 不允许进入机械限位标定和站立过渡。
        std::cout << "[RobotController] *** MONITOR ONLY: 跳过机械标定和站立 ***"
                  << std::endl;
    } else if (!runCalibrationSequence()) {
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
            if (config_.monitor_only) {
                // 在总线线程启动前覆盖默认命令，保证首个周期即为阻尼。
                bus.setDamping(id, 0.02f);
            } else if (calibration_completed_ && calibrated_by_motor_id_[id]) {
                // 同步标定总线切换到并行总线时，首帧即低刚度保持当前安全姿态，
                // 避免先发送 BRAKE 再突然切换到位置控制。
                bus.setPosition(id, post_calib_position_[id], 0.02f, 0.01f);
            }
        }
        std::cout << "[RobotController] 总线 " << bus_cfg.serial_port
                  << " (GPIO" << bus_cfg.gpio_chip << ":" << bus_cfg.gpio_line << ") "
                  << bus_cfg.motor_ids.size() << " 电机" << std::endl;
    }

    // ── 4. 构建 NN 策略链 ──
    //   ONNX → (可选 Compare) → SmoothTakeover → (可选 Validate)
    // 平滑层位于验证层内侧，保证每个过渡候选仍经过完整安全检查。
    {
        std::unique_ptr<NNPolicy> policy;
        std::unique_ptr<NNPolicy> baseline;
        bool onnx_active = false;

        NNCommandSet takeover_initial;
        for (int i = 0; i < 12; ++i) {
            takeover_initial.joint_position_target[i] = config_.default_standing_pose[i];
            takeover_initial.kp[i] = config_.default_kp;
            takeover_initial.kd[i] = config_.default_kd;
        }
        takeover_initial.valid = true;

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
                onnx_active = true;
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

        // 开环站立完成后，先从已锁存的默认站姿缓慢接管。monitor-only
        // 不经过站立流程，保持原有纯观测/日志语义，因此不启用本层。
        if (onnx_active && !config_.monitor_only) {
            const int takeover_frames = std::max(
                1,
                static_cast<int>(std::lround(
                    config_.nn_takeover_duration_sec * config_.nn_hz)));
            policy = std::make_unique<SmoothTakeoverPolicy>(
                std::move(policy), takeover_initial, takeover_frames);
            std::cout << "[RobotController] NN 平滑接管已启用: "
                      << config_.nn_takeover_duration_sec << "s / "
                      << takeover_frames << " 帧" << std::endl;
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

        // 验证器首帧必须从当前站立目标开始比较，避免把 Actor 原始目标
        // 直接当作历史指令；该提交不会推进 SmoothTakeover 的进度。
        if (onnx_active && !config_.monitor_only) {
            nn_policy_->commitAcceptedCommand(takeover_initial);
        }
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
        if (!transitionToStandingInternal(3.0f)) return false;

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

bool RobotController::setVelocityCommand(float vx, float vy, float yaw_rate)
{
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw_rate))
        return false;
    std::lock_guard<std::mutex> lock(velocity_command_mutex_);
    velocity_command_ = {{vx, vy, yaw_rate}};
    return true;
}

void RobotController::requestEmergencyStop() noexcept
{
    ::requestEmergencyStop();
    running_.store(false, std::memory_order_release);
}

void RobotController::safeShutdown()
{
    if (shutdown_started_.exchange(true, std::memory_order_acq_rel)) return;

    // printStatus() 会把 cout 留在两位小数模式；此处强制三位显示，
    // 避免 0.068 被显示成 0.07 而误判实际安全参数。
    std::cout << "[RobotController] 正在安全关闭，阻尼 Kd="
              << std::fixed << std::setprecision(3)
              << config_.emergency_damping_kd << std::defaultfloat
              << "..." << std::endl;
    requestEmergencyStop();

    // 先结束所有可能继续产生位置/速度/力矩指令的线程。
    if (nn_thread_.joinable()) nn_thread_.join();
    if (est_thread_.joinable()) est_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();

    if (motor_ctrl_ && motor_ctrl_->busCount() > 0) {
        const bool all_buses_ready = motor_ctrl_->busCount() == config_.buses.size();
        // 锁存阻尼后普通 setPosition/setVelocity/setTorque 将全部失效。
        motor_ctrl_->enterEmergencyDampingAll(config_.emergency_damping_kd);
        motor_ctrl_->startAll(100);
        usleep(200000);  // 保持周期发送 200 ms，再回收总线线程。
        motor_ctrl_->stopAll();
        // 停止后快照不再变化，可用于判断通信阻塞还是线程调度停顿。
        printBusTimingSnapshot(*motor_ctrl_, "[BusTiming@shutdown]", std::cout);
        if (!all_buses_ready) {
            // 初始化只完成部分总线时，释放占用后再覆盖全部 12 个电机。
            motor_ctrl_.reset();
            enterDampingModeForAllMotors(config_.emergency_damping_kd, 200);
        }
    } else {
        // 初始化/标定提前失败时，并行总线尚不存在，使用同步安全路径。
        enterDampingModeForAllMotors(config_.emergency_damping_kd, 200);
    }

    if (imu_) imu_->close();
    std::cout << "[RobotController] ✓ 12 电机阻尼指令已发送，线程已回收" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  监控接口
// ═══════════════════════════════════════════════════════════════════════════════

bool RobotController::getLatestIMUData(IMURawData& out) const
{
    std::lock_guard<std::mutex> lock(monitor_imu_mutex_);
    if (!monitor_imu_available_) return false;
    out = monitor_imu_snapshot_;
    return true;
}

bool RobotController::getLatestEstimatedState(EstimatedState& out) const
{
    std::lock_guard<std::mutex> lock(monitor_est_mutex_);
    if (!monitor_est_available_) return false;
    out = monitor_est_snapshot_;
    return true;
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
                // 就绪只要求本次启动后每个电机至少成功返回过一帧；不能要求
                // 12 个电机“最近一次事务”在同一采样瞬间全部成功。
                if (s.feedback_timestamp_ns == 0 || s.merror != 0) {
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

    // 站立和 NN 控制需要完整的 12 关节状态；任一关节未标定
    // 都不允许用默认值冒险继续。
    if (ok_count != 12)
    {
        std::cerr << "[Calib] 致命错误: 标定成功数不足 ("
                  << ok_count << "/12)" << std::endl;
        return false;
    }

    // ── 阶段 2: 验证标定结果 ──
    startup_phase_ = StartupPhase::VERIFY_RESULTS;

    const auto* configs = getCalibrationConfigs();
    bool ranges_ok = validateCalibrationResults(calib_results_, configs);

    if (!ranges_ok)
    {
        std::cerr << "[Calib] 错误: 标定行程验证失败, 禁止继续" << std::endl;
        return false;
    }

    // ── 阶段 3: 记录标定后的当前位置 ──
    // configs/results 按腿排列，post_calib_position_ 按 motor ID 排列，
    // 因此每次写入必须显式使用 configs[i].motor_id。
    std::memset(calibrated_by_motor_id_, 0, sizeof(calibrated_by_motor_id_));
    for (int i = 0; i < 12; i++)
    {
        const int mid = configs[i].motor_id;
        if (mid < 0 || mid >= 12) {
            std::cerr << "[Calib] 错误: 非法 motor ID=" << mid << std::endl;
            return false;
        }
        if (calib_results_[i].success)
        {
            // calibrateAllJoints() 已将 Hip 移到默认站立角，并将后腿
            // Thigh 从上限回退 80°；这里记录真实的流程结束姿态。
            float q_urdf_hit = configs[i].hit_upper_first
                             ? configs[i].urdf_upper : configs[i].urdf_lower;
            if (mid >= 0 && mid <= 3) {
                q_urdf_hit = config_.default_standing_pose[mid];
            } else if ((configs[i].leg_index == 2 || configs[i].leg_index == 3)
                       && mid >= 4 && mid <= 7 && configs[i].hit_upper_first) {
                q_urdf_hit -= 1.3963f;  // calibrateAllJoints 中的后腿 80° 回退
            }
            post_calib_position_[mid] = urdfToMotorPosition(mid, q_urdf_hit);
            calibrated_by_motor_id_[mid] = true;
        }
        else
        {
            // 理论上已被 12/12 检查拦截；保留显式防御。
            calibrated_by_motor_id_[mid] = false;
            return false;
        }
    }

    calibration_completed_ = true;

    // 与已经实机验证的 example_usage 启动流程保持一致：正式站立前先把
    // 四个大腿移到安全中间姿态，降低四腿同时展开时的冲击和承重突变。
    if (!prepositionThighsForStanding()) {
        std::cerr << "[Calib] 大腿安全预定位失败, 禁止进入站立" << std::endl;
        calibration_completed_ = false;
        return false;
    }

    std::cout << "\n[Calib] ✓ 标定序列完成 ("
              << ok_count << "/12 关节 OK)" << std::endl;
    return true;
}

bool RobotController::prepositionThighsForStanding()
{
    struct ThighPrePosition { int motor_id; float urdf_delta; };
    const ThighPrePosition prepositions[4] = {
        {4, -1.1343f}, {5, -1.1343f},  // 前腿各减 65°
        {6, -0.6980f}, {7, -0.6980f},  // 后腿各减 40°
    };
    const auto* configs = getCalibrationConfigs();

    std::cout << "\n[PrePos] 大腿安全预定位（与 example_usage 一致）..."
              << std::endl;
    for (const auto& pre : prepositions) {
        if (isEmergencyStopRequested()) return false;

        int config_index = -1;
        for (int i = 0; i < 12; ++i) {
            if (configs[i].motor_id == pre.motor_id) {
                config_index = i;
                break;
            }
        }
        if (config_index < 0 || !calib_results_[config_index].success) return false;

        const auto& joint = configs[config_index];
        const BusConfig* bus_config = nullptr;
        for (const auto& candidate : config_.buses) {
            if (std::find(candidate.motor_ids.begin(), candidate.motor_ids.end(),
                          static_cast<unsigned short>(pre.motor_id))
                    != candidate.motor_ids.end()) {
                bus_config = &candidate;
                break;
            }
        }
        if (!bus_config) return false;

        float current_urdf = joint.hit_upper_first
                           ? joint.urdf_upper : joint.urdf_lower;
        if ((joint.leg_index == 2 || joint.leg_index == 3)
                && joint.hit_upper_first) {
            current_urdf -= 1.3963f;  // calibrateAllJoints 已将后腿退回 80°。
        }
        float target_urdf = current_urdf + pre.urdf_delta;
        target_urdf = std::max(joint.urdf_lower + 0.05f,
                      std::min(joint.urdf_upper - 0.05f, target_urdf));
        const float current_motor = urdfToMotorPosition(pre.motor_id, current_urdf);
        const float target_motor = urdfToMotorPosition(pre.motor_id, target_urdf);

        std::cout << "  Thigh " << pre.motor_id << ": URDF "
                  << current_urdf << " → " << target_urdf << std::endl;
        const int steps = 60;
        const float duration_sec = 0.8f;
        const int global_gpio = bus_config->gpio_chip * 32 + bus_config->gpio_line;
        MotorBus bus(global_gpio, bus_config->serial_port);
        if (!bus.addMotor(static_cast<unsigned short>(pre.motor_id))) return false;
        for (int step = 0; step <= steps && !isEmergencyStopRequested(); ++step) {
            const float alpha = static_cast<float>(step) / steps;
            const float smooth = alpha * alpha * (3.0f - 2.0f * alpha);
            const float command = current_motor
                                + smooth * (target_motor - current_motor);
            bus.setPosition(pre.motor_id, command, 0.15f, 0.01f);
            bus.sendRecv();
            usleep(static_cast<int>(duration_sec / steps * 1.0e6f));
        }
        if (isEmergencyStopRequested()) return false;
        post_calib_position_[pre.motor_id] = target_motor;
    }
    std::cout << "[PrePos] 大腿安全预定位完成" << std::endl;
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

bool RobotController::transitionToStandingInternal(float transition_time_sec)
{
    const int STEPS = 200;
    const float dt = transition_time_sec / STEPS;
    const float KP_START = 0.02f;   // 起步极低刚度, 避免初始大误差造成冲击电流
    const float KP_END   = 0.6f;    // 与已验证的 example_usage 保持一致
    const float KD       = 0.0125f;
    const float MAX_STEP_DELTA = 0.02f;

    // 计算站立目标（URDF 关节角 → 电机输出端位置）
    float stand_target[12] = {0};
    for (int i = 0; i < 12; i++)
    {
        if (calibration_completed_ && calibrated_by_motor_id_[i])
        {
            stand_target[i] = computeMotorTargetFromURDF(i, config_.default_standing_pose[i]);
        }
        else
        {
            stand_target[i] = config_.default_standing_pose[i];
        }
    }

    // 并行总线已经获得反馈：必须从真实位置开始，而不是依赖标定后的推算值。
    float stand_start[12] = {0};
    float last_cmd[12];
    for (size_t b = 0; b < motor_ctrl_->busCount(); ++b) {
        for (unsigned short mid : motor_ctrl_->bus(b).getMotorIds()) {
            const MotorState state = motor_ctrl_->bus(b).getState(mid);
            if (state.feedback_timestamp_ns == 0 || state.merror != 0
                    || !std::isfinite(state.q)) {
                std::cerr << "[Transition] Motor " << mid
                          << " 无有效起始反馈, 禁止站立" << std::endl;
                requestEmergencyStop();
                return false;
            }
            stand_start[mid] = state.q;
            last_cmd[mid] = state.q;
        }
    }

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
                float q_desired = stand_start[mid]
                                + smooth_alpha * (stand_target[mid] - stand_start[mid]);

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

    if (isEmergencyStopRequested()) return false;
    std::cout << "[Transition] ✓ 到达站立姿态并保持 KP=" << KP_END << std::endl;
    return true;
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
    const uint64_t imu_loop_start_ns =
        static_cast<uint64_t>(next.tv_sec) * 1'000'000'000ULL
      + static_cast<uint64_t>(next.tv_nsec);
    const uint64_t simulated_imu_loss_start_ns = imu_loop_start_ns
      + static_cast<uint64_t>(config_.simulated_fault_delay_ms) * 1'000'000ULL;
    bool simulated_imu_loss_announced = false;

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

        {
            std::lock_guard<std::mutex> lock(monitor_imu_mutex_);
            monitor_imu_snapshot_ = data;
            monitor_imu_available_ = true;
        }

        // 故障注入只停止向控制链路提交，真实 I2C 读取和诊断快照仍继续。
        // NN 独立状态流看门狗应在 100 ms 后请求安全停机。
        if (config_.simulated_imu_stream_loss
                && data.timestamp_ns >= simulated_imu_loss_start_ns) {
            if (!simulated_imu_loss_announced) {
                std::cout << "\n[IMU Thread] 注入 IMU 数据流中断故障" << std::endl;
                simulated_imu_loss_announced = true;
            }
        } else {
            // 成功与失败都提交，使估计线程能及时发现读取失败，而不是沿用旧姿态。
            imu_buffer_.commitWrite();
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
    std::unique_ptr<StateEstimator> estimator;
    if (config_.estimator_backend == StateEstimatorBackend::LINEAR_KF) {
#ifdef LINEAR_KF_AVAILABLE
        estimator = std::make_unique<LinearKFStateEstimatorAdapter>();
#else
        std::cerr << "[EST Thread] Linear KF 不可用" << std::endl;
        requestEmergencyStop();
        return;
#endif
    } else {
        estimator = std::make_unique<ComplementaryStateEstimator>();
    }
    const long period_ns = 1'000'000'000L / config_.estimation_hz;
    const int bus_count = static_cast<int>(config_.buses.size());

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const uint64_t estimation_start_ns =
        static_cast<uint64_t>(next.tv_sec) * 1'000'000'000ULL
      + static_cast<uint64_t>(next.tv_nsec);
    const uint64_t simulated_fault_start_ns = estimation_start_ns
      + static_cast<uint64_t>(config_.simulated_fault_delay_ms) * 1'000'000ULL;
    bool simulated_fault_announced = false;
    const uint64_t simulated_slip_start_ns = estimation_start_ns
      + static_cast<uint64_t>(config_.simulated_leg_slip_delay_ms) * 1'000'000ULL;
    bool simulated_slip_announced = false;

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
        uint64_t joint_feedback_timestamp_ns[12] = {0};
        uint64_t joint_age_ns[12];
        uint32_t joint_failure_count[12] = {0};
        for (uint64_t& age : joint_age_ns) age = UINT64_MAX;

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
                // ParallelBus::correct 表示“最近一次事务”是否成功。500 Hz 总线上的
                // 单次 CRC 丢包不能让 50 Hz 状态帧立即失效；继续使用最近一次成功
                // 的反馈，真正的新鲜度由 joint_age_ns 和 100 ms 阈值判断。
                joint_valid[mid] = s.feedback_timestamp_ns > 0 && s.merror == 0;
                joint_feedback_timestamp_ns[mid] = s.feedback_timestamp_ns;
                joint_failure_count[mid] = s.consecutive_failures;
            }
        }

        // 必须在取得全部反馈快照之后采样 now。若先采样，恰好随后由总线线程
        // 更新的反馈时间会短暂落在“未来”，年龄保留 UINT64_MAX 并误报超时。
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        const uint64_t now_ns =
            static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
          + static_cast<uint64_t>(ts.tv_nsec);
        for (int mid = 0; mid < 12; ++mid) {
            const uint64_t feedback_ns = joint_feedback_timestamp_ns[mid];
            if (feedback_ns > 0 && feedback_ns <= now_ns) {
                joint_age_ns[mid] = now_ns - feedback_ns;
            } else if (feedback_ns > now_ns) {
                // 同一 CLOCK_MONOTONIC 下此分支只可能是数据损坏或时钟错误。
                joint_valid[mid] = false;
            }
        }

        // Step 5 实机安全验收用故障注入：真实总线仍正常收发阻尼，只有送入
        // 状态估计器的指定关节反馈时间被冻结，以验证 100 ms 超时保护。
        const int simulated_id = config_.simulated_motor_timeout_id;
        if (simulated_id >= 0 && now_ns >= simulated_fault_start_ns) {
            if (!simulated_fault_announced) {
                std::cout << "\n[EST Thread] 注入 motor " << simulated_id
                          << " 反馈超时故障" << std::endl;
                simulated_fault_announced = true;
            }
            const uint64_t simulated_age_ns = now_ns - simulated_fault_start_ns;
            joint_age_ns[simulated_id] = simulated_age_ns;
            joint_failure_count[simulated_id] = static_cast<uint32_t>(
                simulated_age_ns / static_cast<uint64_t>(period_ns));
        }

        // Step 7 打滑降级验收：仅改变送入估计器的局部速度副本。
        // 真实反馈、电机指令和总线线程均不受影响，Step 7 模式仍强制 dry-run。
        const int simulated_slip_leg = config_.simulated_leg_slip;
        if (simulated_slip_leg >= 0 && now_ns >= simulated_slip_start_ns) {
            if (!simulated_slip_announced) {
                static const char* kLegNames[4] = {"FL", "FR", "RL", "RR"};
                std::cout << "\n[EST Thread] 注入 " << kLegNames[simulated_slip_leg]
                          << " 估计速度不一致（电机指令未改变）"
                          << std::endl;
                simulated_slip_announced = true;
            }
            // Hip 角速度偏置约产生 1 m/s 足端速度，明显超过 0.35 m/s
            // 的多足约束残差阈值，但仍是有限的诊断数值。
            joint_dq[simulated_slip_leg] += 4.0f;
        }

        // ── 3. 运行状态估计 ──
        EstimatedState& est = est_buffer_.acquireWriteSlot();
        est = estimator->update(
            *imu, joint_q, joint_dq, joint_tau, joint_err, joint_valid,
            joint_age_ns, joint_failure_count, now_ns);
        {
            std::lock_guard<std::mutex> lock(monitor_est_mutex_);
            monitor_est_snapshot_ = est;
            monitor_est_available_ = true;
        }
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
                int first_bad_joint = -1;
                for (int i = 0; i < 12; ++i) {
                    if (!est->joint_valid[i]) {
                        first_bad_joint = i;
                        break;
                    }
                }
                std::cerr << "[NN Thread] 状态估计连续无效, status=" << est->status_code
                          << ", bad_joint=" << first_bad_joint;
                if (first_bad_joint >= 0) {
                    std::cerr << ", age="
                              << static_cast<double>(est->joint_age_ns[first_bad_joint]) / 1.0e6
                              << "ms, failures="
                              << est->joint_failure_count[first_bad_joint];
                }
                std::cerr << ", invalid_joints={";
                bool first = true;
                for (int i = 0; i < 12; ++i) {
                    if (est->joint_valid[i]) continue;
                    if (!first) std::cerr << ",";
                    first = false;
                    std::cerr << i << ":age=";
                    if (est->joint_age_ns[i] == UINT64_MAX) {
                        std::cerr << "unknown";
                    } else {
                        std::cerr << std::fixed << std::setprecision(2)
                                  << static_cast<double>(est->joint_age_ns[i]) / 1.0e6
                                  << "ms" << std::defaultfloat;
                    }
                    std::cerr << "/fail=" << est->joint_failure_count[i];
                }
                std::cerr << "}, 请求安全停机" << std::endl;
                if (motor_ctrl_) {
                    printBusTimingSnapshot(
                        *motor_ctrl_, "[BusTiming@fault]", std::cerr);
                }
                requestEmergencyStop();
            }
            continue;
        }
        consecutive_invalid_states = 0;

        // 命令独立于状态估计；只在本地栈上做一次快照，不持锁推理。
        std::array<float, 3> command;
        {
            std::lock_guard<std::mutex> lock(velocity_command_mutex_);
            command = velocity_command_;
        }
        nn_policy_->setVelocityCommand(command);

        // ── 2. 运行推理 (计时) ──
        auto t0 = std::chrono::high_resolution_clock::now();

        NNCommandSet cmds;
        bool inference_ok = nn_policy_->infer(*est, cmds);

        auto t1 = std::chrono::high_resolution_clock::now();
        int latency_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

        // ── 3. 日志记录 (可选) ──
        if (nn_logger_ && nn_logger_->isEnabled()) {
            std::array<float, 48> actor_observation{};
            std::array<float, 12> actor_raw_action{};
            const bool has_actor_io = nn_policy_->getLastActorIO(
                actor_observation, actor_raw_action);
            nn_logger_->log(*est, cmds, inference_ok, latency_us,
                            has_actor_io ? &actor_observation : nullptr,
                            has_actor_io ? &actor_raw_action : nullptr);
        }

        // 这里只提交安全层最终接受的电机命令历史；Actor 原始 action 历史
        // 已由 ONNXPolicy::infer() 保存，二者不得互相覆盖。
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
