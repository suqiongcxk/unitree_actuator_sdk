/**
 * @file robot_control_main.cpp
 *
 * 多线程机器人控制 — 主入口
 *
 * 用法:
 *   sudo ./robot_control [选项]
 *
 * 选项:
 *   --onnx <path>       ONNX 模型文件路径
 *   --dry-run           NN 计算但不发送电机指令 (安全测试)
 *   --monitor-only      跳过标定/站立，电机只阻尼，强制 NN dry-run
 *   --step7-diagnostics 正常标定/站立，强制 NN dry-run，输出接触与里程计详情
 *   --simulate-leg-slip <FL|FR|RL|RR>  仅 Step 7 诊断，模拟单腿打滑
 *   --log              记录每帧输入/输出到 CSV
 *   --log-file <path>  CSV 日志路径 (默认: /tmp/nn_inference_log.csv)
 *   --no-validate      禁用运行时验证
 *   --compare          同时运行 StandingPolicy 对比输出差异
 *   --motor-hz <hz>    电机总线频率 (默认: 500)
 *   --imu-hz <hz>      IMU 读取频率 (默认: 200)
 *   --estimator <name>  complementary (默认) 或 kalman
 *   -h, --help         打印帮助
 *
 * 验证工作流示例:
 *   # 第1步: 干运行 ONNX 模型, 记录日志, 对比站立策略
 *   sudo ./robot_control --onnx model.onnx --dry-run --log --compare
 *
 *   # 第2步: 检查日志确认输出合理后, 去掉 --dry-run 实际控制
 *   sudo ./robot_control --onnx model.onnx --log --compare
 *
 *   # 第3步: 确认稳定后, 去掉调试选项
 *   sudo ./robot_control --onnx model.onnx
 */

#include "robot_controller.h"
#include "emergency_stop.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <cerrno>
#include <poll.h>
#include <cmath>

static void sigint_handler(int sig)
{
    (void)sig;
    // 信号上下文只置位；通信、锁和线程回收由主流程执行。
    requestEmergencyStop();
}

static void keyboardStopLoop(std::atomic<bool>& listener_running)
{
    while (listener_running.load(std::memory_order_acquire)
           && !isEmergencyStopRequested()) {
        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        int rc = poll(&pfd, 1, 100);
        if (rc <= 0 || !(pfd.revents & POLLIN)) continue;

        // 不混用 poll() 与带内部缓冲的 std::getline()；直接读取终端字符。
        // 终端保持默认 canonical 模式，因此用户仍需按 Enter。
        char input[64];
        const ssize_t count = ::read(STDIN_FILENO, input, sizeof(input));
        if (count == 0) break;
        if (count < 0) {
            if (errno == EINTR) continue;
            break;
        }
        for (ssize_t i = 0; i < count; ++i) {
            if (input[i] == 's' || input[i] == 'S') {
                std::cout << "\n[Main] 收到键盘急停请求" << std::endl;
                requestEmergencyStop();
                return;
            }
        }
    }
}

// ── 辅助: 打印状态摘要 ───────────────────────────────────────────────────────

static void printStatus(const RobotController& controller, bool step7_diagnostics)
{
    IMURawData imu{};
    EstimatedState est{};
    const bool has_imu = controller.getLatestIMUData(imu);
    const bool has_est = controller.getLatestEstimatedState(est);

    std::cout << "\n[Health] " << std::fixed << std::setprecision(2)
              << "IMU: ";

    if (has_imu) {
        std::cout << "acc=("
                  << imu.acc.x << "," << imu.acc.y << "," << imu.acc.z
                  << ") gyro=("
                  << imu.gyro.x << "," << imu.gyro.y << "," << imu.gyro.z << ")";
    } else {
        std::cout << "waiting...";
    }

    std::cout << " | EST: ";
    if (has_est) {
        int first_bad_joint = -1;
        for (int i = 0; i < 12; ++i) {
            if (!est.joint_valid[i]) {
                first_bad_joint = i;
                break;
            }
        }

        const double imu_age_ms = static_cast<double>(est.imu_age_ns) / 1.0e6;
        const bool motor_age_known = est.max_joint_age_ns != UINT64_MAX;

        std::cout << "valid=" << (est.valid ? 1 : 0)
                  << " status=" << est.status_code
                  << " invalid_n=" << est.consecutive_invalid_count
                  << " imu_age=" << imu_age_ms << "ms"
                  << " motor_age=";
        if (motor_age_known) {
            std::cout << static_cast<double>(est.max_joint_age_ns) / 1.0e6 << "ms";
        } else {
            std::cout << "unknown";
        }
        std::cout << " bad_joint=" << first_bad_joint
                  << " joint[0]=" << est.joint_position[0]
                  << " contact=";
        for (int i = 0; i < 4; ++i) std::cout << est.contact[i];

        if (first_bad_joint >= 0) {
            // 不再只显示第一个坏关节；同总线三个ID同时变旧时可直接识别。
            std::cout << " bad_joints={";
            bool first = true;
            for (int i = 0; i < 12; ++i) {
                if (est.joint_valid[i]) continue;
                if (!first) std::cout << ",";
                first = false;
                std::cout << i << ":";
                if (est.joint_age_ns[i] == UINT64_MAX) {
                    std::cout << "unknown";
                } else {
                    std::cout << static_cast<double>(est.joint_age_ns[i]) / 1.0e6
                              << "ms";
                }
                std::cout << "/" << est.joint_failure_count[i];
            }
            std::cout << "}";
        }

        if (step7_diagnostics) {
            float torque_norm[4] = {0};
            float foot_speed[4] = {0};
            for (int leg = 0; leg < 4; ++leg) {
                const int ids[3] = {leg, 4 + leg, 8 + leg};
                for (int j = 0; j < 3; ++j) {
                    const float tau = est.joint_torque[ids[j]];
                    torque_norm[leg] += tau * tau;
                    const float v = est.foot_velocity[leg][j];
                    foot_speed[leg] += v * v;
                }
                torque_norm[leg] = std::sqrt(torque_norm[leg]);
                foot_speed[leg] = std::sqrt(foot_speed[leg]);
            }

            std::cout << "\n[Step7] legs=FL,FR,RL,RR"
                      << " contact=";
            for (int leg = 0; leg < 4; ++leg) std::cout << est.contact[leg];
            std::cout << " conf=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << est.contact_confidence[leg];
            std::cout << ") tau_norm=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << torque_norm[leg];
            std::cout << ") foot_z=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << est.foot_position[leg][2];
            std::cout << ") foot_speed=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << foot_speed[leg];
            std::cout << ") vel_w=("
                      << est.linear_velocity[0] << ","
                      << est.linear_velocity[1] << ","
                      << est.linear_velocity[2] << ") vel_b=("
                      << est.body_linear_velocity[0] << ","
                      << est.body_linear_velocity[1] << ","
                      << est.body_linear_velocity[2] << ") height="
                      << est.body_height
                      << " vel_conf=" << est.linear_velocity_confidence
                      << " airborne=" << est.airborne
                      << " slip=" << est.slipping
                      << " impact=" << est.landing_impact
                      << " odom_valid=" << est.leg_odometry_valid;
        }
    } else {
        std::cout << "waiting...";
    }

    std::cout << std::flush;
}

// ── 帮助信息 ─────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout << "用法: sudo " << prog << " [选项]\n\n"
              << "选项:\n"
              << "  --onnx <path>       ONNX 模型文件路径\n"
              << "  --dry-run           NN 计算但不发送指令（仍会执行标定/站立）\n"
              << "  --monitor-only      跳过标定/站立，电机只阻尼，强制 dry-run\n"
              << "  --step7-diagnostics 正常标定/站立，强制 NN dry-run，输出接触/里程计详情\n"
              << "  --simulate-leg-slip <FL|FR|RL|RR>\n"
              << "                      仅Step 7诊断：8秒后只向估计输入注入单腿异常速度\n"
              << "  --simulate-motor-timeout <ID>\n"
              << "                      仅监测模式：3秒后模拟该电机反馈停止\n"
              << "  --simulate-imu-stream-loss\n"
              << "                      仅监测模式：3秒后停止向估计线程发布 IMU 帧\n"
              << "  --log               记录每帧输入/输出到 CSV\n"
              << "  --log-file <path>   CSV 日志路径 (默认: /tmp/nn_inference_log.csv)\n"
              << "  --no-validate       禁用运行时验证\n"
              << "  --compare           同时运行 StandingPolicy 对比输出差异\n"
              << "  --motor-hz <hz>     电机总线频率 (默认: 500)\n"
              << "  --imu-hz <hz>       IMU 读取频率 (默认: 200)\n"
              << "  --estimator <name>  complementary 或 kalman\n"
              << "  -h, --help          打印此帮助\n"
              << "\n急停: 输入 s/S 后回车，或按 Ctrl+C\n"
              << "\n验证工作流:\n"
              << "  # 第1步: 干运行 ONNX 模型, 记录日志, 对比站立策略\n"
              << "  sudo " << prog << " --onnx model.onnx --dry-run --log --compare\n\n"
              << "  # 第2步: 检查日志确认合理后, 实际控制电机\n"
              << "  sudo " << prog << " --onnx model.onnx --log --compare\n\n"
              << "  # 第3步: 稳定后去掉调试选项\n"
              << "  sudo " << prog << " --onnx model.onnx\n"
              << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    resetEmergencyStop();
    // ── 解析命令行参数 ──
    RobotControlConfig config = RobotController::getDefaultConfig();

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);

        if (arg == "--onnx" && i + 1 < argc) {
            config.onnx_model_path = argv[++i];
        }
        else if (arg == "--dry-run") {
            config.nn_flags.dry_run = true;
        }
        else if (arg == "--monitor-only") {
            config.monitor_only = true;
            config.nn_flags.dry_run = true;
            // 未站立姿态与 StandingPolicy 默认目标差异很大，监测模式无需产生
            // JUMP 验证告警；dry-run 和阻尼约束仍然保持。
            config.nn_flags.validate = false;
        }
        else if (arg == "--step7-diagnostics") {
            config.step7_diagnostics = true;
        }
        else if (arg == "--simulate-leg-slip" && i + 1 < argc) {
            const std::string leg(argv[++i]);
            if (leg == "FL" || leg == "fl") config.simulated_leg_slip = 0;
            else if (leg == "FR" || leg == "fr") config.simulated_leg_slip = 1;
            else if (leg == "RL" || leg == "rl") config.simulated_leg_slip = 2;
            else if (leg == "RR" || leg == "rr") config.simulated_leg_slip = 3;
            else {
                std::cerr << "模拟打滑腿必须是 FL/FR/RL/RR" << std::endl;
                return 1;
            }
        }
        else if (arg == "--simulate-motor-timeout" && i + 1 < argc) {
            config.simulated_motor_timeout_id = std::stoi(argv[++i]);
        }
        else if (arg == "--simulate-imu-stream-loss") {
            config.simulated_imu_stream_loss = true;
        }
        else if (arg == "--log") {
            config.nn_flags.log_io = true;
        }
        else if (arg == "--log-file" && i + 1 < argc) {
            config.nn_flags.log_filepath = argv[++i];
        }
        else if (arg == "--no-validate") {
            config.nn_flags.validate = false;
        }
        else if (arg == "--compare") {
            config.nn_flags.compare = true;
        }
        else if (arg == "--motor-hz" && i + 1 < argc) {
            config.motor_hz = std::stoi(argv[++i]);
        }
        else if (arg == "--imu-hz" && i + 1 < argc) {
            config.imu_hz = std::stoi(argv[++i]);
        }
        else if (arg == "--estimator" && i + 1 < argc) {
            const std::string name(argv[++i]);
            if (name == "complementary") {
                config.estimator_backend = StateEstimatorBackend::COMPLEMENTARY;
            } else if (name == "kalman") {
                config.estimator_backend = StateEstimatorBackend::LINEAR_KF;
            } else {
                std::cerr << "未知状态估计器: " << name
                          << " (只支持 complementary/kalman)" << std::endl;
                return 1;
            }
        }
        else if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            return 0;
        }
        else {
            std::cerr << "未知选项: " << arg << "\n使用 -h 查看帮助" << std::endl;
            return 1;
        }
    }

    if (config.simulated_motor_timeout_id >= 0) {
        if (!config.monitor_only) {
            std::cerr << "--simulate-motor-timeout 只能与 --monitor-only 同时使用"
                      << std::endl;
            return 1;
        }
        if (config.simulated_motor_timeout_id > 11) {
            std::cerr << "模拟超时电机 ID 必须为 0..11" << std::endl;
            return 1;
        }
    }
    if (config.simulated_imu_stream_loss && !config.monitor_only) {
        std::cerr << "--simulate-imu-stream-loss 只能与 --monitor-only 同时使用"
                  << std::endl;
        return 1;
    }
    if (config.simulated_leg_slip >= 0 && !config.step7_diagnostics) {
        std::cerr << "--simulate-leg-slip 只能与 --step7-diagnostics 同时使用"
                  << std::endl;
        return 1;
    }
    if (config.step7_diagnostics) {
        if (config.monitor_only) {
            std::cerr << "--step7-diagnostics 不能与 --monitor-only 同时使用："
                         "Step 7 需要机械标定后的 URDF 关节角" << std::endl;
            return 1;
        }
        if (!config.onnx_model_path.empty()) {
            std::cerr << "--step7-diagnostics 禁止同时加载 ONNX 模型" << std::endl;
            return 1;
        }
        // 只保留站立过渡结束时锁存的低刚度目标，禁止 NN 线程覆盖电机命令。
        config.nn_flags.dry_run = true;
        config.nn_flags.validate = false;
    }

    // ── 信号处理 ──
    std::signal(SIGINT,  sigint_handler);
    std::signal(SIGTERM, sigint_handler);
    // 输出经 tee 管道时，Ctrl+C 可能让下游先退出；禁止 SIGPIPE 绕过安全清理。
    std::signal(SIGPIPE, SIG_IGN);

    std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Robot Control System — Multi-Threaded Architecture  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;

    // ── 打印配置 ──
    std::cout << "\n配置:" << std::endl;
    std::cout << "  IMU:     " << config.imu_device << " @ " << config.imu_hz << "Hz" << std::endl;
    std::cout << "  估计:    " << config.estimation_hz << "Hz, "
              << (config.estimator_backend == StateEstimatorBackend::LINEAR_KF
                    ? "LinearKF" : "Complementary") << std::endl;
    std::cout << "  电机:    " << config.buses.size() << " 路总线 @ " << config.motor_hz << "Hz" << std::endl;
    std::cout << "  启动模式: "
              << (config.monitor_only ? "MONITOR ONLY (跳过标定/站立，只阻尼)"
                  : config.step7_diagnostics
                      ? "STEP 7 DIAGNOSTICS (正常标定/站立，NN不下发)"
                      : "正常标定与控制")
              << std::endl;
    if (config.simulated_motor_timeout_id >= 0) {
        std::cout << "  故障注入: motor " << config.simulated_motor_timeout_id
                  << " 在 " << config.simulated_fault_delay_ms
                  << "ms 后停止刷新反馈时间" << std::endl;
    }
    if (config.simulated_imu_stream_loss) {
        std::cout << "  故障注入: IMU 在 " << config.simulated_fault_delay_ms
                  << "ms 后停止向状态估计发布" << std::endl;
    }
    if (config.simulated_leg_slip >= 0) {
        static const char* kLegNames[4] = {"FL", "FR", "RL", "RR"};
        std::cout << "  诊断注入: " << kLegNames[config.simulated_leg_slip]
                  << " 在 " << config.simulated_leg_slip_delay_ms
                  << "ms 后模拟速度不一致（不修改电机指令）" << std::endl;
    }
    std::cout << "  NN:      " << config.nn_hz << "Hz";
    if (!config.onnx_model_path.empty()) {
        std::cout << " 模型=" << config.onnx_model_path;
    } else {
        std::cout << " (StandingPolicy)";
    }
    std::cout << std::endl;

    std::cout << "  NN 标志: "
              << "dry_run=" << (config.nn_flags.dry_run ? "ON" : "OFF")
              << " validate=" << (config.nn_flags.validate ? "ON" : "OFF")
              << " compare=" << (config.nn_flags.compare ? "ON" : "OFF")
              << " log=" << (config.nn_flags.log_io ? "ON" : "OFF")
              << std::endl;
    if (config.nn_flags.log_io) {
        std::cout << "  日志文件: " << config.nn_flags.log_filepath << std::endl;
    }

    // ── 创建控制器和键盘监听 ──
    RobotController controller(config);
    std::atomic<bool> listener_running{true};
    std::thread keyboard_thread(keyboardStopLoop, std::ref(listener_running));

    // ── 初始化 + 启动 ──
    int exit_code = 0;
    try {
        if (!controller.initialize()) {
            std::cerr << "[Main] 硬件初始化失败, 退出" << std::endl;
            exit_code = 1;
        } else if (!controller.start()) {
            std::cerr << "[Main] 线程启动失败, 退出" << std::endl;
            exit_code = 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[Main] 未处理异常: " << e.what() << std::endl;
        exit_code = 1;
        requestEmergencyStop();
    }

    if (exit_code == 0) {
        std::cout << "\n[Main] ✓ 系统运行中 — 输入 s/S 后回车或按 Ctrl+C 急停\n"
                  << std::endl;
    }

    // ── 主线程: 健康监控循环 ──
    int print_count = 0;
    const int status_print_period = config.step7_diagnostics ? 10
                                  : config.monitor_only ? 20 : 200;  // 0.5/1/10 s
    while (exit_code == 0 && controller.isRunning()) {
        usleep(50000);
        print_count++;

        if (print_count % status_print_period == 0) {
            printStatus(controller, config.step7_diagnostics);
        }
    }

    // ── 统一安全清理 ──
    controller.safeShutdown();
    listener_running.store(false, std::memory_order_release);
    if (keyboard_thread.joinable()) keyboard_thread.join();

    std::cout << "\n[Main] 程序退出" << std::endl;
    return exit_code;
}
