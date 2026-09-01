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
 *   --contact-diagnostics 仅增加接触/里程计详情输出，不改变控制模式
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
#include <algorithm>
#include <sstream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>

static void sigint_handler(int sig)
{
    (void)sig;
    // 信号上下文只置位；通信、锁和线程回收由主流程执行。
    requestEmergencyStop();
}

static bool processKeyboardLine(const std::string& line,
                                RobotController& controller)
{
    std::istringstream stream(line);
    std::string command;
    if (!(stream >> command)) return false;

    if (command == "s" || command == "S") {
        std::cout << "\n[Main] 收到键盘急停请求" << std::endl;
        requestEmergencyStop();
        return true;
    }
    if (command == "1" || command == "2" || command == "3") {
        static const char* markers[3] = {"BASELINE", "UNLOADED", "RELOADED"};
        std::cout << "\n[ContactMarker] " << markers[command[0] - '1']
                  << std::endl;
        return false;
    }
    if (command == "z" || command == "Z" || command == "zero") {
        controller.submitVelocityCommand(0.0f, 0.0f, 0.0f);
        std::cout << "\n[Command] 目标回零（经斜坡平滑执行）"
                  << std::endl;
        return false;
    }
    if (command == "v" || command == "V") {
        float vx = 0.0f, vy = 0.0f, yaw_rate = 0.0f;
        int hold_ms = 0;
        if (!(stream >> vx >> vy >> yaw_rate)) {
            std::cerr << "\n[Command] 格式: v <vx> <vy> <yaw_rate> [hold_ms]"
                      << std::endl;
            return false;
        }
        std::string hold_text;
        if (stream >> hold_text) {
            try {
                size_t parsed = 0;
                hold_ms = std::stoi(hold_text, &parsed);
                if (parsed != hold_text.size() || hold_ms <= 0)
                    throw std::invalid_argument("hold_ms");
            } catch (const std::exception&) {
                std::cerr << "\n[Command] hold_ms必须为正整数" << std::endl;
                return false;
            }
        }
        std::string extra;
        if (stream >> extra) {
            std::cerr << "\n[Command] 参数过多" << std::endl;
            return false;
        }
        const auto result = controller.submitVelocityCommand(
            vx, vy, yaw_rate, hold_ms);
        if (result == VelocityCommandSubmitResult::INVALID) {
            std::cerr << "\n[Command] 拒绝NaN/Inf命令" << std::endl;
            return false;
        }
        const auto status = controller.getVelocityCommandStatus();
        std::cout << "\n[Command] raw=(" << vx << "," << vy << ","
                  << yaw_rate << ") limited=(" << status.limited[0] << ","
                  << status.limited[1] << "," << status.limited[2] << ")"
                  << (result == VelocityCommandSubmitResult::CLAMPED
                          ? " [CLAMPED]" : "")
                  << " hold=" << status.effective_hold_ns / 1'000'000ULL
                  << "ms" << std::endl;
        if (status.hold_clamped) {
            std::cerr << "[Command] 注意：请求保持 " << hold_ms
                      << "ms，已按安全上限截断为 "
                      << status.effective_hold_ns / 1'000'000ULL
                      << "ms" << std::endl;
        }
        return false;
    }

    std::cerr << "\n[Command] 未知输入；可用 s | z | "
                 "v <vx> <vy> <yaw_rate> [hold_ms]" << std::endl;
    return false;
}

static void keyboardStopLoop(std::atomic<bool>& listener_running,
                             RobotController& controller)
{
    std::string pending_line;
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
            const char ch = input[i];
            if (ch == '\n' || ch == '\r') {
                if (!pending_line.empty()
                        && processKeyboardLine(pending_line, controller)) return;
                pending_line.clear();
            } else if (pending_line.size() < 255) {
                pending_line.push_back(ch);
            }
        }
    }
}

static void fifoCommandLoop(std::atomic<bool>& listener_running,
                            RobotController& controller, int fifo_fd)
{
    std::string pending_line;
    while (listener_running.load(std::memory_order_acquire)
           && !isEmergencyStopRequested()) {
        pollfd pfd{fifo_fd, POLLIN, 0};
        const int rc = poll(&pfd, 1, 100);
        if (rc <= 0 || !(pfd.revents & POLLIN)) continue;

        char input[256];
        const ssize_t count = ::read(fifo_fd, input, sizeof(input));
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            break;
        }
        for (ssize_t i = 0; i < count; ++i) {
            const char ch = input[i];
            if (ch == '\n' || ch == '\r') {
                if (!pending_line.empty()
                        && processKeyboardLine(pending_line, controller)) return;
                pending_line.clear();
            } else if (pending_line.size() < 255) {
                pending_line.push_back(ch);
            }
        }
    }
}

static int openCommandFifo(const std::string& path, bool& created)
{
    created = false;
    struct stat info{};
    if (lstat(path.c_str(), &info) == 0) {
        if (!S_ISFIFO(info.st_mode)) {
            std::cerr << "命令管道路径已存在且不是FIFO: " << path
                      << std::endl;
            return -1;
        }
    } else {
        if (errno != ENOENT || mkfifo(path.c_str(), 0600) != 0) {
            std::cerr << "创建命令管道失败: " << path
                      << " errno=" << errno << std::endl;
            return -1;
        }
        created = true;

        // sudo启动时把FIFO交给原用户，第二终端无需sudo即可写入。
        const char* sudo_uid = std::getenv("SUDO_UID");
        const char* sudo_gid = std::getenv("SUDO_GID");
        if (sudo_uid && sudo_gid) {
            const uid_t uid = static_cast<uid_t>(std::strtoul(sudo_uid, nullptr, 10));
            const gid_t gid = static_cast<gid_t>(std::strtoul(sudo_gid, nullptr, 10));
            if (chown(path.c_str(), uid, gid) != 0) {
                std::cerr << "警告: 无法调整命令管道所有者" << std::endl;
            }
        }
    }

    return open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
}

// ── 辅助: 打印状态摘要 ───────────────────────────────────────────────────────

static void printStatus(const RobotController& controller,
                        bool step7_diagnostics,
                        bool contact_diagnostics)
{
    IMURawData imu{};
    EstimatedState est{};
    const bool has_imu = controller.getLatestIMUData(imu);
    const bool has_est = controller.getLatestEstimatedState(est);
    const VelocityCommandStatus command = controller.getVelocityCommandStatus();

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

    std::cout << " | CMD raw=(" << command.raw[0] << "," << command.raw[1]
              << "," << command.raw[2] << ") applied=(" << command.applied[0]
              << "," << command.applied[1] << "," << command.applied[2] << ")"
              << " age=" << static_cast<double>(command.age_ns) / 1.0e6 << "ms"
              << " timeout=" << (command.timed_out ? 1 : 0);

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

        if (step7_diagnostics || contact_diagnostics) {
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

            std::cout << (step7_diagnostics
                              ? "\n[Step7] legs=FL,FR,RL,RR"
                              : "\n[ContactDiag] legs=FL,FR,RL,RR")
                      << " ts_ns=" << est.timestamp_ns
                      << " contact=";
            for (int leg = 0; leg < 4; ++leg) std::cout << est.contact[leg];
            std::cout << " conf=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << est.contact_confidence[leg];
            std::cout << ") tau_norm=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << torque_norm[leg];
            std::cout << ") force_xyz=(";
            for (int leg = 0; leg < 4; ++leg) {
                if (leg) std::cout << ";";
                std::cout << est.foot_force_body[leg][0] << ","
                          << est.foot_force_body[leg][1] << ","
                          << est.foot_force_body[leg][2];
            }
            std::cout << ") force_valid=";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << est.foot_force_valid[leg];
            float normal_force_sum = 0.0f;
            for (int leg = 0; leg < 4; ++leg)
                normal_force_sum += est.normal_force[leg];
            std::cout << " fz_sum=" << normal_force_sum;
            std::cout << " force_residual=(";
            for (int leg = 0; leg < 4; ++leg)
                std::cout << (leg ? "," : "") << est.foot_force_residual[leg];
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
              << "  --contact-diagnostics\n"
              << "                      仅以2 Hz输出接触/里程计详情，不改变dry-run或电机控制\n"
              << "  --simulate-leg-slip <FL|FR|RL|RR>\n"
              << "                      仅Step 7诊断：8秒后只向估计输入注入单腿异常速度\n"
              << "  --simulate-motor-timeout <ID>\n"
              << "                      仅监测模式：3秒后模拟该电机反馈停止\n"
              << "  --simulate-imu-stream-loss\n"
              << "                      仅监测模式：3秒后停止向估计线程发布 IMU 帧\n"
              << "  --log               记录每帧输入/输出到 CSV\n"
              << "  --log-file <path>   CSV 日志路径 (默认: /tmp/nn_inference_log.csv)\n"
              << "  --no-validate       禁用NN限位/JUMP验证（不关闭独立运动保护）\n"
              << "  --compare           同时运行 StandingPolicy 对比输出差异\n"
              << "  --motor-hz <hz>     电机总线频率 (默认: 500)\n"
              << "  --imu-hz <hz>       IMU 读取频率 (默认: 200)\n"
              << "  --estimator <name>  complementary 或 kalman\n"
              << "  --command-fifo <path> 速度命令FIFO路径\n"
              << "  --no-command-fifo  禁用第二终端命令管道\n"
              << "  -h, --help          打印此帮助\n"
              << "\n急停: 输入 s/S 后回车，或按 Ctrl+C\n"
              << "速度命令: v <vx> <vy> <yaw_rate> [hold_ms]\n"
              << "  范围为[-1,1]，默认500ms超时，hold_ms最大5000ms\n"
              << "平滑回零: z 后回车\n"
              << "接触诊断标记: 1=基线，2=卸载，3=重新承重（均需按回车）\n"
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
    bool contact_diagnostics = false;
    std::string command_fifo_path = "/tmp/creeper_velocity_command.fifo";
    bool command_fifo_enabled = true;

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
        else if (arg == "--contact-diagnostics") {
            // 只增强可观测性；不修改NN策略、电机指令或安全保护。
            contact_diagnostics = true;
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
        else if (arg == "--command-fifo" && i + 1 < argc) {
            command_fifo_path = argv[++i];
        }
        else if (arg == "--no-command-fifo") {
            command_fifo_enabled = false;
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
    std::cout << "  运动保护: "
              << (config.motion_safety.enabled ? "ON" : "OFF")
              << " target_rate="
              << (config.motion_safety.enforce_target_velocity
                      ? "ENFORCED<=" : "MONITOR_ONLY threshold=")
              << config.motion_safety.max_target_velocity_rad_s << "rad/s"
              << " feedback_speed="
              << (config.motion_safety.enforce_feedback_velocity
                      ? "ENFORCED<=" : "MONITOR_ONLY threshold=")
              << config.motion_safety.max_feedback_velocity_rad_s << "rad/s"
              << " aggregate_delta="
              << (config.motion_safety.enforce_aggregate_target_delta
                      ? "ENFORCED<=" : "MONITOR_ONLY threshold=")
              << config.motion_safety.max_aggregate_target_delta_rad << "rad"
              << std::endl;
    if (contact_diagnostics) {
        std::cout << "  接触诊断: ON (仅输出，不改变控制)" << std::endl;
    }
    if (config.nn_flags.log_io) {
        std::cout << "  日志文件: " << config.nn_flags.log_filepath << std::endl;
    }

    // ── 创建控制器和键盘监听 ──
    RobotController controller(config);
    std::atomic<bool> listener_running{true};
    std::thread keyboard_thread(
        keyboardStopLoop, std::ref(listener_running), std::ref(controller));
    bool command_fifo_created = false;
    int command_fifo_fd = -1;
    std::thread fifo_thread;
    if (command_fifo_enabled) {
        command_fifo_fd = openCommandFifo(
            command_fifo_path, command_fifo_created);
        if (command_fifo_fd >= 0) {
            fifo_thread = std::thread(
                fifoCommandLoop, std::ref(listener_running),
                std::ref(controller), command_fifo_fd);
            std::cout << "[Main] 速度命令FIFO: " << command_fifo_path
                      << std::endl;
        } else {
            std::cerr << "[Main] FIFO不可用，仍可在主终端输入命令"
                      << std::endl;
        }
    }

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
    const int status_print_period = (config.step7_diagnostics || contact_diagnostics) ? 10
                                  : config.monitor_only ? 20 : 200;  // 0.5/1/10 s
    while (exit_code == 0 && controller.isRunning()) {
        usleep(50000);
        if (!controller.checkRuntimeSafety()) break;
        print_count++;

        if (print_count % status_print_period == 0) {
            printStatus(controller, config.step7_diagnostics,
                        contact_diagnostics);
        }
    }

    // ── 统一安全清理 ──
    controller.safeShutdown();
    listener_running.store(false, std::memory_order_release);
    if (keyboard_thread.joinable()) keyboard_thread.join();
    if (fifo_thread.joinable()) fifo_thread.join();
    if (command_fifo_fd >= 0) close(command_fifo_fd);
    if (command_fifo_created) unlink(command_fifo_path.c_str());

    std::cout << "\n[Main] 程序退出" << std::endl;
    return exit_code;
}
