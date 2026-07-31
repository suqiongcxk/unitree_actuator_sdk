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
 *   --log              记录每帧输入/输出到 CSV
 *   --log-file <path>  CSV 日志路径 (默认: /tmp/nn_inference_log.csv)
 *   --no-validate      禁用运行时验证
 *   --compare          同时运行 StandingPolicy 对比输出差异
 *   --motor-hz <hz>    电机总线频率 (默认: 500)
 *   --imu-hz <hz>      IMU 读取频率 (默认: 200)
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
#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <cstring>

// ── 全局指针，用于 SIGINT 信号处理 ──────────────────────────────────────────
static std::atomic<RobotController*> g_active_controller{nullptr};

static void sigint_handler(int sig)
{
    (void)sig;
    auto* ctrl = g_active_controller.load();
    if (ctrl) {
        std::cout << "\n[Main] 收到中断信号，正在关闭..." << std::endl;
        ctrl->stop();
    }
}

// ── 辅助: 打印状态摘要 ───────────────────────────────────────────────────────

static void printStatus(const RobotController& controller)
{
    const IMURawData* imu = const_cast<RobotController&>(controller).getLatestIMUData();
    const EstimatedState* est = const_cast<RobotController&>(controller).getLatestEstimatedState();

    std::cout << "\r" << std::fixed << std::setprecision(2)
              << "IMU: ";

    if (imu) {
        std::cout << "acc=("
                  << imu->acc.x << "," << imu->acc.y << "," << imu->acc.z
                  << ") gyro=("
                  << imu->gyro.x << "," << imu->gyro.y << "," << imu->gyro.z << ")";
    } else {
        std::cout << "waiting...";
    }

    std::cout << " | EST: ";
    if (est) {
        std::cout << "joint[0]=" << est->joint_position[0]
                  << " contact=";
        for (int i = 0; i < 4; ++i) std::cout << est->contact[i];
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
              << "  --dry-run           NN 计算但不发送电机指令 (安全测试)\n"
              << "  --log               记录每帧输入/输出到 CSV\n"
              << "  --log-file <path>   CSV 日志路径 (默认: /tmp/nn_inference_log.csv)\n"
              << "  --no-validate       禁用运行时验证\n"
              << "  --compare           同时运行 StandingPolicy 对比输出差异\n"
              << "  --motor-hz <hz>     电机总线频率 (默认: 500)\n"
              << "  --imu-hz <hz>       IMU 读取频率 (默认: 200)\n"
              << "  -h, --help          打印此帮助\n"
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
        else if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            return 0;
        }
        else {
            std::cerr << "未知选项: " << arg << "\n使用 -h 查看帮助" << std::endl;
            return 1;
        }
    }

    // ── 信号处理 ──
    std::signal(SIGINT,  sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Robot Control System — Multi-Threaded Architecture  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;

    // ── 打印配置 ──
    std::cout << "\n配置:" << std::endl;
    std::cout << "  IMU:     " << config.imu_device << " @ " << config.imu_hz << "Hz" << std::endl;
    std::cout << "  估计:    " << config.estimation_hz << "Hz" << std::endl;
    std::cout << "  电机:    " << config.buses.size() << " 路总线 @ " << config.motor_hz << "Hz" << std::endl;
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

    // ── 创建控制器 ──
    RobotController controller(config);
    g_active_controller.store(&controller);

    // ── 初始化 + 启动 ──
    if (!controller.initialize()) {
        std::cerr << "[Main] 硬件初始化失败, 退出" << std::endl;
        return 1;
    }
    if (!controller.start()) {
        std::cerr << "[Main] 线程启动失败, 退出" << std::endl;
        return 1;
    }

    std::cout << "\n[Main] ✓ 系统运行中 — 按 Ctrl+C 退出\n" << std::endl;

    // ── 主线程: 健康监控循环 ──
    int print_count = 0;
    while (controller.isRunning()) {
        sleep(1);
        print_count++;

        if (print_count % 10 == 0) {
            printStatus(controller);
        }
    }

    // ── 清理 ──
    g_active_controller.store(nullptr);
    controller.stop();

    std::cout << "\n[Main] 程序退出" << std::endl;
    return 0;
}
