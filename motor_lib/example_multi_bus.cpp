/**
 * @example example_multi_bus.cpp
 *
 * 多路 RS-485 总线并行控制演示
 *
 * 架构:
 *   4 路独立串口 → 4 个独立线程 → 每路 3 个电机 = 12 电机并行控制
 *
 * 硬件要求:
 *   - 4 路串口 (ttyS4 ~ ttyS7)
 *   - 4 个 GPIO 用于 RS-485 方向控制
 *   - 12 台 GO-M8010-6 电机
 *
 * 编译:
 *   cd build && cmake .. && make example_multi_bus
 *
 * 运行:
 *   sudo ./example_multi_bus
 */

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include "parallel_bus.h"

// ── 硬件配置 ────────────────────────────────────────────────────────────────
//
// 4 路总线的 GPIO 和串口映射（按实际接线修改）
//
struct BusConfig {
    int gpio_chip;
    int gpio_line;
    const char* serial_port;
};

static const BusConfig BUS_CONFIGS[] = {
    {0, 63,  "/dev/ttyS4"},   // 总线 A → 电机 0,1,2
    {0, 120, "/dev/ttyS5"},   // 总线 B → 电机 3,4,5
    {0, 121, "/dev/ttyS6"},   // 总线 C → 电机 6,7,8
    {0, 122, "/dev/ttyS7"},   // 总线 D → 电机 9,10,11
};

constexpr int MOTORS_PER_BUS = 3;
constexpr int CONTROL_HZ     = 500;

// ── 全局标志 ────────────────────────────────────────────────────────────────
static volatile bool g_running = true;
void sigint_handler(int) { g_running = false; }

// ── 主函数 ──────────────────────────────────────────────────────────────────

int main()
{
    std::signal(SIGINT, sigint_handler);

    constexpr int BUS_COUNT = sizeof(BUS_CONFIGS) / sizeof(BUS_CONFIGS[0]);

    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║  多路 RS-485 并行总线控制演示                 ║\n";
    std::cout << "║  " << BUS_COUNT << " 路总线 × " << MOTORS_PER_BUS
              << " 电机/路 = " << (BUS_COUNT * MOTORS_PER_BUS)
              << " 电机  @ " << CONTROL_HZ << " Hz\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    // ── 1. 创建多路总线控制器 ──
    MultiBusController controller;

    for (int b = 0; b < BUS_COUNT; ++b) {
        const auto& cfg = BUS_CONFIGS[b];
        try {
            controller.addBus(cfg.gpio_chip, cfg.gpio_line, cfg.serial_port);
            std::cout << "[总线 " << b << "] gpiochip" << cfg.gpio_chip
                      << ":" << cfg.gpio_line
                      << "  →  " << cfg.serial_port << "  ✓\n";
        } catch (const std::exception& e) {
            std::cerr << "[总线 " << b << "] 创建失败: " << e.what() << "\n";
            return 1;
        }
    }

    // ── 2. 为每路总线注册电机 ──
    for (int b = 0; b < BUS_COUNT; ++b) {
        for (int m = 0; m < MOTORS_PER_BUS; ++m) {
            int global_id = b * MOTORS_PER_BUS + m;
            controller.bus(b).addMotor(m);   // 每路内部 ID 从 0 开始
            std::cout << "  总线" << b << " 注册电机 id=" << m
                      << " (全局编号 " << global_id << ")\n";
        }
    }

    // ── 3. 启动所有总线线程 ──
    controller.startAll(CONTROL_HZ);
    std::cout << "\n[OK] " << BUS_COUNT << " 路总线全部启动 @ "
              << CONTROL_HZ << " Hz\n\n";

    // ── 4. 主循环：异步下发指令 + 读取状态 ──
    const float kp = 0.03f;
    const float kd = 0.01f;
    int step = 0;

    while (g_running) {
        // 所有电机统一做位置摆动（每路相位不同）
        for (int b = 0; b < BUS_COUNT; ++b) {
            float phase_offset = b * 0.5f;   // 每路相位错开
            float target = (((step + b * 25) / 25) % 2 == 0) ? 0.3f : -0.3f;

            for (int m = 0; m < MOTORS_PER_BUS; ++m) {
                controller.bus(b).setPosition(m, target, kp, kd);
            }
        }

        // 每秒打印一次各总线实际频率
        if (step % CONTROL_HZ == 0) {
            std::cout << "──── 周期 " << (step / CONTROL_HZ) << " ────\n";
            for (int b = 0; b < BUS_COUNT; ++b) {
                auto& bus = controller.bus(b);
                MotorState s0 = bus.getState(0);
                std::cout << "  总线" << b
                          << "  实际频率=" << std::fixed << std::setprecision(0)
                          << bus.getActualHz() << " Hz"
                          << "  motor0: q=" << std::setprecision(3) << s0.q
                          << "  dq=" << s0.dq
                          << "  err=" << s0.merror
                          << "  correct=" << (s0.correct ? "是" : "否")
                          << "\n";
            }
        }

        step++;
        usleep(1'000'000 / CONTROL_HZ);   // 按控制频率休眠
    }

    // ── 5. 优雅停机 ──
    std::cout << "\n正在停机...\n";

    // 先刹车所有电机
    for (int b = 0; b < BUS_COUNT; ++b) {
        for (int m = 0; m < MOTORS_PER_BUS; ++m) {
            controller.bus(b).brake(m);
        }
    }
    usleep(50000);  // 给总线线程一点时间执行刹车指令

    controller.stopAll();
    std::cout << "全部总线已停止。\n";

    return 0;
}
