/**
 * @example example_usage.cpp
 *
 * GO-M8010-6 电机控制库 —— 四种模式完整演示
 *
 * 硬件要求:
 *   - Unitree GO-M8010-6 电机，通过 RS-485 连接
 *   - 一个 GPIO 引脚用于 RS-485 方向控制
 *   - Orange Pi（或同类开发板），具备串口和 sysfs GPIO
 *
 * 编译:
 *   cd build && cmake .. && make example_usage
 *
 * 运行（需要 root 或 gpio/串口 权限）:
 *   sudo ./example_usage
 */

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include "motor_controller.h"

// ── 硬件配置（请根据实际接线修改） ──────────────────────────────────────────
constexpr int    GPIO_PIN    = 63;              // GPIO1_D7 → sysfs gpio63
constexpr const char* SERIAL_PORT = "/dev/ttyS4"; // Orange Pi UART4
constexpr unsigned short MOTOR_ID = 0;

// ── 全局运行标志，Ctrl+C 优雅退出 ───────────────────────────────────────────
static volatile bool g_running = true;

void sigint_handler(int) { g_running = false; }

// ── 工具函数：打印电机状态 ──────────────────────────────────────────────────

void printState(const char* label, const MotorState& s)
{
    std::cout << "[" << label << "] "
              << "q="    << std::fixed << std::setprecision(3) << s.q
              << "  dq=" << std::fixed << std::setprecision(3) << s.dq
              << "  tau=" << std::fixed << std::setprecision(3) << s.tau
              << "  temp=" << s.temp << "°C"
              << "  err=" << s.merror
              << "  ok=" << (s.correct ? "是" : "否")
              << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 演示 1 —— 位置控制
// ═══════════════════════════════════════════════════════════════════════════════

void demo_position_control()
{
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout <<   "║  演示 1: 位置控制（单电机独立模式）           ║\n";
    std::cout <<   "╚══════════════════════════════════════════════╝\n";

    MotorController motor(GPIO_PIN, SERIAL_PORT, MOTOR_ID);
    std::cout << "电机 ID=" << motor.getId()
              << "  减速比=" << motor.getGearRatio() << "\n";

    // 在 ±0.5 rad（约 ±28.6°）之间摆动，持续约 5 秒
    const float amplitude = 0.5f;    // 摆动幅度 (rad)
    const float kp = 0.03f;          // 位置刚度系数
    const float kd = 0.01f;          // 速度阻尼系数

    for (int i = 0; i < 50 && g_running; ++i) {
        float target = (i / 25 % 2 == 0) ? amplitude : -amplitude;
        motor.setPosition(target, kp, kd);

        MotorState s = motor.getState();
        printState("位置", s);

        usleep(100000);  // 100 ms 周期
    }

    motor.brake();
    std::cout << "位置演示结束。\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 演示 2 —— 速度控制
// ═══════════════════════════════════════════════════════════════════════════════

void demo_velocity_control()
{
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout <<   "║  演示 2: 速度控制（单电机独立模式）           ║\n";
    std::cout <<   "╚══════════════════════════════════════════════╝\n";

    MotorController motor(GPIO_PIN, SERIAL_PORT, MOTOR_ID);

    const float speed = 3.14f;   // 输出端 0.5 转/秒
    const float kd    = 0.02f;   // 阻尼系数

    // 正转约 3 秒，反转约 3 秒
    for (int dir = 0; dir < 2 && g_running; ++dir) {
        float target = (dir == 0) ? speed : -speed;
        std::cout << "  → 设定 dq=" << target << " rad/s\n";

        for (int i = 0; i < 30 && g_running; ++i) {
            motor.setVelocity(target, kd);
            MotorState s = motor.getState();
            printState("速度", s);
            usleep(100000);
        }
    }

    motor.brake();
    std::cout << "速度演示结束。\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 演示 3 —— 力矩控制
// ═══════════════════════════════════════════════════════════════════════════════

void demo_torque_control()
{
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout <<   "║  演示 3: 力矩控制（单电机独立模式）           ║\n";
    std::cout <<   "╚══════════════════════════════════════════════╝\n";

    MotorController motor(GPIO_PIN, SERIAL_PORT, MOTOR_ID);

    // 施加小力矩斜坡，感受电机推力（请根据实际负载调整）
    const float tau_max = 0.3f;   // N·m

    for (int i = 0; i < 60 && g_running; ++i) {
        // 斜坡上升 → 保持 → 斜坡下降
        float tau;
        if (i < 20)       tau = tau_max * (i / 20.0f);
        else if (i < 40)  tau = tau_max;
        else              tau = tau_max * ((60 - i) / 20.0f);

        motor.setTorque(tau);

        MotorState s = motor.getState();
        std::cout << "[" << "力矩" << "] "
                  << "指令力矩=" << std::fixed << std::setprecision(3) << tau
                  << "  反馈力矩=" << s.tau
                  << "  速度=" << s.dq
                  << "  错误=" << s.merror << std::endl;

        usleep(50000);   // 50 ms 周期（力矩环可以跑得更快）
    }

    motor.brake();
    std::cout << "力矩演示结束。\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 演示 4 —— 多电机总线
// ═══════════════════════════════════════════════════════════════════════════════

void demo_multimotor_bus()
{
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout <<   "║  演示 4: 多电机总线                            ║\n";
    std::cout <<   "╚══════════════════════════════════════════════╝\n";

    MotorBus bus(GPIO_PIN, SERIAL_PORT);

    // 注册两台电机
    bus.addMotor(0);
    bus.addMotor(1);
    std::cout << "总线上电机数: " << bus.motorCount()
              << "  减速比=" << bus.getGearRatio() << "\n";

    // 双电机同步位置控制，相位相反（"镜像"运动）
    const float kp = 0.03f;
    const float kd = 0.01f;

    for (int i = 0; i < 50 && g_running; ++i) {
        float target0 = (i / 25 % 2 == 0) ? 0.3f : -0.3f;
        float target1 = (i / 25 % 2 == 0) ? -0.3f : 0.3f;  // 反相

        bus.setPosition(0, target0, kp, kd);
        bus.setPosition(1, target1, kp, kd);

        bus.sendRecv();   // 一次事务完成两台电机收发

        if (i % 5 == 0) {
            MotorState s0 = bus.getState(0);
            MotorState s1 = bus.getState(1);
            std::cout << "  电机0: q=" << s0.q << "  dq=" << s0.dq
                      << "  |  电机1: q=" << s1.q << "  dq=" << s1.dq
                      << std::endl;
        }

        usleep(100000);
    }

    // 两台电机同时刹车
    bus.brake(0);
    bus.brake(1);
    bus.sendRecv();

    std::cout << "多电机演示结束。\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 主函数 —— 依次运行四个演示
// ═══════════════════════════════════════════════════════════════════════════════

int main()
{
    std::signal(SIGINT, sigint_handler);

    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║  GO-M8010-6 电机控制库 演示程序              ║\n";
    std::cout << "║  GPIO=" << GPIO_PIN << "  串口=" << SERIAL_PORT
              << "  电机ID=" << MOTOR_ID << "\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";

    std::cout << "\n[提示] 每个演示运行几秒，按 Ctrl+C 可跳到下一个演示。\n\n";

    // ── 演示 1: 位置控制 ──
    if (g_running) demo_position_control();

    // ── 演示 2: 速度控制 ──
    if (g_running) demo_velocity_control();

    // ── 演示 3: 力矩控制 ──
    if (g_running) demo_torque_control();

    // ── 演示 4: 多电机总线 ──
    if (g_running) demo_multimotor_bus();

    std::cout << "\n全部演示完成。\n";
    return 0;
}
