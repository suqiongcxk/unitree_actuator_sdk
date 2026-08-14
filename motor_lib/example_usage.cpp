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
#include <time.h>
#include "ZeroPointCalibration.h"
// ── 硬件配置（请根据实际接线修改） ──────────────────────────────────────────
// 按 Leg1..Leg4 排列：UART6、UART4、UART7、UART0。
constexpr int    GPIO_PIN[4]    = {39,63,35,133};
constexpr const char* SERIAL_PORT[4] = {"/dev/ttyS6","/dev/ttyS4","/dev/ttyS7","/dev/ttyS0"};
constexpr unsigned short MOTOR_ID[4] = {0,4,8,1} ;
const int text_number = 0; 

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

    MotorController motor(GPIO_PIN[text_number], SERIAL_PORT[text_number], MOTOR_ID[text_number]);
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

    MotorController motor(GPIO_PIN[text_number], SERIAL_PORT[text_number], MOTOR_ID[text_number]);

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

    MotorController motor(GPIO_PIN[text_number], SERIAL_PORT[text_number], MOTOR_ID[text_number]);

    // 施加小力矩斜坡，感受电机推力（请根据实际负载调整）
    const float tau_max = 0.3f;   // N·m

    for (int i = 0; i < 60000 && g_running; ++i) {

        float tau;
        if (i < 20)       tau = tau_max * (i / 20.0f);
        else if (i < 40)  tau = tau_max;
        else              tau = tau_max * ((60 - i) / 20.0f);

        motor.setTorque(tau);
        MotorState s = motor.getState();
        usleep(50000);   // 50 ms 周期（力矩环可以跑得更快）
    }

    motor.brake();
    std::cout << "力矩演示结束。\n";
}

//校准时间
void timeCalibration ()
{
    for (int i = 0; i < 5 ; ++i) 
    {
        std::cout <<"1\n"<< std::endl;
        usleep(2000000); 
    }

}


int main()
{


    // demo_torque_control();
    ZeroPointCalibration();
    return 0;
}
