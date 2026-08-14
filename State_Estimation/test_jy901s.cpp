/**
 * @example test_jy901s.cpp
 * @brief JY901S IMU 传感器测试程序
 *
 * 接线: Orange Pi 5 Pro 40-pin 排针
 *   pin 5  (GPIO1_D2, I2C1_M4_SCL) → JY901S SCL
 *   pin 3  (GPIO1_D3, I2C1_M4_SDA) → JY901S SDA
 *   pin 1  (3.3V)          → JY901S VCC
 *   pin 9  (GND)           → JY901S GND
 *
 * 编译:
 *   cd build && cmake .. && make test_jy901s
 *
 * 运行:
 *   sudo ./test_jy901s
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include "jy901s.h"

static volatile bool g_running = true;
void sigint_handler(int) { g_running = false; }

int main()
{
    signal(SIGINT, sigint_handler);

    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║     JY901S IMU 传感器测试                     ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    // ── 1. 创建并初始化 ──
    JY901S imu("/dev/i2c-1");
    JY901S_Status st = imu.init(200);  // 200 Hz
    if (st != JY901S_Status::OK) {
        std::cerr << "初始化失败! 错误码=" << static_cast<int>(st) << "\n";
        return 1;
    }

    std::cout << "\n开始读取数据 (Ctrl+C 退出)...\n\n";

    // 打印表头
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "   Roll     Pitch    Yaw      AccX     AccY     AccZ     GyroX   GyroY   GyroZ\n";
    std::cout << "   (deg)    (deg)    (deg)    (m/s2)   (m/s2)   (m/s2)   (rad/s) (rad/s) (rad/s)\n";
    std::cout << "──────────────────────────────────────────────────────────────────────────────\n";

    int loop_count = 0;

    // ── 2. 主循环 ──
    while (g_running) {
        JY901S_AngleData angles;
        JY901S_ACCData   acc;
        JY901S_GyroData  gyro;

        // 批量读取（2 次 I2C 事务读取全部 3 组数据）
        st = imu.readAll(angles, acc, gyro);
        if (st != JY901S_Status::OK) {
            std::cerr << "读取失败! 错误码=" << static_cast<int>(st) << "\n";
            usleep(5000);
            continue;
        }

        std::cout << "\r"
                  << std::setw(7) << angles.roll  << " "
                  << std::setw(7) << angles.pitch << " "
                  << std::setw(7) << angles.yaw   << "  "
                  << std::setw(7) << acc.x  << " "
                  << std::setw(7) << acc.y  << " "
                  << std::setw(7) << acc.z  << "  "
                  << std::setw(6) << gyro.x << " "
                  << std::setw(6) << gyro.y << " "
                  << std::setw(6) << gyro.z
                  << std::flush;

        // 每秒打印一次详细信息
        if (++loop_count % 200 == 0) {
            std::cout << "\n\n── 详细数据 (第 " << loop_count / 200 << " 秒) ──\n";
            std::cout << "  角度:  Roll=" << angles.roll
                      << "°  Pitch=" << angles.pitch
                      << "°  Yaw=" << angles.yaw << "°\n";
            std::cout << "  加速度: X=" << acc.x
                      << "  Y=" << acc.y
                      << "  Z=" << acc.z << " m/s²\n";
            std::cout << "  角速度: X=" << gyro.x
                      << "  Y=" << gyro.y
                      << "  Z=" << gyro.z << " rad/s\n\n";

            // 也读一下四元数（调试用）
            JY901S_Quaternion quat;
            if (imu.readQuaternion(quat) == JY901S_Status::OK) {
                const float quat_norm = std::sqrt(
                    quat.q0 * quat.q0 + quat.q1 * quat.q1 +
                    quat.q2 * quat.q2 + quat.q3 * quat.q3);
                std::cout << "  四元数: w=" << quat.q0
                          << "  x=" << quat.q1
                          << "  y=" << quat.q2
                          << "  z=" << quat.q3
                          << "  |q|=" << quat_norm << "\n\n";
            }
        }

        usleep(5000);  // 200 Hz = 5ms 间隔
    }

    std::cout << "\n\n[OK] 测试结束.\n";
    imu.close();
    return 0;
}
