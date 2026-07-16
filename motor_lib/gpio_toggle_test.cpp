/**
 * GPIO 翻转测试 — 验证 GPIO1_D7 (gpiochip1 line31) 物理电平
 *
 * 运行: sudo ./gpio_toggle_test
 * 用万用表直流电压档测 GPIO1_D7 引脚:
 *   - 1 秒高电平 (~3.3V)
 *   - 1 秒低电平 (~0V)
 *   - 交替循环, Ctrl+C 退出
 */
#include <iostream>
#include <csignal>
#include <unistd.h>
#include "fast_gpio.h"

static volatile bool g_running = true;
void sigint_handler(int) { g_running = false; }

int main() {
    std::signal(SIGINT, sigint_handler);

    try {
        // GPIO1_D7 = chip1, line31
        FastGPIO gpio(1, 31);
        std::cout << "GPIO1_D7 (gpiochip1 line31) 初始化成功\n";
        std::cout << "开始翻转: 1秒高 → 1秒低, Ctrl+C 退出\n";

        while (g_running) {
            std::cout << "HIGH (3.3V)" << std::endl;
            gpio.set(1);
            sleep(1);

            if (!g_running) break;

            std::cout << "LOW  (0V)" << std::endl;
            gpio.set(0);
            sleep(1);
        }

        std::cout << "\n测试结束\n";
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
