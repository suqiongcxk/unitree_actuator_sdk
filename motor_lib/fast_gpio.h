#ifndef __FAST_GPIO_H
#define __FAST_GPIO_H

#include <cstdint>
#include <string>

/**
 * @brief 高速 GPIO 控制（基于 Linux GPIO v2 字符设备 /dev/gpiochipX）
 *
 * 通过 ioctl 直接操作 GPIO，切换延迟 ~1-10 µs，远优于 sysfs 的 100-200 µs。
 *
 * 使用示例:
 * @code
 *   FastGPIO gpio(0, 63);    // gpiochip0, line 63
 *   gpio.set(1);             // 拉高 (RS-485 TX)
 *   gpio.set(0);             // 拉低 (RS-485 RX)
 * @endcode
 */
class FastGPIO {
public:
    /**
     * @param chip_num GPIO 芯片编号（对应 /dev/gpiochipX 中的 X）
     * @param line     芯片内的引脚偏移量（全局 gpio 编号需查数据手册映射）
     */
    FastGPIO(int chip_num, int line);
    ~FastGPIO();

    /// 设置输出电平 (0=低, 1=高)
    void set(int value);

    /// 读取当前电平
    int get();

    bool isValid() const { return line_fd_ >= 0; }

private:
    int chip_fd_  = -1;   // /dev/gpiochipX 的文件描述符
    int line_fd_  = -1;   // 请求到的 GPIO line 文件描述符
    int line_;             // 行偏移
};

#endif  // __FAST_GPIO_H
