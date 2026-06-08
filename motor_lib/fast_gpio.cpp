#include "fast_gpio.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <stdexcept>
#include <iostream>

FastGPIO::FastGPIO(int chip_num, int line)
    : line_(line)
{
    // 1. 打开 GPIO 芯片设备
    std::string chip_path = "/dev/gpiochip" + std::to_string(chip_num);
    chip_fd_ = open(chip_path.c_str(), O_RDONLY);
    if (chip_fd_ < 0) {
        throw std::runtime_error("无法打开 " + chip_path);
    }

    // 2. 准备 GPIO v2 请求结构体
    gpio_v2_line_request req;
    std::memset(&req, 0, sizeof(req));

    req.offsets[0]    = static_cast<__u32>(line_);
    req.num_lines     = 1;
    req.config.flags  = GPIO_V2_LINE_FLAG_OUTPUT;

    // 设置初始值为低电平（RS-485 默认接收模式）
    req.config.num_attrs = 1;
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].mask    = 1;   // 仅 bit 0 有效
    req.config.attrs[0].attr.values = 0;   // 初始值 0 → 低电平

    snprintf(req.consumer, GPIO_MAX_NAME_SIZE, "motor_rs485_dir");

    // 3. 请求 GPIO line，内核返回 line fd
    if (ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        close(chip_fd_);
        chip_fd_ = -1;
        throw std::runtime_error("无法请求 GPIO line " + std::to_string(line_));
    }
    line_fd_ = req.fd;
}

FastGPIO::~FastGPIO()
{
    // 关闭 line fd 自动释放 GPIO 资源
    if (line_fd_ >= 0) close(line_fd_);
    if (chip_fd_ >= 0) close(chip_fd_);
}

void FastGPIO::set(int value)
{
    if (line_fd_ < 0) return;

    gpio_v2_line_values vals;
    std::memset(&vals, 0, sizeof(vals));
    vals.mask = 1;
    vals.bits = (value != 0) ? 1 : 0;

    ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
    // 注: ioctl 单次调用耗时约 1-5 µs（对比 sysfs 的 100-200 µs）
}

int FastGPIO::get()
{
    if (line_fd_ < 0) return 0;

    gpio_v2_line_values vals;
    std::memset(&vals, 0, sizeof(vals));
    vals.mask = 1;

    if (ioctl(line_fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals) < 0) {
        return 0;
    }
    return (vals.bits & 1) ? 1 : 0;
}
