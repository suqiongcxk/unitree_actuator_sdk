#include "fast_gpio.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <stdexcept>
#include <iostream>
#include <sstream>

FastGPIO::FastGPIO(int chip_num, int line)
    : line_(line)
{
    // 1. 打开 GPIO 芯片设备
    std::string chip_path = "/dev/gpiochip" + std::to_string(chip_num);
    chip_fd_ = open(chip_path.c_str(), O_RDONLY);
    if (chip_fd_ < 0) {
        throw std::runtime_error("无法打开 " + chip_path);
    }

    // 2. 准备 GPIO v2 请求（最简：只用 FLAG_OUTPUT，不用属性）
    gpio_v2_line_request req;
    std::memset(&req, 0, sizeof(req));

    req.offsets[0]    = static_cast<__u32>(line_);
    req.num_lines     = 1;
    req.config.flags  = GPIO_V2_LINE_FLAG_OUTPUT;
    req.config.num_attrs = 0;   // 不用 OUTPUT_VALUES 属性，内核可能不兼容

    snprintf(req.consumer, GPIO_MAX_NAME_SIZE, "motor485");

    // 3. 请求 GPIO line
    if (ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        int saved_errno = errno;
        close(chip_fd_);
        chip_fd_ = -1;
        std::ostringstream msg;
        msg << "无法请求 GPIO chip" << chip_num << " line" << line_
            << " (errno=" << saved_errno << ": " << strerror(saved_errno) << ")";
        throw std::runtime_error(msg.str());
    }
    line_fd_ = req.fd;

    // 4. 初始化为低电平（接收模式）
    set(0);
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
