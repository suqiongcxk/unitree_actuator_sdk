#ifndef UNITREE_ACTUATOR_SDK_QUIET_SERIAL_RECV_H
#define UNITREE_ACTUATOR_SDK_QUIET_SERIAL_RECV_H

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include "crc/crc_ccitt.h"

namespace motor_io {

// 厂商 SDK 的 extract_data() 会为每个坏 CRC 向 stderr 打印一行警告。
// 在进入 SDK 前做完全相同的 CRC-CCITT 校验，坏帧仍由调用方计数。
inline bool hasValidGoM8010Crc(const uint8_t* frame, size_t length)
{
    constexpr size_t kFrameLength = 16;
    constexpr size_t kPayloadLength = 14;
    if (frame == nullptr || length != kFrameLength) return false;

    const uint16_t expected = static_cast<uint16_t>(frame[14])
                            | (static_cast<uint16_t>(frame[15]) << 8);
    return crc_ccitt(0, frame, kPayloadLength) == expected;
}

// 等价于厂商 SerialPort 的非阻塞接收流程，但不为每次超时/短帧打印警告。
// 返回真实接收长度；调用方仍可执行短帧计数、连续失败和反馈超时保护。
inline size_t quietSerialRecv(int fd, uint8_t* buffer, size_t expected_length)
{
    constexpr long kReceiveTimeoutUs = 20'000;  // 与 SerialPort 默认值一致
    if (fd < 0 || buffer == nullptr || expected_length == 0) return 0;

    auto readOnce = [&](uint8_t* destination, size_t length) -> size_t {
        int ready = -1;
        while (ready < 0) {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(fd, &read_set);
            timeval timeout{0, kReceiveTimeoutUs};
            ready = ::select(fd + 1, &read_set, nullptr, nullptr, &timeout);
            if (ready < 0 && errno != EINTR) break;
        }

        if (ready <= 0) {
            ::tcflush(fd, TCIOFLUSH);
            return 0;
        }

        ssize_t received;
        do {
            received = ::read(fd, destination, length);
        } while (received < 0 && errno == EINTR);
        if (received <= 0) {
            ::tcflush(fd, TCIOFLUSH);
            return 0;
        }
        return static_cast<size_t>(received);
    };

    const size_t first = readOnce(buffer, expected_length);
    if (first == 0 || first == expected_length) return first;

    const size_t second = readOnce(buffer + first, expected_length - first);
    const size_t total = first + second;
    if (total != expected_length) ::tcflush(fd, TCIOFLUSH);
    return total;
}

}  // namespace motor_io

#endif  // UNITREE_ACTUATOR_SDK_QUIET_SERIAL_RECV_H
