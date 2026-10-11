#include "serial_protocol_v1.hpp"
#include "serial_remote_diagnostic.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/random.h>
#include <termios.h>
#include <unistd.h>

namespace {

using creeper::remote::DiagnosticEvent;
using creeper::remote::DiagnosticResult;
using creeper::remote::ParserStats;
using creeper::remote::SerialRemoteDiagnosticSession;
using creeper::remote::StreamParser;

volatile std::sig_atomic_t g_stop_requested = 0;

void handleSignal(int) {
    g_stop_requested = 1;
}

std::uint64_t monotonicMilliseconds() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

std::uint32_t generateNonce() {
    std::uint32_t nonce = 0;
    const ssize_t count = ::getrandom(&nonce, sizeof(nonce), 0);
    if (count != static_cast<ssize_t>(sizeof(nonce)) || nonce == 0) {
        nonce = static_cast<std::uint32_t>(monotonicMilliseconds()) ^
                static_cast<std::uint32_t>(::getpid() * 0x9E3779B9U);
    }
    return nonce == 0 ? 1U : nonce;
}

int openSerial(const std::string& device) {
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK |
                                           O_CLOEXEC);
    if (fd < 0) {
        throw std::runtime_error("open " + device + ": " +
                                 std::strerror(errno));
    }

    termios options{};
    if (::tcgetattr(fd, &options) != 0) {
        const std::string error = std::strerror(errno);
        ::close(fd);
        throw std::runtime_error("tcgetattr: " + error);
    }
    ::cfmakeraw(&options);
    ::cfsetispeed(&options, B115200);
    ::cfsetospeed(&options, B115200);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &options) != 0) {
        const std::string error = std::strerror(errno);
        ::close(fd);
        throw std::runtime_error("tcsetattr: " + error);
    }
    ::tcflush(fd, TCIOFLUSH);
    return fd;
}

void writeAll(int fd, const std::vector<std::uint8_t>& bytes) {
    std::size_t offset = 0;
    while (offset < bytes.size() && !g_stop_requested) {
        const ssize_t count =
            ::write(fd, bytes.data() + offset, bytes.size() - offset);
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{fd, POLLOUT, 0};
            if (::poll(&pfd, 1, 100) >= 0) {
                continue;
            }
        }
        throw std::runtime_error(std::string("serial write: ") +
                                 std::strerror(errno));
    }
}

void logParserErrors(const ParserStats& before, const ParserStats& after) {
    if (after.crc_errors != before.crc_errors) {
        std::cout << "[RX] CRC_ERROR total=" << after.crc_errors << '\n';
    }
    if (after.version_errors != before.version_errors) {
        std::cout << "[RX] VERSION_ERROR total=" << after.version_errors
                  << '\n';
    }
    if (after.length_errors != before.length_errors) {
        std::cout << "[RX] LENGTH_ERROR total=" << after.length_errors << '\n';
    }
}

void logResult(const DiagnosticResult& result,
               const SerialRemoteDiagnosticSession& session) {
    switch (result.event) {
        case DiagnosticEvent::Hello:
            std::cout << "[RX] HELLO seq=" << result.request_sequence
                      << " client_nonce=" << result.client_nonce
                      << " capabilities=0x" << std::hex
                      << result.peer_capabilities << std::dec
                      << " accepted session_nonce=" << session.sessionNonce()
                      << '\n';
            std::cout << "[TX] HELLO_ACK session_nonce="
                      << session.sessionNonce() << '\n';
            break;
        case DiagnosticEvent::Heartbeat:
            std::cout << "[RX] HEARTBEAT seq=" << result.request_sequence
                      << " -> HEARTBEAT_ACK\n";
            break;
        case DiagnosticEvent::CommandVelocity:
            std::cout << "[RX] CMD_VEL seq=" << result.request_sequence
                      << " raw_mm_s=(" << result.raw_command.vx_mm_s << ','
                      << result.raw_command.vy_mm_s << ','
                      << result.raw_command.yaw_mrad_s << ") limited=("
                      << result.limited_command.vx_mm_s << ','
                      << result.limited_command.vy_mm_s << ','
                      << result.limited_command.yaw_mrad_s
                      << ") DIAGNOSTIC_ONLY_NOT_SUBMITTED\n";
            break;
        case DiagnosticEvent::ClaimControl:
            std::cout << "[RX] CLAIM_CONTROL seq=" << result.request_sequence
                      << " recognized; diagnostic mode grants no control\n";
            break;
        case DiagnosticEvent::ReleaseControl:
            std::cout << "[RX] RELEASE_CONTROL seq=" << result.request_sequence
                      << " recognized; diagnostic mode has no owner\n";
            break;
        case DiagnosticEvent::Stop:
            std::cout << "[RX] STOP seq=" << result.request_sequence
                      << " recorded; safe diagnostic state retained\n";
            break;
        case DiagnosticEvent::EmergencyStop:
            std::cout << "[RX] ESTOP seq=" << result.request_sequence
                      << (result.stale_estop_recorded ? " stale" : "")
                      << " recorded; safe diagnostic state retained\n";
            break;
        case DiagnosticEvent::EmergencyStopClear:
            std::cout << "[RX] ESTOP_CLEAR seq=" << result.request_sequence
                      << " recognized; diagnostic mode changes no safety state\n";
            break;
        case DiagnosticEvent::RejectedUnknownType:
            std::cout << "[RX] UNKNOWN_TYPE seq=" << result.request_sequence
                      << " rejected\n";
            break;
        case DiagnosticEvent::RejectedLength:
            std::cout << "[RX] PAYLOAD_LENGTH_ERROR seq="
                      << result.request_sequence << " rejected\n";
            break;
        case DiagnosticEvent::RejectedStaleSequence:
            std::cout << "[RX] STALE_SEQUENCE seq="
                      << result.request_sequence << " rejected\n";
            break;
        case DiagnosticEvent::RejectedNoSession:
            std::cout << "[RX] NO_SESSION seq=" << result.request_sequence
                      << " rejected; HELLO required\n";
            break;
        case DiagnosticEvent::RejectedHello:
            std::cout << "[RX] HELLO_INVALID seq="
                      << result.request_sequence
                      << " client_nonce=" << result.client_nonce
                      << " capabilities=0x" << std::hex
                      << result.peer_capabilities << std::dec
                      << " rejected\n";
            break;
        case DiagnosticEvent::KnownButIgnored:
            std::cout << "[RX] known message seq=" << result.request_sequence
                      << " ignored in phase-1 diagnostic\n";
            break;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);

    std::string device = "/dev/ttyS1";
    std::uint64_t timeout_ms = 600;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--device" && i + 1 < argc) {
            device = argv[++i];
        } else if (argument == "--timeout-ms" && i + 1 < argc) {
            timeout_ms = std::stoull(argv[++i]);
        } else if (argument == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--device /dev/ttyS1] [--timeout-ms 600]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            return 2;
        }
    }
    if (timeout_ms < 1) {
        std::cerr << "timeout must be positive\n";
        return 2;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    int fd = -1;
    try {
        fd = openSerial(device);
        StreamParser parser;
        SerialRemoteDiagnosticSession session(generateNonce);
        const std::uint64_t process_start_ms = monotonicMilliseconds();
        std::cout << "[READY] device=" << device
                  << " baud=115200 mode=8N1/raw/no-flow-control"
                  << " timeout_ms=" << timeout_ms
                  << " SAFE_DIAGNOSTIC_NO_ROBOT_CONTROL\n";

        while (!g_stop_requested) {
            pollfd pfd{fd, POLLIN, 0};
            const int poll_result = ::poll(&pfd, 1, 50);
            if (poll_result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("poll: ") +
                                         std::strerror(errno));
            }
            if (poll_result > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
                throw std::runtime_error("serial device disconnected");
            }
            if (poll_result > 0 && (pfd.revents & POLLIN)) {
                std::uint8_t bytes[256];
                const ssize_t count = ::read(fd, bytes, sizeof(bytes));
                if (count > 0) {
                    const ParserStats before = parser.stats();
                    const auto frames = parser.feed(
                        bytes, static_cast<std::size_t>(count));
                    logParserErrors(before, parser.stats());
                    for (const auto& frame : frames) {
                        const std::uint64_t now_ms = monotonicMilliseconds();
                        const auto result = session.process(
                            frame, now_ms,
                            static_cast<std::uint32_t>(now_ms - process_start_ms));
                        logResult(result, session);
                        if (!result.response.empty()) {
                            writeAll(fd, result.response);
                        }
                    }
                } else if (count < 0 && errno != EAGAIN &&
                           errno != EWOULDBLOCK && errno != EINTR) {
                    throw std::runtime_error(std::string("serial read: ") +
                                             std::strerror(errno));
                }
            }
            if (session.updateTimeout(monotonicMilliseconds(), timeout_ms)) {
                std::cout << "[WATCHDOG] TIMEOUT no valid frame for >"
                          << timeout_ms
                          << "ms; safe diagnostic state retained\n";
            }
        }
        std::cout << "[EXIT] signal received; no robot command was submitted\n";
        ::close(fd);
        return 0;
    } catch (const std::exception& error) {
        if (fd >= 0) {
            ::close(fd);
        }
        std::cerr << "[FATAL] " << error.what() << '\n';
        return 1;
    }
}
