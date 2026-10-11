#include "serial_protocol_v1.hpp"
#include "serial_velocity_control.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/random.h>
#include <termios.h>
#include <unistd.h>

namespace {

using creeper::remote::ControlOwner;
using creeper::remote::DiagnosticEvent;
using creeper::remote::ParserStats;
using creeper::remote::ProtocolResult;
using creeper::remote::SerialControlResult;
using creeper::remote::SerialVelocityControl;
using creeper::remote::StreamParser;

volatile std::sig_atomic_t g_stop_requested = 0;

void handleSignal(int) { g_stop_requested = 1; }

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
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else if (count < 0 &&
                   (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{fd, POLLOUT, 0};
            if (::poll(&pfd, 1, 100) >= 0) continue;
            throw std::runtime_error("serial write poll failed");
        } else {
            throw std::runtime_error(std::string("serial write: ") +
                                     std::strerror(errno));
        }
    }
}

const char* ownerName(ControlOwner owner) {
    switch (owner) {
        case ControlOwner::None: return "NONE";
        case ControlOwner::Keyboard: return "KEYBOARD";
        case ControlOwner::Serial: return "SERIAL";
    }
    return "UNKNOWN";
}

const char* resultName(ProtocolResult result) {
    switch (result) {
        case ProtocolResult::Ok: return "OK";
        case ProtocolResult::BadState: return "BAD_STATE";
        case ProtocolResult::NotOwner: return "NOT_OWNER";
        case ProtocolResult::OutOfRange: return "OUT_OF_RANGE";
        case ProtocolResult::EstopLatched: return "ESTOP_LATCHED";
        case ProtocolResult::StaleSequence: return "STALE_SEQUENCE";
        case ProtocolResult::Unsupported: return "UNSUPPORTED";
        case ProtocolResult::InternalError: return "INTERNAL_ERROR";
    }
    return "UNKNOWN";
}

const char* eventName(DiagnosticEvent event) {
    switch (event) {
        case DiagnosticEvent::Hello: return "HELLO";
        case DiagnosticEvent::Heartbeat: return "HEARTBEAT";
        case DiagnosticEvent::CommandVelocity: return "CMD_VEL";
        case DiagnosticEvent::ClaimControl: return "CLAIM_CONTROL";
        case DiagnosticEvent::ReleaseControl: return "RELEASE_CONTROL";
        case DiagnosticEvent::Stop: return "STOP";
        case DiagnosticEvent::EmergencyStop: return "ESTOP";
        case DiagnosticEvent::EmergencyStopClear: return "ESTOP_CLEAR";
        case DiagnosticEvent::KnownButIgnored: return "KNOWN_IGNORED";
        case DiagnosticEvent::RejectedUnknownType: return "UNKNOWN_TYPE";
        case DiagnosticEvent::RejectedLength: return "BAD_LENGTH";
        case DiagnosticEvent::RejectedStaleSequence: return "STALE_SEQUENCE";
        case DiagnosticEvent::RejectedNoSession: return "NO_SESSION";
        case DiagnosticEvent::RejectedHello: return "BAD_HELLO";
    }
    return "UNKNOWN";
}

void printControlResult(const SerialControlResult& result) {
    std::cout << "[CONTROL] rx=" << eventName(result.diagnostic.event)
              << " seq=" << result.diagnostic.request_sequence
              << " decision=" << resultName(result.result)
              << " owner=" << ownerName(result.owner)
              << " estop=" << (result.estop_latched ? 1 : 0);
    if (result.diagnostic.event == DiagnosticEvent::CommandVelocity) {
        std::cout << std::fixed << std::setprecision(3)
                  << " received_mks=(" << result.received_command[0] << ','
                  << result.received_command[1] << ','
                  << result.received_command[2] << ") accepted_target=("
                  << result.manager_status.limited[0] << ','
                  << result.manager_status.limited[1] << ','
                  << result.manager_status.limited[2] << ')'
                  << std::defaultfloat;
    }
    std::cout << (result.command_submitted ? " SUBMITTED_TO_MANAGER" : "")
              << " NO_ROBOT_OUTPUT\n";
}

void logParserErrors(const ParserStats& before, const ParserStats& after) {
    if (after.crc_errors != before.crc_errors)
        std::cout << "[RX] CRC_ERROR total=" << after.crc_errors << '\n';
    if (after.version_errors != before.version_errors)
        std::cout << "[RX] VERSION_ERROR total=" << after.version_errors << '\n';
    if (after.length_errors != before.length_errors)
        std::cout << "[RX] LENGTH_ERROR total=" << after.length_errors << '\n';
}

bool changed(const std::array<float, 3>& a,
             const std::array<float, 3>& b) {
    return a != b;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);
    std::string device = "/dev/ttyS1";
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--device" && i + 1 < argc) {
            device = argv[++i];
        } else if (argument == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--device /dev/ttyS1]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            return 2;
        }
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    int fd = -1;
    try {
        fd = openSerial(device);
        StreamParser parser;
        SerialVelocityControl control(generateNonce);
        const std::uint64_t start_ms = monotonicMilliseconds();
        std::uint64_t last_effective_log_ms = 0;
        std::array<float, 3> previous_applied{{0.0f, 0.0f, 0.0f}};
        std::cout << "[READY] device=" << device
                  << " 115200/8N1/no-flow-control"
                  << " SAFE_VALIDATOR_NO_ROBOT_CONTROL\n"
                  << "[INFO] Serial motion remains locked until valid "
                     "CLAIM_CONTROL. Local ESTOP clear requires typing "
                     "authorize-clear then sending ESTOP_CLEAR.\n";

        while (!g_stop_requested) {
            pollfd fds[2]{{fd, POLLIN, 0}, {STDIN_FILENO, POLLIN, 0}};
            const int poll_result = ::poll(fds, 2, 20);
            if (poll_result < 0 && errno != EINTR) {
                throw std::runtime_error(std::string("poll: ") +
                                         std::strerror(errno));
            }
            if (poll_result > 0 &&
                (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))) {
                throw std::runtime_error("serial device disconnected");
            }
            if (poll_result > 0 && (fds[0].revents & POLLIN)) {
                std::uint8_t bytes[256];
                const ssize_t count = ::read(fd, bytes, sizeof(bytes));
                if (count > 0) {
                    const ParserStats before = parser.stats();
                    const auto frames = parser.feed(
                        bytes, static_cast<std::size_t>(count));
                    logParserErrors(before, parser.stats());
                    for (const auto& frame : frames) {
                        const std::uint64_t now_ms = monotonicMilliseconds();
                        auto result = control.processFrame(
                            frame, now_ms,
                            static_cast<std::uint32_t>(now_ms - start_ms));
                        printControlResult(result);
                        if (!result.response.empty()) {
                            writeAll(fd, result.response);
                        }
                    }
                }
            }
            if (poll_result > 0 && (fds[1].revents & POLLIN)) {
                std::string line;
                if (std::getline(std::cin, line) && line == "authorize-clear") {
                    control.authorizeEstopClearOnce();
                    std::cout << "[LOCAL] one ESTOP_CLEAR attempt authorized\n";
                } else if (!line.empty()) {
                    std::cout << "[LOCAL] unknown command; only authorize-clear"
                              << " is accepted\n";
                }
            }

            const std::uint64_t now_ms = monotonicMilliseconds();
            const auto update = control.update(now_ms);
            if (update.command_timed_out) {
                std::cout << "[WATCHDOG] CMD_TIMEOUT >250ms; target zeroed\n";
            }
            if (update.link_timed_out) {
                std::cout << "[WATCHDOG] LINK_TIMEOUT >600ms; serial owner "
                             "released and target zeroed\n";
            }
            if (changed(update.applied, previous_applied) &&
                now_ms - last_effective_log_ms >= 100) {
                std::cout << std::fixed << std::setprecision(3)
                          << "[EFFECTIVE] vx/vy/wz=(" << update.applied[0]
                          << ',' << update.applied[1] << ','
                          << update.applied[2]
                          << ") owner=" << ownerName(control.owner())
                          << " NO_ROBOT_OUTPUT\n" << std::defaultfloat;
                previous_applied = update.applied;
                last_effective_log_ms = now_ms;
            }
        }
        std::cout << "[EXIT] no robot command was emitted\n";
        ::close(fd);
        return 0;
    } catch (const std::exception& error) {
        if (fd >= 0) ::close(fd);
        std::cerr << "[FATAL] " << error.what() << '\n';
        return 1;
    }
}
