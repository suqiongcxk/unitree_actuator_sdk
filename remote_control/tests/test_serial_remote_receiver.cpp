#include "serial_protocol_v1.hpp"
#include "serial_remote_diagnostic.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {

using creeper::remote::Frame;
using creeper::remote::MessageType;
using creeper::remote::StreamParser;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void appendU32(std::vector<std::uint8_t>& payload, std::uint32_t value) {
    payload.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

std::uint32_t readU32(const std::vector<std::uint8_t>& payload,
                      std::size_t offset) {
    return static_cast<std::uint32_t>(payload[offset]) |
           (static_cast<std::uint32_t>(payload[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(payload[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(payload[offset + 3]) << 24);
}

void writeAll(int fd, const std::vector<std::uint8_t>& bytes) {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        const ssize_t count =
            ::write(fd, bytes.data() + offset, bytes.size() - offset);
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
        } else if (count < 0 && errno == EINTR) {
            continue;
        } else {
            throw std::runtime_error(std::string("PTY write failed: ") +
                                     std::strerror(errno));
        }
    }
}

std::string collectLogs(int fd, int timeout_ms) {
    std::string logs;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, POLLIN, 0};
        const int remaining = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        const int result = ::poll(&pfd, 1, remaining > 0 ? remaining : 1);
        if (result > 0 && (pfd.revents & POLLIN)) {
            char buffer[1024];
            const ssize_t count = ::read(fd, buffer, sizeof(buffer));
            if (count > 0) {
                logs.append(buffer, static_cast<std::size_t>(count));
            }
        } else if (result <= 0) {
            break;
        }
    }
    return logs;
}

void waitForReady(int log_fd, std::string& logs) {
    for (int attempt = 0; attempt < 30; ++attempt) {
        logs += collectLogs(log_fd, 100);
        if (logs.find("[READY]") != std::string::npos) {
            return;
        }
    }
    throw std::runtime_error("receiver did not become ready; logs=" + logs);
}

Frame readFrame(int fd, int timeout_ms) {
    StreamParser parser;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, POLLIN, 0};
        const int remaining = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        const int result = ::poll(&pfd, 1, remaining > 0 ? remaining : 1);
        if (result > 0 && (pfd.revents & POLLIN)) {
            std::uint8_t buffer[256];
            const ssize_t count = ::read(fd, buffer, sizeof(buffer));
            if (count > 0) {
                const auto frames = parser.feed(
                    buffer, static_cast<std::size_t>(count));
                if (!frames.empty()) {
                    return frames.front();
                }
            }
        }
    }
    throw std::runtime_error("timed out waiting for response frame");
}

void requireNoFrame(int fd, int timeout_ms) {
    pollfd pfd{fd, POLLIN, 0};
    const int result = ::poll(&pfd, 1, timeout_ms);
    require(result == 0, "unexpected response to rejected frame");
}

void testSessionStateMachine() {
    std::uint32_t nonce = 0x12345678U;
    creeper::remote::SerialRemoteDiagnosticSession session(
        [&nonce]() { return nonce++; });

    Frame prehello_heartbeat;
    prehello_heartbeat.message_type =
        static_cast<std::uint8_t>(MessageType::Heartbeat);
    prehello_heartbeat.sequence = 1;
    prehello_heartbeat.payload.assign(4, 0);
    require(!session.process(prehello_heartbeat, 900, 0).accepted,
            "HEARTBEAT without HELLO session was accepted");

    Frame hello;
    hello.message_type = static_cast<std::uint8_t>(MessageType::Hello);
    hello.sequence = 10;
    appendU32(hello.payload, 0xAABBCCDDU);
    appendU32(hello.payload, creeper::remote::kProtocolV1Capabilities);
    const auto hello_result = session.process(hello, 1000, 0);
    require(hello_result.accepted && !hello_result.response.empty(),
            "HELLO was not accepted");
    require(session.sessionNonce() == 0x12345678U,
            "session nonce mismatch");

    Frame heartbeat;
    heartbeat.message_type =
        static_cast<std::uint8_t>(MessageType::Heartbeat);
    heartbeat.sequence = 11;
    heartbeat.payload.assign(4, 0);
    require(session.process(heartbeat, 1100, 100).accepted,
            "HEARTBEAT was not accepted");
    require(!session.process(heartbeat, 1200, 200).accepted,
            "duplicate sequence was accepted");
    require(!session.updateTimeout(1700, 600),
            "timeout fired at exactly 600ms");
    require(session.updateTimeout(1701, 600) && session.timedOut(),
            "timeout did not fire after 600ms");
    heartbeat.sequence = 12;
    require(!session.process(heartbeat, 1750, 750).accepted,
            "HEARTBEAT after timeout was accepted without new HELLO");

    Frame stale_estop;
    stale_estop.message_type =
        static_cast<std::uint8_t>(MessageType::EmergencyStop);
    stale_estop.sequence = 1;
    const auto estop_result = session.process(stale_estop, 1800, 800);
    require(!estop_result.accepted && estop_result.stale_estop_recorded,
            "stale ESTOP was not recorded safely");
}

void testPseudoTerminal(const std::string& receiver_path) {
    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    require(::openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == 0,
            "openpty failed");

    int log_pipe[2]{};
    require(::pipe(log_pipe) == 0, "log pipe failed");

    const pid_t child = ::fork();
    require(child >= 0, "fork failed");
    if (child == 0) {
        ::close(master_fd);
        ::close(slave_fd);
        ::close(log_pipe[0]);
        ::dup2(log_pipe[1], STDOUT_FILENO);
        ::dup2(log_pipe[1], STDERR_FILENO);
        ::close(log_pipe[1]);
        ::execl(receiver_path.c_str(), receiver_path.c_str(),
                "--device", slave_name, "--timeout-ms", "600",
                static_cast<char*>(nullptr));
        _exit(127);
    }

    ::close(slave_fd);
    ::close(log_pipe[1]);
    const int flags = ::fcntl(log_pipe[0], F_GETFL, 0);
    ::fcntl(log_pipe[0], F_SETFL, flags | O_NONBLOCK);

    std::string logs;
    try {
        waitForReady(log_pipe[0], logs);

        std::vector<std::uint8_t> hello_payload;
        appendU32(hello_payload, 0xCAFEBABEU);
        appendU32(hello_payload, creeper::remote::kProtocolV1Capabilities);
        writeAll(master_fd, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Hello), 1,
            hello_payload));
        const Frame hello_ack = readFrame(master_fd, 1000);
        require(hello_ack.message_type ==
                    static_cast<std::uint8_t>(MessageType::HelloAck),
                "expected HELLO_ACK");
        require(hello_ack.sequence == 1 && hello_ack.payload.size() == 8,
                "HELLO_ACK schema mismatch");
        require(readU32(hello_ack.payload, 0) != 0,
                "HELLO_ACK nonce must be nonzero");
        require(readU32(hello_ack.payload, 4) ==
                    creeper::remote::kProtocolV1Capabilities,
                "HELLO_ACK capabilities mismatch");

        std::vector<std::uint8_t> heartbeat_payload;
        appendU32(heartbeat_payload, 0x01020304U);
        const auto heartbeat = creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Heartbeat), 2,
            heartbeat_payload);
        writeAll(master_fd, heartbeat);
        const Frame heartbeat_ack = readFrame(master_fd, 1000);
        require(heartbeat_ack.message_type ==
                    static_cast<std::uint8_t>(MessageType::HeartbeatAck),
                "expected HEARTBEAT_ACK");
        require(heartbeat_ack.sequence == 2 &&
                    readU32(heartbeat_ack.payload, 0) == 2 &&
                    readU32(heartbeat_ack.payload, 4) == 0x01020304U,
                "HEARTBEAT_ACK payload or TX sequence mismatch");

        writeAll(master_fd, heartbeat);
        requireNoFrame(master_fd, 100);

        auto corrupt = creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Heartbeat), 3,
            heartbeat_payload);
        corrupt.back() ^= 0x80U;
        writeAll(master_fd, corrupt);
        requireNoFrame(master_fd, 100);

        writeAll(master_fd, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Heartbeat), 4, {}));
        requireNoFrame(master_fd, 100);

        writeAll(master_fd, creeper::remote::encodeFrame(0xEE, 5, {}));
        requireNoFrame(master_fd, 100);

        const creeper::remote::CommandVelocity raw{1500, -1500, 500};
        writeAll(master_fd, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::CommandVelocity), 6,
            creeper::remote::encodeCommandVelocityPayload(raw)));
        writeAll(master_fd, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Stop), 7, {}));
        writeAll(master_fd, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::EmergencyStop), 1, {}));

        ::usleep(750000);
        logs += collectLogs(log_pipe[0], 200);
        require(logs.find("HELLO") != std::string::npos,
                "HELLO log missing");
        require(logs.find("HELLO_ACK") != std::string::npos,
                "HELLO_ACK log missing");
        require(logs.find("HEARTBEAT") != std::string::npos,
                "HEARTBEAT log missing");
        require(logs.find("CRC_ERROR") != std::string::npos,
                "CRC error log missing");
        require(logs.find("PAYLOAD_LENGTH_ERROR") != std::string::npos,
                "payload length error log missing");
        require(logs.find("STALE_SEQUENCE") != std::string::npos,
                "stale sequence log missing");
        require(logs.find("CMD_VEL") != std::string::npos &&
                    logs.find("limited=(1000,-1000,500)") !=
                        std::string::npos &&
                    logs.find("DIAGNOSTIC_ONLY_NOT_SUBMITTED") !=
                        std::string::npos,
                "CMD_VEL diagnostic clamp log missing");
        require(logs.find("STOP") != std::string::npos,
                "STOP log missing");
        require(logs.find("ESTOP") != std::string::npos,
                "ESTOP log missing");
        require(logs.find("TIMEOUT") != std::string::npos,
                "600ms timeout log missing");
    } catch (...) {
        ::kill(child, SIGTERM);
        ::waitpid(child, nullptr, 0);
        ::close(master_fd);
        ::close(log_pipe[0]);
        throw;
    }

    ::kill(child, SIGTERM);
    int status = 0;
    ::waitpid(child, &status, 0);
    ::close(master_fd);
    ::close(log_pipe[0]);
    require(WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "receiver did not exit cleanly");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2,
                "usage: test_serial_remote_receiver <receiver-path>");
        testSessionStateMachine();
        testPseudoTerminal(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << "test_serial_remote_receiver: FAIL: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "test_serial_remote_receiver: PASS\n";
    return EXIT_SUCCESS;
}
