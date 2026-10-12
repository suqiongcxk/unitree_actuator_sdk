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
    if (!condition) throw std::runtime_error(message);
}

void appendU32(std::vector<std::uint8_t>& payload, std::uint32_t value) {
    payload.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
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
            throw std::runtime_error(std::string("PTY write: ") +
                                     std::strerror(errno));
        }
    }
}

Frame readFrame(int fd, StreamParser& parser, int timeout_ms) {
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
                if (!frames.empty()) return frames.front();
            }
        }
    }
    throw std::runtime_error("response timeout");
}

std::string readLogs(int fd, int timeout_ms) {
    std::string logs;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, POLLIN, 0};
        const int result = ::poll(&pfd, 1, 50);
        if (result > 0 && (pfd.revents & POLLIN)) {
            char buffer[1024];
            const ssize_t count = ::read(fd, buffer, sizeof(buffer));
            if (count > 0) logs.append(buffer, static_cast<std::size_t>(count));
        }
    }
    return logs;
}

void waitReady(int fd, std::string& logs) {
    for (int i = 0; i < 30; ++i) {
        logs += readLogs(fd, 100);
        if (logs.find("[READY]") != std::string::npos) return;
    }
    throw std::runtime_error("validator not ready: " + logs);
}

void requireAck(const Frame& response, MessageType type,
                std::uint8_t result) {
    require(response.message_type == static_cast<std::uint8_t>(type),
            "unexpected ACK type");
    require(response.payload.size() >= 5 && response.payload[4] == result,
            "unexpected ACK result");
}

void run(const std::string& validator_path) {
    int master = -1;
    int slave = -1;
    char slave_name[128]{};
    require(::openpty(&master, &slave, slave_name, nullptr, nullptr) == 0,
            "openpty failed");
    int log_pipe[2]{};
    require(::pipe(log_pipe) == 0, "pipe failed");

    const pid_t child = ::fork();
    require(child >= 0, "fork failed");
    if (child == 0) {
        ::close(master);
        ::close(slave);
        ::close(log_pipe[0]);
        ::dup2(log_pipe[1], STDOUT_FILENO);
        ::dup2(log_pipe[1], STDERR_FILENO);
        ::close(log_pipe[1]);
        ::execl(validator_path.c_str(), validator_path.c_str(),
                "--device", slave_name, static_cast<char*>(nullptr));
        _exit(127);
    }
    ::close(slave);
    ::close(log_pipe[1]);
    const int flags = ::fcntl(log_pipe[0], F_GETFL, 0);
    ::fcntl(log_pipe[0], F_SETFL, flags | O_NONBLOCK);

    std::string logs;
    try {
        waitReady(log_pipe[0], logs);
        StreamParser response_parser;

        std::vector<std::uint8_t> hello_payload;
        appendU32(hello_payload, 0x11223344U);
        appendU32(hello_payload, creeper::remote::kProtocolV1Capabilities);
        writeAll(master, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::Hello), 1,
            hello_payload));
        require(readFrame(master, response_parser, 1000).message_type ==
                    static_cast<std::uint8_t>(MessageType::HelloAck),
                "HELLO_ACK missing");

        const auto command_payload =
            creeper::remote::encodeCommandVelocityPayload({400, 0, 0});
        writeAll(master, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::CommandVelocity), 2,
            command_payload));
        requireAck(readFrame(master, response_parser, 1000),
                   MessageType::CommandAck, 2);  // NOT_OWNER

        writeAll(master, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::ClaimControl), 3, {}));
        const Frame claim_ack = readFrame(master, response_parser, 1000);
        requireAck(claim_ack, MessageType::ClaimAck, 0);
        require(claim_ack.payload.size() == 6 && claim_ack.payload[5] == 2,
                "CLAIM_ACK did not report SERIAL owner");

        writeAll(master, creeper::remote::encodeFrame(
            static_cast<std::uint8_t>(MessageType::CommandVelocity), 4,
            command_payload));
        requireAck(readFrame(master, response_parser, 1000),
                   MessageType::CommandAck, 0);

        ::usleep(350000);
        logs += readLogs(log_pipe[0], 300);
        require(logs.find("decision=NOT_OWNER") != std::string::npos,
                "pre-claim command was not visibly rejected");
        require(logs.find("decision=OK owner=SERIAL") != std::string::npos,
                "claim result log missing");
        require(logs.find("received_mks=(0.400,0.000,0.000)") !=
                    std::string::npos &&
                logs.find("SUBMITTED_TO_MANAGER") != std::string::npos,
                "converted manager submission log missing");
        require(logs.find("CMD_TIMEOUT >250ms") != std::string::npos,
                "command watchdog log missing");
        require(logs.find("NO_ROBOT_OUTPUT") != std::string::npos,
                "no-robot safety marker missing");
    } catch (...) {
        ::kill(child, SIGTERM);
        ::waitpid(child, nullptr, 0);
        ::close(master);
        ::close(log_pipe[0]);
        throw;
    }

    ::kill(child, SIGTERM);
    int status = 0;
    ::waitpid(child, &status, 0);
    ::close(master);
    ::close(log_pipe[0]);
    require(WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "validator did not exit cleanly");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2,
                "usage: test_serial_velocity_validator_pty <validator-path>");
        run(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << "test_serial_velocity_validator_pty: FAIL: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "test_serial_velocity_validator_pty: PASS\n";
    return EXIT_SUCCESS;
}
