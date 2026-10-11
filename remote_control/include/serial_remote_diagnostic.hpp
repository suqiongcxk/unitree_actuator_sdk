#pragma once

#include "serial_protocol_v1.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace creeper {
namespace remote {

constexpr std::uint32_t kProtocolV1Capabilities = 0x0000001FU;

enum class DiagnosticEvent {
    Hello,
    Heartbeat,
    CommandVelocity,
    ClaimControl,
    ReleaseControl,
    Stop,
    EmergencyStop,
    EmergencyStopClear,
    KnownButIgnored,
    RejectedUnknownType,
    RejectedLength,
    RejectedStaleSequence,
    RejectedNoSession,
    RejectedHello,
};

struct DiagnosticResult {
    DiagnosticEvent event = DiagnosticEvent::KnownButIgnored;
    bool accepted = false;
    bool stale_estop_recorded = false;
    std::uint32_t request_sequence = 0;
    std::uint32_t client_nonce = 0;
    std::uint32_t peer_capabilities = 0;
    CommandVelocity raw_command{};
    CommandVelocity limited_command{};
    std::vector<std::uint8_t> response;
};

// Protocol V1 handshake-only diagnostic state machine. It deliberately has no
// dependency on robot_control, VelocityCommandManager, GPIO or motor drivers.
class SerialRemoteDiagnosticSession {
public:
    using NonceGenerator = std::function<std::uint32_t()>;

    explicit SerialRemoteDiagnosticSession(NonceGenerator nonce_generator);

    DiagnosticResult process(const Frame& frame,
                             std::uint64_t monotonic_ms,
                             std::uint32_t rk_uptime_ms);

    bool updateTimeout(std::uint64_t monotonic_ms,
                       std::uint64_t timeout_ms = 600);

    // Encode a server response using the RK3588->ESP32 sequence space.
    std::vector<std::uint8_t> makeResponse(
        MessageType type, const std::vector<std::uint8_t>& payload);

    bool sessionActive() const { return session_active_; }
    bool timedOut() const { return timed_out_; }
    std::uint32_t sessionNonce() const { return session_nonce_; }
    std::uint32_t lastReceiveSequence() const { return last_receive_sequence_; }

private:
    std::uint32_t nextTransmitSequence();
    void markValidFrame(std::uint64_t monotonic_ms);

    NonceGenerator nonce_generator_;
    bool session_active_ = false;
    bool has_valid_frame_ = false;
    bool timed_out_ = false;
    std::uint32_t session_nonce_ = 0;
    std::uint32_t last_receive_sequence_ = 0;
    std::uint32_t next_transmit_sequence_ = 1;
    std::uint64_t last_valid_frame_ms_ = 0;
};

}  // namespace remote
}  // namespace creeper
