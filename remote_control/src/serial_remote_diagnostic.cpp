#include "serial_remote_diagnostic.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace creeper {
namespace remote {
namespace {

std::uint32_t readU32(const std::vector<std::uint8_t>& payload,
                      std::size_t offset) {
    return static_cast<std::uint32_t>(payload[offset]) |
           (static_cast<std::uint32_t>(payload[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(payload[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(payload[offset + 3]) << 24);
}

void appendU32(std::vector<std::uint8_t>& payload, std::uint32_t value) {
    payload.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

std::int16_t clampCommand(std::int16_t value) {
    return std::max<std::int16_t>(kCommandMinimum,
                                  std::min<std::int16_t>(kCommandMaximum,
                                                         value));
}

}  // namespace

SerialRemoteDiagnosticSession::SerialRemoteDiagnosticSession(
    NonceGenerator nonce_generator)
    : nonce_generator_(std::move(nonce_generator)) {
    if (!nonce_generator_) {
        throw std::invalid_argument("nonce generator is required");
    }
}

std::uint32_t SerialRemoteDiagnosticSession::nextTransmitSequence() {
    const std::uint32_t sequence = next_transmit_sequence_;
    ++next_transmit_sequence_;
    if (next_transmit_sequence_ == 0) {
        next_transmit_sequence_ = 1;
    }
    return sequence;
}

std::vector<std::uint8_t> SerialRemoteDiagnosticSession::makeResponse(
    MessageType type, const std::vector<std::uint8_t>& payload) {
    return encodeFrame(static_cast<std::uint8_t>(type),
                       nextTransmitSequence(), payload);
}

void SerialRemoteDiagnosticSession::markValidFrame(
    std::uint64_t monotonic_ms) {
    has_valid_frame_ = true;
    timed_out_ = false;
    last_valid_frame_ms_ = monotonic_ms;
}

DiagnosticResult SerialRemoteDiagnosticSession::process(
    const Frame& frame,
    std::uint64_t monotonic_ms,
    std::uint32_t rk_uptime_ms) {
    DiagnosticResult result;
    result.request_sequence = frame.sequence;

    std::size_t expected_size = 0;
    if (!expectedPayloadSize(frame.message_type, expected_size)) {
        result.event = DiagnosticEvent::RejectedUnknownType;
        return result;
    }
    if (frame.payload.size() != expected_size) {
        result.event = DiagnosticEvent::RejectedLength;
        return result;
    }

    const MessageType type = static_cast<MessageType>(frame.message_type);

    // A valid HELLO creates a fresh session and therefore does not depend on
    // sequence history from a previous session. Sequence zero remains invalid.
    if (type == MessageType::Hello) {
        if (frame.sequence == 0) {
            result.event = DiagnosticEvent::RejectedStaleSequence;
            return result;
        }
        result.client_nonce = readU32(frame.payload, 0);
        result.peer_capabilities = readU32(frame.payload, 4);
        if (result.client_nonce == 0 ||
            (result.peer_capabilities & kProtocolV1Capabilities) !=
                kProtocolV1Capabilities) {
            result.event = DiagnosticEvent::RejectedHello;
            return result;
        }
        std::uint32_t nonce = nonce_generator_();
        if (nonce == 0) {
            nonce = 1;
        }
        session_nonce_ = nonce;
        session_active_ = true;
        last_receive_sequence_ = frame.sequence;
        next_transmit_sequence_ = 1;
        markValidFrame(monotonic_ms);

        std::vector<std::uint8_t> payload;
        payload.reserve(8);
        appendU32(payload, session_nonce_);
        appendU32(payload, kProtocolV1Capabilities);
        result.response = makeResponse(MessageType::HelloAck, payload);
        result.event = DiagnosticEvent::Hello;
        result.accepted = true;
        return result;
    }

    if (!session_active_) {
        // STOP/ESTOP are always safe to record, but without a HELLO session
        // they do not refresh the watchdog or advance sequence history.
        if (type == MessageType::Stop) {
            result.event = DiagnosticEvent::Stop;
        } else if (type == MessageType::EmergencyStop) {
            result.event = DiagnosticEvent::EmergencyStop;
            result.stale_estop_recorded = true;
        } else {
            result.event = DiagnosticEvent::RejectedNoSession;
        }
        return result;
    }

    const bool sequence_new =
        sequenceIsNew(frame.sequence, last_receive_sequence_);
    if (!sequence_new) {
        result.event = DiagnosticEvent::RejectedStaleSequence;
        // Frozen V1 requires ESTOP to be honored even when stale. This
        // diagnostic only records it and stays safe; it never touches motors.
        if (type == MessageType::EmergencyStop) {
            result.event = DiagnosticEvent::EmergencyStop;
            result.stale_estop_recorded = true;
        }
        return result;
    }

    last_receive_sequence_ = frame.sequence;
    markValidFrame(monotonic_ms);
    result.accepted = true;

    switch (type) {
        case MessageType::Heartbeat: {
            const std::uint32_t esp_uptime_ms = readU32(frame.payload, 0);
            std::vector<std::uint8_t> payload;
            payload.reserve(8);
            appendU32(payload, frame.sequence);
            appendU32(payload, esp_uptime_ms);
            result.response = makeResponse(MessageType::HeartbeatAck, payload);
            result.event = DiagnosticEvent::Heartbeat;
            break;
        }
        case MessageType::CommandVelocity: {
            result.event = DiagnosticEvent::CommandVelocity;
            if (!decodeCommandVelocityPayload(frame.payload,
                                               result.raw_command)) {
                result.accepted = false;
                result.event = DiagnosticEvent::RejectedLength;
                break;
            }
            result.limited_command = result.raw_command;
            result.limited_command.vx_mm_s =
                clampCommand(result.limited_command.vx_mm_s);
            result.limited_command.vy_mm_s =
                clampCommand(result.limited_command.vy_mm_s);
            result.limited_command.yaw_mrad_s =
                clampCommand(result.limited_command.yaw_mrad_s);
            break;
        }
        case MessageType::ClaimControl:
            result.event = DiagnosticEvent::ClaimControl;
            break;
        case MessageType::ReleaseControl:
            result.event = DiagnosticEvent::ReleaseControl;
            break;
        case MessageType::Stop:
            result.event = DiagnosticEvent::Stop;
            break;
        case MessageType::EmergencyStop:
            result.event = DiagnosticEvent::EmergencyStop;
            break;
        case MessageType::EmergencyStopClear:
            result.event = DiagnosticEvent::EmergencyStopClear;
            break;
        default:
            result.event = DiagnosticEvent::KnownButIgnored;
            break;
    }
    (void)rk_uptime_ms;
    return result;
}

bool SerialRemoteDiagnosticSession::updateTimeout(
    std::uint64_t monotonic_ms,
    std::uint64_t timeout_ms) {
    if (!has_valid_frame_ || timed_out_ ||
        monotonic_ms - last_valid_frame_ms_ <= timeout_ms) {
        return false;
    }
    timed_out_ = true;
    session_active_ = false;
    return true;
}

}  // namespace remote
}  // namespace creeper
