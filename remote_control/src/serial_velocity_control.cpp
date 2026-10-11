#include "serial_velocity_control.hpp"

#include <cmath>
#include <mutex>
#include <utility>

namespace creeper {
namespace remote {
namespace {

constexpr std::uint32_t kFaultEstopLatched = 1U << 2;

void appendU32(std::vector<std::uint8_t>& payload, std::uint32_t value) {
    payload.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

std::uint32_t readU32(const std::vector<std::uint8_t>& payload) {
    return static_cast<std::uint32_t>(payload[0]) |
           (static_cast<std::uint32_t>(payload[1]) << 8) |
           (static_cast<std::uint32_t>(payload[2]) << 16) |
           (static_cast<std::uint32_t>(payload[3]) << 24);
}

bool nearZero(float value) {
    return std::fabs(value) <= 1.0e-5f;
}

}  // namespace

SerialVelocityControl::SerialVelocityControl(
    NonceGenerator nonce_generator,
    const VelocityCommandConfig& command_config)
    : session_(std::move(nonce_generator)),
      command_manager_(command_config) {}

void SerialVelocityControl::submitZero(std::uint64_t monotonic_ms) {
    command_manager_.submit({{0.0f, 0.0f, 0.0f}},
                            monotonic_ms * 1'000'000ULL);
    serial_command_active_ = false;
}

std::vector<std::uint8_t> SerialVelocityControl::makeClaimAck(
    std::uint32_t request_sequence, ProtocolResult result) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    appendU32(payload, request_sequence);
    payload.push_back(static_cast<std::uint8_t>(result));
    payload.push_back(static_cast<std::uint8_t>(owner_));
    return session_.makeResponse(MessageType::ClaimAck, payload);
}

std::vector<std::uint8_t> SerialVelocityControl::makeCommandAck(
    std::uint32_t request_sequence, ProtocolResult result) {
    std::vector<std::uint8_t> payload;
    payload.reserve(9);
    appendU32(payload, request_sequence);
    payload.push_back(static_cast<std::uint8_t>(result));
    std::uint32_t fault_bits = 0;
    if (estop_latched_) fault_bits |= kFaultEstopLatched;
    appendU32(payload, fault_bits);
    return session_.makeResponse(MessageType::CommandAck, payload);
}

bool SerialVelocityControl::zeroForEstopClear(
    std::uint64_t monotonic_ms) const {
    const auto status = command_manager_.status(monotonic_ms * 1'000'000ULL);
    for (std::size_t i = 0; i < 3; ++i) {
        if (!nearZero(status.limited[i]) || !nearZero(status.applied[i])) {
            return false;
        }
    }
    return owner_ == ControlOwner::None;
}

SerialControlResult SerialVelocityControl::processFrame(
    const Frame& frame,
    std::uint64_t monotonic_ms,
    std::uint32_t rk_uptime_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    SerialControlResult output;
    output.diagnostic = session_.process(frame, monotonic_ms, rk_uptime_ms);
    output.response = output.diagnostic.response;
    output.owner = owner_;

    const auto event = output.diagnostic.event;
    if (event == DiagnosticEvent::Hello && output.diagnostic.accepted) {
        // A new serial session invalidates only a previous serial owner. It
        // must never steal or zero an active keyboard owner.
        if (owner_ == ControlOwner::Serial) {
            submitZero(monotonic_ms);
            owner_ = ControlOwner::None;
        }
        output.result = ProtocolResult::Ok;
        output.owner = owner_;
        output.estop_latched = estop_latched_;
        output.manager_status = commandStatus(monotonic_ms);
        return output;
    }

    if (event == DiagnosticEvent::EmergencyStop) {
        // Frozen V1 requires a valid ESTOP to latch even if its sequence is
        // stale. The parser has already guaranteed CRC/version/exact length.
        estop_latched_ = true;
        owner_ = ControlOwner::None;
        submitZero(monotonic_ms);
        output.estop_changed = true;
        output.result = ProtocolResult::Ok;
        if (session_.sessionActive()) {
            output.response = makeCommandAck(frame.sequence, output.result);
        }
        output.owner = owner_;
        output.estop_latched = estop_latched_;
        output.manager_status = commandStatus(monotonic_ms);
        return output;
    }

    if (!output.diagnostic.accepted) {
        if (event == DiagnosticEvent::RejectedStaleSequence) {
            output.result = ProtocolResult::StaleSequence;
        } else if (event == DiagnosticEvent::RejectedUnknownType) {
            output.result = ProtocolResult::Unsupported;
        } else {
            output.result = ProtocolResult::BadState;
        }
        output.estop_latched = estop_latched_;
        output.manager_status = commandStatus(monotonic_ms);
        return output;
    }

    switch (event) {
        case DiagnosticEvent::ClaimControl:
            if (estop_latched_) {
                output.result = ProtocolResult::EstopLatched;
            } else if (owner_ == ControlOwner::None ||
                       owner_ == ControlOwner::Serial) {
                owner_ = ControlOwner::Serial;
                submitZero(monotonic_ms);
                output.result = ProtocolResult::Ok;
            } else {
                output.result = ProtocolResult::BadState;
            }
            output.response = makeClaimAck(frame.sequence, output.result);
            break;

        case DiagnosticEvent::ReleaseControl:
            if (owner_ != ControlOwner::Serial) {
                output.result = ProtocolResult::NotOwner;
            } else {
                submitZero(monotonic_ms);
                owner_ = ControlOwner::None;
                output.result = ProtocolResult::Ok;
            }
            output.response = makeCommandAck(frame.sequence, output.result);
            break;

        case DiagnosticEvent::CommandVelocity: {
            output.received_command = {{
                static_cast<float>(output.diagnostic.raw_command.vx_mm_s) /
                    1000.0f,
                static_cast<float>(output.diagnostic.raw_command.vy_mm_s) /
                    1000.0f,
                static_cast<float>(output.diagnostic.raw_command.yaw_mrad_s) /
                    1000.0f,
            }};
            if (estop_latched_) {
                output.result = ProtocolResult::EstopLatched;
            } else if (owner_ != ControlOwner::Serial) {
                output.result = ProtocolResult::NotOwner;
            } else if (!commandVelocityInRange(
                           output.diagnostic.raw_command)) {
                output.result = ProtocolResult::OutOfRange;
            } else {
                const auto submit_result = command_manager_.submit(
                    output.received_command,
                    monotonic_ms * 1'000'000ULL,
                    kCommandTimeoutMs * 1'000'000ULL);
                if (submit_result == VelocityCommandSubmitResult::INVALID) {
                    output.result = ProtocolResult::InternalError;
                } else {
                    output.result = ProtocolResult::Ok;
                    output.command_submitted = true;
                    serial_command_active_ = true;
                    last_serial_command_ms_ = monotonic_ms;
                }
            }
            output.response = makeCommandAck(frame.sequence, output.result);
            break;
        }

        case DiagnosticEvent::Stop:
            if (owner_ != ControlOwner::Serial) {
                output.result = ProtocolResult::NotOwner;
            } else {
                submitZero(monotonic_ms);
                output.result = ProtocolResult::Ok;
            }
            output.response = makeCommandAck(frame.sequence, output.result);
            break;

        case DiagnosticEvent::EmergencyStopClear: {
            const bool authorized = estop_clear_authorized_;
            estop_clear_authorized_ = false;
            const bool nonce_matches =
                frame.payload.size() == 4 &&
                readU32(frame.payload) == session_.sessionNonce();
            if (!estop_latched_) {
                output.result = ProtocolResult::BadState;
            } else if (!authorized || !nonce_matches ||
                       !zeroForEstopClear(monotonic_ms)) {
                output.result = ProtocolResult::BadState;
            } else {
                estop_latched_ = false;
                submitZero(monotonic_ms);
                output.estop_changed = true;
                output.result = ProtocolResult::Ok;
            }
            output.response = makeCommandAck(frame.sequence, output.result);
            break;
        }

        default:
            output.result = ProtocolResult::Ok;
            break;
    }

    output.owner = owner_;
    output.estop_latched = estop_latched_;
    output.manager_status = commandStatus(monotonic_ms);
    return output;
}

SerialControlUpdate SerialVelocityControl::update(
    std::uint64_t monotonic_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    SerialControlUpdate output;
    if (serial_command_active_ &&
        monotonic_ms - last_serial_command_ms_ > kCommandTimeoutMs) {
        submitZero(monotonic_ms);
        output.command_timed_out = true;
    }
    if (session_.updateTimeout(monotonic_ms, kLinkTimeoutMs)) {
        output.link_timed_out = true;
        if (owner_ == ControlOwner::Serial) {
            submitZero(monotonic_ms);
            owner_ = ControlOwner::None;
        }
    }
    output.applied = command_manager_.update(monotonic_ms * 1'000'000ULL);
    return output;
}

bool SerialVelocityControl::claimKeyboard(std::uint64_t monotonic_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (estop_latched_ || owner_ != ControlOwner::None) return false;
    owner_ = ControlOwner::Keyboard;
    submitZero(monotonic_ms);
    return true;
}

bool SerialVelocityControl::releaseKeyboard(std::uint64_t monotonic_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (owner_ != ControlOwner::Keyboard) return false;
    submitZero(monotonic_ms);
    owner_ = ControlOwner::None;
    return true;
}

VelocityCommandSubmitResult SerialVelocityControl::submitKeyboard(
    const std::array<float, 3>& command,
    std::uint64_t monotonic_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (estop_latched_ || owner_ != ControlOwner::Keyboard) {
        return VelocityCommandSubmitResult::INVALID;
    }
    return command_manager_.submit(command,
                                   monotonic_ms * 1'000'000ULL);
}

VelocityCommandStatus SerialVelocityControl::commandStatus(
    std::uint64_t monotonic_ms) const {
    return command_manager_.status(monotonic_ms * 1'000'000ULL);
}

void SerialVelocityControl::authorizeEstopClearOnce() {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_clear_authorized_ = true;
}

ControlOwner SerialVelocityControl::owner() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return owner_;
}

bool SerialVelocityControl::estopLatched() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return estop_latched_;
}

bool SerialVelocityControl::sessionActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.sessionActive();
}

std::uint32_t SerialVelocityControl::sessionNonce() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.sessionNonce();
}

}  // namespace remote
}  // namespace creeper
