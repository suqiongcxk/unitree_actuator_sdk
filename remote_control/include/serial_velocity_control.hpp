#pragma once

#include "serial_remote_diagnostic.hpp"
#include "velocity_command_manager.h"

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

namespace creeper {
namespace remote {

enum class ControlOwner : std::uint8_t {
    None = 0,
    Keyboard = 1,
    Serial = 2,
};

enum class ProtocolResult : std::uint8_t {
    Ok = 0,
    BadState = 1,
    NotOwner = 2,
    OutOfRange = 3,
    EstopLatched = 4,
    StaleSequence = 5,
    Unsupported = 6,
    InternalError = 7,
};

struct SerialControlResult {
    DiagnosticResult diagnostic;
    ProtocolResult result = ProtocolResult::Unsupported;
    ControlOwner owner = ControlOwner::None;
    bool command_submitted = false;
    bool estop_changed = false;
    bool estop_latched = false;
    std::array<float, 3> received_command{{0.0f, 0.0f, 0.0f}};
    VelocityCommandStatus manager_status{};
    std::vector<std::uint8_t> response;
};

struct SerialControlUpdate {
    std::array<float, 3> applied{{0.0f, 0.0f, 0.0f}};
    bool command_timed_out = false;
    bool link_timed_out = false;
};

// Input ownership and watchdog layer shared by the no-motor validator and the
// future robot_control integration. It owns the real VelocityCommandManager,
// but has no motor, GPIO, IMU or policy dependency.
class SerialVelocityControl {
public:
    using NonceGenerator = SerialRemoteDiagnosticSession::NonceGenerator;

    explicit SerialVelocityControl(
        NonceGenerator nonce_generator,
        const VelocityCommandConfig& command_config = VelocityCommandConfig{});

    SerialControlResult processFrame(const Frame& frame,
                                     std::uint64_t monotonic_ms,
                                     std::uint32_t rk_uptime_ms);
    SerialControlUpdate update(std::uint64_t monotonic_ms);

    bool claimKeyboard(std::uint64_t monotonic_ms);
    bool releaseKeyboard(std::uint64_t monotonic_ms);
    VelocityCommandSubmitResult submitKeyboard(
        const std::array<float, 3>& command, std::uint64_t monotonic_ms);

    // ESTOP_CLEAR additionally requires one explicit local authorization.
    // It is consumed on the next clear attempt, successful or not.
    void authorizeEstopClearOnce();

    ControlOwner owner() const;
    bool estopLatched() const;
    bool sessionActive() const;
    std::uint32_t sessionNonce() const;
    VelocityCommandStatus commandStatus(std::uint64_t monotonic_ms) const;

    static constexpr std::uint64_t kCommandTimeoutMs = 250;
    static constexpr std::uint64_t kLinkTimeoutMs = 600;

private:
    void submitZero(std::uint64_t monotonic_ms);
    std::vector<std::uint8_t> makeClaimAck(
        std::uint32_t request_sequence, ProtocolResult result);
    std::vector<std::uint8_t> makeCommandAck(
        std::uint32_t request_sequence, ProtocolResult result);
    bool zeroForEstopClear(std::uint64_t monotonic_ms) const;

    SerialRemoteDiagnosticSession session_;
    VelocityCommandManager command_manager_;
    mutable std::mutex mutex_;
    ControlOwner owner_ = ControlOwner::None;
    bool estop_latched_ = false;
    bool estop_clear_authorized_ = false;
    bool serial_command_active_ = false;
    std::uint64_t last_serial_command_ms_ = 0;
};

}  // namespace remote
}  // namespace creeper
