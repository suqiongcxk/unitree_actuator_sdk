#include "serial_velocity_control.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using creeper::remote::ControlOwner;
using creeper::remote::Frame;
using creeper::remote::MessageType;
using creeper::remote::ProtocolResult;
using creeper::remote::SerialVelocityControl;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void appendU32(std::vector<std::uint8_t>& payload, std::uint32_t value) {
    payload.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

Frame frame(MessageType type, std::uint32_t sequence,
            std::vector<std::uint8_t> payload = {}) {
    Frame value;
    value.message_type = static_cast<std::uint8_t>(type);
    value.sequence = sequence;
    value.payload = std::move(payload);
    return value;
}

Frame hello(std::uint32_t sequence) {
    std::vector<std::uint8_t> payload;
    appendU32(payload, 0xCAFEBABEU);
    appendU32(payload, creeper::remote::kProtocolV1Capabilities);
    return frame(MessageType::Hello, sequence, payload);
}

Frame command(std::uint32_t sequence, std::int16_t vx,
              std::int16_t vy, std::int16_t wz) {
    return frame(MessageType::CommandVelocity, sequence,
                 creeper::remote::encodeCommandVelocityPayload({vx, vy, wz}));
}

bool near(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1.0e-6f;
}

void testOwnershipUnitsAndWatchdogs() {
    std::uint32_t nonce = 0x12345678U;
    SerialVelocityControl control([&nonce]() { return nonce++; });

    auto result = control.processFrame(hello(1), 1000, 0);
    require(result.result == ProtocolResult::Ok &&
                control.owner() == ControlOwner::None,
            "HELLO must not grant control");

    result = control.processFrame(command(2, 400, -200, 500), 1010, 10);
    require(result.result == ProtocolResult::NotOwner &&
                !result.command_submitted,
            "CMD_VEL moved without CLAIM_CONTROL");
    require(near(control.commandStatus(1010).limited[0], 0.0f),
            "rejected serial command changed manager target");

    require(control.claimKeyboard(1020), "keyboard claim failed");
    require(control.submitKeyboard({{0.2f, 0.0f, 0.0f}}, 1030) !=
                VelocityCommandSubmitResult::INVALID,
            "keyboard submit failed");
    result = control.processFrame(
        frame(MessageType::ClaimControl, 3), 1040, 40);
    require(result.result == ProtocolResult::BadState &&
                result.owner == ControlOwner::Keyboard,
            "serial stole keyboard ownership");
    result = control.processFrame(command(4, 400, 0, 0), 1050, 50);
    require(result.result == ProtocolResult::NotOwner,
            "serial command overrode keyboard");

    require(control.releaseKeyboard(1060), "keyboard release failed");
    result = control.processFrame(
        frame(MessageType::ClaimControl, 5), 1070, 70);
    require(result.result == ProtocolResult::Ok &&
                result.owner == ControlOwner::Serial,
            "serial claim failed after keyboard release");

    result = control.processFrame(command(6, 400, -200, 500), 1080, 80);
    require(result.result == ProtocolResult::Ok && result.command_submitted,
            "owned serial command rejected");
    require(near(result.received_command[0], 0.4f) &&
                near(result.received_command[1], -0.2f) &&
                near(result.received_command[2], 0.5f),
            "mm/s or mrad/s conversion mismatch");
    require(near(result.manager_status.limited[0], 0.4f) &&
                near(result.manager_status.limited[1], -0.2f) &&
                near(result.manager_status.limited[2], 0.5f),
            "VelocityCommandManager did not receive converted command");

    std::vector<std::uint8_t> uptime;
    appendU32(uptime, 1234);
    control.processFrame(frame(MessageType::Heartbeat, 7, uptime), 1250, 250);
    const auto command_timeout = control.update(1331);
    require(command_timeout.command_timed_out,
            "HEARTBEAT incorrectly extended CMD_VEL timeout");
    require(near(control.commandStatus(1331).limited[0], 0.0f),
            "command timeout did not zero target");
    require(control.owner() == ControlOwner::Serial,
            "command timeout should retain serial owner");

    result = control.processFrame(command(8, 2000, 0, 0), 1340, 340);
    require(result.result == ProtocolResult::OutOfRange &&
                !result.command_submitted,
            "out-of-range Protocol V1 command accepted");

    result = control.processFrame(command(9, 300, 0, 0), 1350, 350);
    require(result.command_submitted, "second serial command rejected");
    result = control.processFrame(frame(MessageType::Stop, 10), 1360, 360);
    require(result.result == ProtocolResult::Ok &&
                near(result.manager_status.limited[0], 0.0f),
            "STOP did not zero target");
    require(control.owner() == ControlOwner::Serial,
            "STOP should retain serial owner");

    result = control.processFrame(
        frame(MessageType::ReleaseControl, 11), 1370, 370);
    require(result.result == ProtocolResult::Ok &&
                control.owner() == ControlOwner::None,
            "RELEASE_CONTROL failed");

    control.processFrame(frame(MessageType::ClaimControl, 12), 1380, 380);
    control.processFrame(command(13, 300, 0, 0), 1390, 390);
    const auto link_timeout = control.update(1991);
    require(link_timeout.link_timed_out &&
                control.owner() == ControlOwner::None &&
                near(control.commandStatus(1991).limited[0], 0.0f),
            "link timeout did not zero and release serial owner");
}

void testEstopAndExplicitClear() {
    std::uint32_t nonce = 0x0BADF00DU;
    SerialVelocityControl control([&nonce]() { return nonce++; });
    control.processFrame(hello(1), 1000, 0);
    control.processFrame(frame(MessageType::ClaimControl, 2), 1010, 10);
    control.processFrame(command(3, 300, 0, 0), 1020, 20);
    control.update(1020);
    control.update(1120);

    auto result = control.processFrame(
        frame(MessageType::EmergencyStop, 1), 1130, 130);
    require(control.estopLatched() &&
                control.owner() == ControlOwner::None &&
                near(control.commandStatus(1130).limited[0], 0.0f),
            "stale ESTOP did not latch and zero all input");

    result = control.processFrame(frame(MessageType::ClaimControl, 4),
                                  1140, 140);
    require(result.result == ProtocolResult::EstopLatched,
            "claim accepted while ESTOP latched");
    require(!control.claimKeyboard(1140),
            "keyboard claim accepted while ESTOP latched");

    std::vector<std::uint8_t> clear_payload;
    appendU32(clear_payload, control.sessionNonce());
    result = control.processFrame(
        frame(MessageType::EmergencyStopClear, 5, clear_payload),
        1150, 150);
    require(result.result == ProtocolResult::BadState &&
                control.estopLatched(),
            "ESTOP cleared without local authorization");

    control.update(1200);  // allow the real manager ramp to reach zero
    control.authorizeEstopClearOnce();
    result = control.processFrame(
        frame(MessageType::EmergencyStopClear, 6, clear_payload),
        1210, 210);
    require(result.result == ProtocolResult::Ok &&
                !control.estopLatched() &&
                control.owner() == ControlOwner::None &&
                near(control.commandStatus(1210).limited[0], 0.0f),
            "authorized ESTOP_CLEAR failed or restored stale command");
}

}  // namespace

int main() {
    try {
        testOwnershipUnitsAndWatchdogs();
        testEstopAndExplicitClear();
    } catch (const std::exception& error) {
        std::cerr << "test_serial_velocity_control: FAIL: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "test_serial_velocity_control: PASS\n";
    return EXIT_SUCCESS;
}
