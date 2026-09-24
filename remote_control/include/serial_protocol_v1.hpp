#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace creeper {
namespace remote {

constexpr std::uint8_t kSof0 = 0xAA;
constexpr std::uint8_t kSof1 = 0x55;
constexpr std::uint8_t kProtocolVersion = 0x01;
constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kMaxPayloadSize = 48;
constexpr std::int16_t kCommandMinimum = -1000;
constexpr std::int16_t kCommandMaximum = 1000;

enum class MessageType : std::uint8_t {
    Hello = 0x01,
    ClaimControl = 0x02,
    ReleaseControl = 0x03,
    CommandVelocity = 0x10,
    Stop = 0x11,
    EmergencyStop = 0x12,
    EmergencyStopClear = 0x13,
    Heartbeat = 0x20,
    HelloAck = 0x81,
    ClaimAck = 0x82,
    CommandAck = 0x90,
    Status = 0x91,
    HeartbeatAck = 0xA0,
};

struct Frame {
    std::uint8_t version = kProtocolVersion;
    std::uint8_t message_type = 0;
    std::uint32_t sequence = 0;
    std::vector<std::uint8_t> payload;
};

struct CommandVelocity {
    std::int16_t vx_mm_s = 0;
    std::int16_t vy_mm_s = 0;
    std::int16_t yaw_mrad_s = 0;
};

struct ParserStats {
    std::uint64_t bytes_received = 0;
    std::uint64_t frames_ok = 0;
    std::uint64_t crc_errors = 0;
    std::uint64_t version_errors = 0;
    std::uint64_t length_errors = 0;
    std::uint64_t discarded_bytes = 0;
};

std::uint16_t crc16CcittFalse(const std::uint8_t* data, std::size_t size);

std::vector<std::uint8_t> encodeFrame(
    std::uint8_t message_type,
    std::uint32_t sequence,
    const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encodeCommandVelocityPayload(
    const CommandVelocity& command);

bool decodeCommandVelocityPayload(
    const std::vector<std::uint8_t>& payload,
    CommandVelocity& command);

bool commandVelocityInRange(const CommandVelocity& command);

// Returns true for a sequence that is newer under the frozen V1 wrap rule.
// Sequence zero is reserved and is never considered valid.
bool sequenceIsNew(std::uint32_t candidate, std::uint32_t last);

// Returns true for a known V1 message and writes its exact payload size.
// Unknown message types return false so the caller can respond UNSUPPORTED.
bool expectedPayloadSize(std::uint8_t message_type, std::size_t& size);

class StreamParser {
public:
    std::vector<Frame> feed(const std::uint8_t* data, std::size_t size);
    std::vector<Frame> feed(const std::vector<std::uint8_t>& data);

    void reset();
    const ParserStats& stats() const { return stats_; }
    std::size_t bufferedBytes() const { return buffer_.size(); }

private:
    std::vector<std::uint8_t> buffer_;
    ParserStats stats_;
};

}  // namespace remote
}  // namespace creeper
