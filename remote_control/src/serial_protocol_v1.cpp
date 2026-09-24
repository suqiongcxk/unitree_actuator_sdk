#include "serial_protocol_v1.hpp"

#include <stdexcept>

namespace creeper {
namespace remote {
namespace {

std::uint16_t readU16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           (static_cast<std::uint16_t>(data[1]) << 8);
}

std::uint32_t readU32(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

void appendU16(std::vector<std::uint8_t>& output, std::uint16_t value) {
    output.push_back(static_cast<std::uint8_t>(value & 0xFF));
    output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void appendU32(std::vector<std::uint8_t>& output, std::uint32_t value) {
    output.push_back(static_cast<std::uint8_t>(value & 0xFF));
    output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    output.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    output.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

}  // namespace

std::uint16_t crc16CcittFalse(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U)
                      ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::vector<std::uint8_t> encodeFrame(
    std::uint8_t message_type,
    std::uint32_t sequence,
    const std::vector<std::uint8_t>& payload) {
    if (payload.size() > kMaxPayloadSize) {
        throw std::invalid_argument("Protocol V1 payload exceeds 48 bytes");
    }

    std::vector<std::uint8_t> output;
    output.reserve(kHeaderSize + payload.size() + kCrcSize);
    output.push_back(kSof0);
    output.push_back(kSof1);
    output.push_back(kProtocolVersion);
    output.push_back(message_type);
    appendU16(output, static_cast<std::uint16_t>(payload.size()));
    appendU32(output, sequence);
    output.insert(output.end(), payload.begin(), payload.end());
    const std::uint16_t crc = crc16CcittFalse(output.data() + 2,
                                              output.size() - 2);
    appendU16(output, crc);
    return output;
}

std::vector<std::uint8_t> encodeCommandVelocityPayload(
    const CommandVelocity& command) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    appendU16(payload, static_cast<std::uint16_t>(command.vx_mm_s));
    appendU16(payload, static_cast<std::uint16_t>(command.vy_mm_s));
    appendU16(payload, static_cast<std::uint16_t>(command.yaw_mrad_s));
    return payload;
}

bool decodeCommandVelocityPayload(
    const std::vector<std::uint8_t>& payload,
    CommandVelocity& command) {
    if (payload.size() != 6) {
        return false;
    }
    command.vx_mm_s = static_cast<std::int16_t>(readU16(payload.data()));
    command.vy_mm_s = static_cast<std::int16_t>(readU16(payload.data() + 2));
    command.yaw_mrad_s = static_cast<std::int16_t>(readU16(payload.data() + 4));
    return true;
}

bool commandVelocityInRange(const CommandVelocity& command) {
    return command.vx_mm_s >= kCommandMinimum &&
           command.vx_mm_s <= kCommandMaximum &&
           command.vy_mm_s >= kCommandMinimum &&
           command.vy_mm_s <= kCommandMaximum &&
           command.yaw_mrad_s >= kCommandMinimum &&
           command.yaw_mrad_s <= kCommandMaximum;
}

bool sequenceIsNew(std::uint32_t candidate, std::uint32_t last) {
    if (candidate == 0) {
        return false;
    }
    if (last == 0) {
        return true;
    }
    const std::uint32_t delta = candidate - last;
    return delta >= 1U && delta <= 0x7FFFFFFFU;
}

bool expectedPayloadSize(std::uint8_t message_type, std::size_t& size) {
    switch (static_cast<MessageType>(message_type)) {
        case MessageType::Hello: size = 8; return true;
        case MessageType::ClaimControl: size = 0; return true;
        case MessageType::ReleaseControl: size = 0; return true;
        case MessageType::CommandVelocity: size = 6; return true;
        case MessageType::Stop: size = 0; return true;
        case MessageType::EmergencyStop: size = 0; return true;
        case MessageType::EmergencyStopClear: size = 4; return true;
        case MessageType::Heartbeat: size = 4; return true;
        case MessageType::HelloAck: size = 8; return true;
        case MessageType::ClaimAck: size = 6; return true;
        case MessageType::CommandAck: size = 9; return true;
        case MessageType::Status: size = 20; return true;
        case MessageType::HeartbeatAck: size = 8; return true;
    }
    return false;
}

std::vector<Frame> StreamParser::feed(const std::uint8_t* data,
                                      std::size_t size) {
    if (data != nullptr && size != 0) {
        buffer_.insert(buffer_.end(), data, data + size);
        stats_.bytes_received += size;
    }

    std::vector<Frame> frames;
    while (true) {
        auto sof = buffer_.end();
        for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
            if (*it == kSof0 && (it + 1) != buffer_.end() &&
                *(it + 1) == kSof1) {
                sof = it;
                break;
            }
        }
        if (sof == buffer_.end()) {
            // Preserve a trailing 0xAA because it may be the first byte of SOF.
            const bool preserve_aa = !buffer_.empty() && buffer_.back() == kSof0;
            const std::size_t discarded = buffer_.size() - (preserve_aa ? 1U : 0U);
            stats_.discarded_bytes += discarded;
            if (preserve_aa) {
                buffer_.assign(1, kSof0);
            } else {
                buffer_.clear();
            }
            break;
        }

        if (sof != buffer_.begin()) {
            const std::size_t discarded = static_cast<std::size_t>(sof - buffer_.begin());
            stats_.discarded_bytes += discarded;
            buffer_.erase(buffer_.begin(), sof);
        }
        if (buffer_.size() < kHeaderSize) {
            break;
        }
        if (buffer_[2] != kProtocolVersion) {
            ++stats_.version_errors;
            ++stats_.discarded_bytes;
            buffer_.erase(buffer_.begin());
            continue;
        }

        const std::size_t payload_size = readU16(buffer_.data() + 4);
        if (payload_size > kMaxPayloadSize) {
            ++stats_.length_errors;
            ++stats_.discarded_bytes;
            buffer_.erase(buffer_.begin());
            continue;
        }
        const std::size_t frame_size = kHeaderSize + payload_size + kCrcSize;
        if (buffer_.size() < frame_size) {
            break;
        }

        const std::uint16_t received_crc =
            readU16(buffer_.data() + kHeaderSize + payload_size);
        const std::uint16_t computed_crc =
            crc16CcittFalse(buffer_.data() + 2, kHeaderSize - 2 + payload_size);
        if (received_crc != computed_crc) {
            ++stats_.crc_errors;
            ++stats_.discarded_bytes;
            buffer_.erase(buffer_.begin());
            continue;
        }

        Frame frame;
        frame.version = buffer_[2];
        frame.message_type = buffer_[3];
        frame.sequence = readU32(buffer_.data() + 6);
        frame.payload.assign(buffer_.begin() + kHeaderSize,
                             buffer_.begin() + kHeaderSize + payload_size);
        frames.push_back(std::move(frame));
        ++stats_.frames_ok;
        buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
    }
    return frames;
}

std::vector<Frame> StreamParser::feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
}

void StreamParser::reset() {
    buffer_.clear();
    stats_ = ParserStats{};
}

}  // namespace remote
}  // namespace creeper
