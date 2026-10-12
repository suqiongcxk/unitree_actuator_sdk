#include "serial_protocol_v1.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using creeper::remote::CommandVelocity;
using creeper::remote::Frame;
using creeper::remote::MessageType;
using creeper::remote::StreamParser;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testGoldenVector() {
    const CommandVelocity command{500, 0, -500};
    const auto encoded = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::CommandVelocity), 1,
        creeper::remote::encodeCommandVelocityPayload(command));
    const std::vector<std::uint8_t> expected{
        0xAA, 0x55, 0x01, 0x10, 0x06, 0x00, 0x01, 0x00, 0x00,
        0x00, 0xF4, 0x01, 0x00, 0x00, 0x0C, 0xFE, 0x34, 0xD6};
    require(encoded == expected, "CRC golden vector mismatch");
}

void testEverySplitPoint() {
    const CommandVelocity command{-1000, 321, 1000};
    const auto encoded = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::CommandVelocity), 42,
        creeper::remote::encodeCommandVelocityPayload(command));
    for (std::size_t split = 0; split <= encoded.size(); ++split) {
        StreamParser parser;
        const auto first = parser.feed(encoded.data(), split);
        const auto second = parser.feed(encoded.data() + split,
                                        encoded.size() - split);
        require(first.size() + second.size() == 1,
                "split frame did not produce exactly one frame");
        const Frame& frame = first.empty() ? second.front() : first.front();
        require(frame.sequence == 42, "split frame sequence mismatch");
        CommandVelocity decoded;
        require(creeper::remote::decodeCommandVelocityPayload(frame.payload,
                                                               decoded),
                "velocity payload decode failed");
        require(decoded.vx_mm_s == -1000 && decoded.vy_mm_s == 321 &&
                    decoded.yaw_mrad_s == 1000,
                "signed velocity payload mismatch");
        require(creeper::remote::commandVelocityInRange(decoded),
                "boundary velocity should be in range");
    }
}

void testNoiseStickyFramesAndTrailingSof() {
    const auto stop = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::Stop), 7, {});
    const auto heartbeat = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::Heartbeat), 8,
        {0x78, 0x56, 0x34, 0x12});
    std::vector<std::uint8_t> bytes{0x13, 0x37, 0xAA};
    bytes.insert(bytes.end(), stop.begin(), stop.end());
    bytes.insert(bytes.end(), heartbeat.begin(), heartbeat.end());
    bytes.push_back(0xAA);

    StreamParser parser;
    const auto frames = parser.feed(bytes);
    require(frames.size() == 2, "sticky frames were not both parsed");
    require(frames[0].sequence == 7 && frames[1].sequence == 8,
            "sticky frame ordering mismatch");
    require(parser.bufferedBytes() == 1,
            "trailing SOF byte must be preserved");
    require(parser.stats().discarded_bytes == 3,
            "noise discard counter mismatch");
}

void testCorruptFrameRecovery() {
    auto corrupt = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::CommandVelocity), 9,
        {1, 2, 3, 4, 5, 6});
    corrupt[10] ^= 0x80;
    const auto estop = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::EmergencyStop), 10, {});
    corrupt.insert(corrupt.end(), estop.begin(), estop.end());

    StreamParser parser;
    const auto frames = parser.feed(corrupt);
    require(frames.size() == 1, "parser did not recover after CRC error");
    require(frames[0].message_type ==
                static_cast<std::uint8_t>(MessageType::EmergencyStop),
            "recovered wrong frame");
    require(parser.stats().crc_errors == 1, "CRC error not counted");
}

void testBadHeaderRecovery() {
    std::vector<std::uint8_t> bad_version{
        0xAA, 0x55, 0x02, 0x11, 0, 0, 1, 0, 0, 0};
    std::vector<std::uint8_t> bad_length{
        0xAA, 0x55, 0x01, 0x11, 49, 0, 2, 0, 0, 0};
    const auto good = creeper::remote::encodeFrame(
        static_cast<std::uint8_t>(MessageType::Stop), 3, {});
    bad_version.insert(bad_version.end(), bad_length.begin(), bad_length.end());
    bad_version.insert(bad_version.end(), good.begin(), good.end());

    StreamParser parser;
    const auto frames = parser.feed(bad_version);
    require(frames.size() == 1 && frames[0].sequence == 3,
            "parser did not recover after invalid headers");
    require(parser.stats().version_errors == 1, "version error not counted");
    require(parser.stats().length_errors == 1, "length error not counted");
}

void testSequenceAndSchemaRules() {
    require(creeper::remote::sequenceIsNew(1, 0), "first sequence rejected");
    require(!creeper::remote::sequenceIsNew(0, 0), "zero sequence accepted");
    require(creeper::remote::sequenceIsNew(1, 0xFFFFFFFFU),
            "sequence wrap rejected");
    require(!creeper::remote::sequenceIsNew(10, 10),
            "duplicate sequence accepted");
    require(!creeper::remote::sequenceIsNew(9, 10),
            "old sequence accepted");

    std::size_t payload_size = 99;
    require(creeper::remote::expectedPayloadSize(
                static_cast<std::uint8_t>(MessageType::Status), payload_size) &&
                payload_size == 20,
            "STATUS schema mismatch");
    require(!creeper::remote::expectedPayloadSize(0xEE, payload_size),
            "unknown message type treated as known");

    CommandVelocity bad{1001, 0, 0};
    require(!creeper::remote::commandVelocityInRange(bad),
            "out-of-range velocity accepted");
    CommandVelocity decoded;
    require(!creeper::remote::decodeCommandVelocityPayload({1, 2}, decoded),
            "invalid velocity payload length accepted");
}

}  // namespace

int main() {
    try {
        testGoldenVector();
        testEverySplitPoint();
        testNoiseStickyFramesAndTrailingSof();
        testCorruptFrameRecovery();
        testBadHeaderRecovery();
        testSequenceAndSchemaRules();
    } catch (const std::exception& error) {
        std::cerr << "test_serial_protocol_v1: FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "test_serial_protocol_v1: PASS\n";
    return EXIT_SUCCESS;
}
