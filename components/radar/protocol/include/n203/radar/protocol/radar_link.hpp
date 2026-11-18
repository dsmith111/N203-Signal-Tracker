#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/radar/radar_frame.hpp"

namespace n203::radar::protocol {

inline constexpr std::uint8_t kProtocolVersion = 2U;
inline constexpr std::size_t kIdentityTextSize = 16U;
inline constexpr std::size_t kWireHeaderSize = 12U;
inline constexpr std::size_t kHelloPayloadSize = 48U;
inline constexpr std::size_t kHelloAckPayloadSize = 8U;
inline constexpr std::size_t kFramePayloadHeaderSize = 20U;
inline constexpr std::size_t kContactPayloadSize = 16U;
inline constexpr std::size_t kCrcSize = 2U;
inline constexpr std::size_t kMaximumPayloadSize =
    kFramePayloadHeaderSize + kMaximumContacts * kContactPayloadSize;
inline constexpr std::size_t kMaximumDecodedPacketSize =
    kWireHeaderSize + kMaximumPayloadSize + kCrcSize;
inline constexpr std::size_t kMaximumCobsEncodedSize =
    kMaximumDecodedPacketSize + kMaximumDecodedPacketSize / 254U + 1U;
inline constexpr std::size_t kMaximumPacketSize = kMaximumCobsEncodedSize + 1U;

enum class PacketType : std::uint8_t {
    kHello = 1,
    kHelloAck = 2,
    kRadarFrame = 3,
};

struct ScannerDescriptor {
    std::uint32_t session_id = 0;
    std::uint32_t capabilities_flags = 0;
    std::uint32_t maximum_range_mm = 0;
    std::uint16_t horizontal_fov_cdeg = 0;
    std::uint8_t maximum_contacts = 0;
    std::array<char, kIdentityTextSize> scanner_firmware{};
    std::array<char, kIdentityTextSize> radar_type{};
};

struct HelloAck {
    std::uint32_t scanner_session_id = 0;
    std::uint8_t accepted_protocol_version = kProtocolVersion;
};

struct PacketMetadata {
    PacketType type = PacketType::kHello;
    std::uint32_t packet_sequence = 0;
    std::uint32_t sender_uptime_ms = 0;
};

struct DecodedPacket {
    PacketMetadata metadata{};
    ScannerDescriptor hello{};
    HelloAck hello_ack{};
    RadarFrame radar_frame{};
};

enum class CodecError : std::uint8_t {
    kNone,
    kInvalidArgument,
    kOutputTooSmall,
    kInvalidFrame,
    kCobsMalformed,
    kUnsupportedVersion,
    kUnsupportedMessageType,
    kInvalidLength,
    kCrcMismatch,
    kInvalidPayload,
};

enum class StreamEvent : std::uint8_t {
    kNone,
    kPacketReady,
    kCobsRejected,
    kCrcRejected,
    kFormatRejected,
    kPayloadRejected,
    kOverflowRejected,
};

enum class SequenceEvent : std::uint8_t {
    kFirst,
    kInOrder,
    kGap,
    kDuplicate,
    kRestart,
};

struct SequenceUpdate {
    SequenceEvent event = SequenceEvent::kFirst;
    std::uint32_t missed = 0;
};

class SequenceTracker final {
public:
    [[nodiscard]] SequenceUpdate observe(std::uint32_t sequence);
    void reset();

private:
    std::uint32_t last_sequence_ = 0;
    bool observed_ = false;
};

[[nodiscard]] std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t size);

[[nodiscard]] CodecError cobsEncode(
    const std::uint8_t* input,
    std::size_t input_size,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written);

[[nodiscard]] CodecError cobsDecode(
    const std::uint8_t* input,
    std::size_t input_size,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written);

[[nodiscard]] CodecError encodeHelloPacket(
    const ScannerDescriptor& descriptor,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written);

[[nodiscard]] CodecError encodeHelloAckPacket(
    const HelloAck& acknowledgment,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written);

[[nodiscard]] CodecError encodeRadarFramePacket(
    const RadarFrame& frame,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written);

[[nodiscard]] CodecError decodePacket(
    const std::uint8_t* decoded_packet,
    std::size_t decoded_packet_size,
    DecodedPacket& packet);

class RadarLinkStreamDecoder final {
public:
    [[nodiscard]] StreamEvent push(std::uint8_t byte, DecodedPacket& packet);
    void reset();

private:
    std::array<std::uint8_t, kMaximumCobsEncodedSize> encoded_{};
    std::array<std::uint8_t, kMaximumDecodedPacketSize> decoded_{};
    std::size_t encoded_size_ = 0;
    bool overflowed_ = false;
};

}  // namespace n203::radar::protocol
