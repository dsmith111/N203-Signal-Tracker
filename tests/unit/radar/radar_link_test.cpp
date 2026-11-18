#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "n203/radar/protocol/radar_link.hpp"

namespace {

using n203::radar::Contact;
using n203::radar::MeasurementState;
using n203::radar::MotionState;
using n203::radar::PresenceState;
using n203::radar::RadarFrame;
using n203::radar::RadarHealth;
namespace protocol = n203::radar::protocol;

template <std::size_t Size>
void setText(std::array<char, Size>& destination, const char* value) {
    destination.fill('\0');
    for (std::size_t index = 0; index < Size && value[index] != '\0'; ++index) {
        destination[index] = value[index];
    }
}

RadarFrame sampleFrame() {
    RadarFrame frame{};
    frame.sequence = 0x12345678U;
    frame.sensor_timestamp_ms = 9321U;
    frame.capabilities.flags =
        n203::radar::kPresence | n203::radar::kRange | n203::radar::kPosition |
        n203::radar::kAzimuth | n203::radar::kVelocity |
        n203::radar::kTrackIdentity | n203::radar::kMultipleContacts;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kScanning;
    frame.presence = PresenceState::kDetected;
    frame.motion = MotionState::kTracking;
    frame.motion_quality = 93;
    frame.contact_count = 2;

    frame.contacts[0].id = 7;
    frame.contacts[0].confidence = 94;
    frame.contacts[0].valid_fields =
        n203::radar::kContactPosition | n203::radar::kContactRange |
        n203::radar::kContactAzimuth | n203::radar::kContactRadialVelocity |
        n203::radar::kContactSignal | n203::radar::kContactConfidence;
    frame.contacts[0].x_mm = -900;
    frame.contacts[0].y_mm = 5100;
    frame.contacts[0].range_mm = 5179;
    frame.contacts[0].azimuth_cdeg = -1001;
    frame.contacts[0].radial_velocity_mm_s = -120;
    frame.contacts[0].signal = 780;

    frame.contacts[1] = frame.contacts[0];
    frame.contacts[1].id = 9;
    frame.contacts[1].x_mm = 1800;
    frame.contacts[1].azimuth_cdeg = 2790;
    frame.contacts[1].radial_velocity_mm_s = 75;
    return frame;
}

void assertContactEqual(const Contact& left, const Contact& right) {
    assert(left.id == right.id);
    assert(left.valid_fields == right.valid_fields);
    assert(left.x_mm == right.x_mm);
    assert(left.y_mm == right.y_mm);
    assert(left.range_mm == right.range_mm);
    assert(left.azimuth_cdeg == right.azimuth_cdeg);
    assert(left.radial_velocity_mm_s == right.radial_velocity_mm_s);
    assert(left.signal == right.signal);
    assert(left.confidence == right.confidence);
}

void assertFrameEqual(const RadarFrame& left, const RadarFrame& right) {
    assert(left.sequence == right.sequence);
    assert(left.sensor_timestamp_ms == right.sensor_timestamp_ms);
    assert(left.capabilities.flags == right.capabilities.flags);
    assert(left.health == right.health);
    assert(left.measurement_state == right.measurement_state);
    assert(left.presence == right.presence);
    assert(left.motion == right.motion);
    assert(left.motion_quality == right.motion_quality);
    assert(left.contact_count == right.contact_count);
    for (std::size_t index = 0; index < left.contact_count; ++index) {
        assertContactEqual(left.contacts[index], right.contacts[index]);
    }
}

protocol::DecodedPacket decodeWire(
    const std::array<std::uint8_t, protocol::kMaximumPacketSize>& wire,
    std::size_t wire_size) {
    assert(wire_size >= 2U);
    assert(wire[wire_size - 1U] == 0U);
    std::array<std::uint8_t, protocol::kMaximumDecodedPacketSize> decoded{};
    std::size_t decoded_size = 0;
    assert(protocol::cobsDecode(wire.data(), wire_size - 1U, decoded.data(), decoded.size(),
                                decoded_size) == protocol::CodecError::kNone);
    protocol::DecodedPacket packet{};
    assert(protocol::decodePacket(decoded.data(), decoded_size, packet) ==
           protocol::CodecError::kNone);
    return packet;
}

std::size_t extractRaw(
    const std::array<std::uint8_t, protocol::kMaximumPacketSize>& wire,
    std::size_t wire_size,
    std::array<std::uint8_t, protocol::kMaximumDecodedPacketSize>& raw) {
    std::size_t raw_size = 0;
    assert(protocol::cobsDecode(wire.data(), wire_size - 1U, raw.data(), raw.size(), raw_size) ==
           protocol::CodecError::kNone);
    return raw_size;
}

void writeRawCrc(
    std::array<std::uint8_t, protocol::kMaximumDecodedPacketSize>& raw,
    std::size_t raw_size) {
    const std::uint16_t crc = protocol::crc16Ccitt(raw.data(), raw_size - protocol::kCrcSize);
    raw[raw_size - 2U] = static_cast<std::uint8_t>(crc);
    raw[raw_size - 1U] = static_cast<std::uint8_t>(crc >> 8U);
}

std::size_t encodeRaw(
    const std::array<std::uint8_t, protocol::kMaximumDecodedPacketSize>& raw,
    std::size_t raw_size,
    std::array<std::uint8_t, protocol::kMaximumPacketSize>& wire) {
    std::size_t encoded_size = 0;
    assert(protocol::cobsEncode(raw.data(), raw_size, wire.data(), wire.size() - 1U,
                                encoded_size) == protocol::CodecError::kNone);
    wire[encoded_size] = 0U;
    return encoded_size + 1U;
}

protocol::StreamEvent feed(
    protocol::RadarLinkStreamDecoder& stream,
    const std::uint8_t* bytes,
    std::size_t size,
    protocol::DecodedPacket& packet) {
    protocol::StreamEvent event = protocol::StreamEvent::kNone;
    for (std::size_t index = 0; index < size; ++index) {
        const protocol::StreamEvent next = stream.push(bytes[index], packet);
        if (next != protocol::StreamEvent::kNone) {
            event = next;
        }
    }
    return event;
}

void testCobs() {
    constexpr std::array<std::uint8_t, 9> input{{0, 1, 2, 0, 3, 0, 0, 4, 5}};
    std::array<std::uint8_t, 32> encoded{};
    std::array<std::uint8_t, 32> decoded{};
    std::size_t encoded_size = 0;
    std::size_t decoded_size = 0;
    assert(protocol::cobsEncode(input.data(), input.size(), encoded.data(), encoded.size(),
                                encoded_size) == protocol::CodecError::kNone);
    assert(std::find(encoded.begin(), encoded.begin() + encoded_size, 0U) ==
           encoded.begin() + encoded_size);
    assert(protocol::cobsDecode(encoded.data(), encoded_size, decoded.data(), decoded.size(),
                                decoded_size) == protocol::CodecError::kNone);
    assert(decoded_size == input.size());
    assert(std::equal(input.begin(), input.end(), decoded.begin()));

    std::array<std::uint8_t, 254> maximum_block{};
    maximum_block.fill(0x5AU);
    std::array<std::uint8_t, 256> maximum_encoded{};
    std::array<std::uint8_t, 254> maximum_decoded{};
    assert(protocol::cobsEncode(maximum_block.data(), maximum_block.size(),
                                maximum_encoded.data(), maximum_encoded.size(), encoded_size) ==
           protocol::CodecError::kNone);
    assert(encoded_size == 256U);
    assert(protocol::cobsDecode(maximum_encoded.data(), encoded_size, maximum_decoded.data(),
                                maximum_decoded.size(), decoded_size) ==
           protocol::CodecError::kNone);
    assert(decoded_size == maximum_block.size());
    assert(maximum_decoded == maximum_block);

    assert(protocol::cobsEncode(input.data(), input.size(), encoded.data(), 1U, encoded_size) ==
           protocol::CodecError::kOutputTooSmall);
    constexpr std::array<std::uint8_t, 2> malformed{{3, 0x55}};
    assert(protocol::cobsDecode(malformed.data(), malformed.size(), decoded.data(), decoded.size(),
                                decoded_size) == protocol::CodecError::kCobsMalformed);
}

void testHelloAndAck() {
    protocol::ScannerDescriptor descriptor{};
    descriptor.session_id = 0xA1B2C3D4U;
    descriptor.capabilities_flags = n203::radar::kPresence | n203::radar::kRange;
    descriptor.maximum_range_mm = 12000U;
    descriptor.horizontal_fov_cdeg = 8000U;
    descriptor.maximum_contacts = 1U;
    setText(descriptor.scanner_firmware, "c3-sim/0.1");
    setText(descriptor.radar_type, "simulated");

    std::array<std::uint8_t, protocol::kMaximumPacketSize> wire{};
    std::size_t wire_size = 0;
    assert(protocol::encodeHelloPacket(descriptor, 41U, 900U, wire.data(), wire.size(),
                                       wire_size) == protocol::CodecError::kNone);
    const protocol::DecodedPacket hello = decodeWire(wire, wire_size);
    assert(hello.metadata.type == protocol::PacketType::kHello);
    assert(hello.metadata.packet_sequence == 41U);
    assert(hello.metadata.sender_uptime_ms == 900U);
    assert(hello.hello.session_id == descriptor.session_id);
    assert(hello.hello.capabilities_flags == descriptor.capabilities_flags);
    assert(hello.hello.maximum_range_mm == descriptor.maximum_range_mm);
    assert(hello.hello.horizontal_fov_cdeg == descriptor.horizontal_fov_cdeg);
    assert(hello.hello.maximum_contacts == descriptor.maximum_contacts);
    assert(hello.hello.scanner_firmware == descriptor.scanner_firmware);
    assert(hello.hello.radar_type == descriptor.radar_type);

    protocol::HelloAck acknowledgment{};
    acknowledgment.scanner_session_id = descriptor.session_id;
    assert(protocol::encodeHelloAckPacket(acknowledgment, 7U, 1000U, wire.data(), wire.size(),
                                          wire_size) == protocol::CodecError::kNone);
    const protocol::DecodedPacket ack = decodeWire(wire, wire_size);
    assert(ack.metadata.type == protocol::PacketType::kHelloAck);
    assert(ack.metadata.packet_sequence == 7U);
    assert(ack.hello_ack.scanner_session_id == descriptor.session_id);
    assert(ack.hello_ack.accepted_protocol_version == protocol::kProtocolVersion);
}

void testRadarFrames() {
    const RadarFrame source = sampleFrame();
    std::array<std::uint8_t, protocol::kMaximumPacketSize> wire{};
    std::size_t wire_size = 0;
    assert(protocol::encodeRadarFramePacket(source, 22U, 555U, wire.data(), wire.size(),
                                            wire_size) == protocol::CodecError::kNone);
    protocol::DecodedPacket packet = decodeWire(wire, wire_size);
    assert(packet.metadata.type == protocol::PacketType::kRadarFrame);
    assert(packet.metadata.packet_sequence == 22U);
    assert(packet.metadata.sender_uptime_ms == 555U);
    assertFrameEqual(source, packet.radar_frame);

    RadarFrame empty = source;
    empty.contact_count = 0;
    assert(protocol::encodeRadarFramePacket(empty, 23U, 655U, wire.data(), wire.size(), wire_size) ==
           protocol::CodecError::kNone);
    packet = decodeWire(wire, wire_size);
    assertFrameEqual(empty, packet.radar_frame);

    RadarFrame clear = empty;
    clear.presence = PresenceState::kClear;
    assert(protocol::encodeRadarFramePacket(clear, 24U, 755U, wire.data(), wire.size(), wire_size) ==
           protocol::CodecError::kNone);
    packet = decodeWire(wire, wire_size);
    assertFrameEqual(clear, packet.radar_frame);

    RadarFrame maximum = source;
    maximum.contact_count = static_cast<std::uint8_t>(n203::radar::kMaximumContacts);
    for (std::size_t index = 0; index < maximum.contact_count; ++index) {
        maximum.contacts[index] = source.contacts[index % source.contact_count];
        maximum.contacts[index].id = static_cast<std::uint8_t>(index + 1U);
    }
    assert(protocol::encodeRadarFramePacket(maximum, 25U, 855U, wire.data(), wire.size(),
                                            wire_size) == protocol::CodecError::kNone);
    packet = decodeWire(wire, wire_size);
    assertFrameEqual(maximum, packet.radar_frame);

    for (std::uint8_t value = static_cast<std::uint8_t>(MotionState::kUnknown);
         value <= static_cast<std::uint8_t>(MotionState::kStabilizing); ++value) {
        RadarFrame motion = empty;
        motion.motion = static_cast<MotionState>(value);
        motion.motion_quality = motion.motion == MotionState::kUnknown ? 0U :
                                static_cast<std::uint8_t>(20U + value);
        assert(protocol::encodeRadarFramePacket(motion, value, 1000U, wire.data(), wire.size(),
                                                wire_size) == protocol::CodecError::kNone);
        packet = decodeWire(wire, wire_size);
        assertFrameEqual(motion, packet.radar_frame);
    }

    assert(protocol::encodeRadarFramePacket(source, 1U, 1U, wire.data(), wire.size(), wire_size) ==
           protocol::CodecError::kNone);
    std::size_t unused_size = 123U;
    assert(protocol::encodeRadarFramePacket(source, 1U, 1U, wire.data(), wire_size - 1U,
                                            unused_size) == protocol::CodecError::kOutputTooSmall);
    assert(unused_size == 0U);

    RadarFrame invalid = source;
    invalid.motion_quality = 101U;
    assert(protocol::encodeRadarFramePacket(invalid, 1U, 1U, wire.data(), wire.size(),
                                            unused_size) == protocol::CodecError::kInvalidFrame);
}

void testStreamRecoveryAndValidation() {
    const RadarFrame source = sampleFrame();
    std::array<std::uint8_t, protocol::kMaximumPacketSize> valid{};
    std::size_t valid_size = 0;
    assert(protocol::encodeRadarFramePacket(source, 90U, 1234U, valid.data(), valid.size(),
                                            valid_size) == protocol::CodecError::kNone);

    protocol::RadarLinkStreamDecoder stream;
    protocol::DecodedPacket packet{};
    assert(feed(stream, valid.data(), valid_size, packet) == protocol::StreamEvent::kPacketReady);
    assertFrameEqual(source, packet.radar_frame);
    assert(stream.push(0U, packet) == protocol::StreamEvent::kNone);

    constexpr std::array<std::uint8_t, 3> bad_cobs{{5U, 0x11U, 0U}};
    assert(feed(stream, bad_cobs.data(), bad_cobs.size(), packet) ==
           protocol::StreamEvent::kCobsRejected);

    std::array<std::uint8_t, protocol::kMaximumDecodedPacketSize> raw{};
    const std::size_t raw_size = extractRaw(valid, valid_size, raw);
    std::array<std::uint8_t, protocol::kMaximumPacketSize> changed{};

    auto corrupted_raw = raw;
    corrupted_raw[protocol::kWireHeaderSize + 4U] ^= 0x40U;
    std::size_t changed_size = encodeRaw(corrupted_raw, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kCrcRejected);
    assert(feed(stream, valid.data(), valid_size, packet) == protocol::StreamEvent::kPacketReady);

    auto bad_version = raw;
    bad_version[0] = protocol::kProtocolVersion + 1U;
    writeRawCrc(bad_version, raw_size);
    changed_size = encodeRaw(bad_version, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kFormatRejected);

    auto bad_type = raw;
    bad_type[1] = 0x7FU;
    writeRawCrc(bad_type, raw_size);
    changed_size = encodeRaw(bad_type, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kFormatRejected);

    auto bad_length = raw;
    ++bad_length[2];
    writeRawCrc(bad_length, raw_size);
    changed_size = encodeRaw(bad_length, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kFormatRejected);

    auto bad_enum = raw;
    bad_enum[protocol::kWireHeaderSize + 14U] = 0x7FU;
    writeRawCrc(bad_enum, raw_size);
    changed_size = encodeRaw(bad_enum, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kPayloadRejected);

    auto bad_payload = raw;
    bad_payload[protocol::kWireHeaderSize + 18U] = 1U;
    writeRawCrc(bad_payload, raw_size);
    changed_size = encodeRaw(bad_payload, raw_size, changed);
    assert(feed(stream, changed.data(), changed_size, packet) ==
           protocol::StreamEvent::kPayloadRejected);

    assert(feed(stream, valid.data(), valid_size - 2U, packet) == protocol::StreamEvent::kNone);
    const protocol::StreamEvent truncated = stream.push(0U, packet);
    assert(truncated == protocol::StreamEvent::kCobsRejected ||
           truncated == protocol::StreamEvent::kFormatRejected);

    for (std::size_t index = 0; index <= protocol::kMaximumCobsEncodedSize; ++index) {
        assert(stream.push(1U, packet) == protocol::StreamEvent::kNone);
    }
    assert(stream.push(0U, packet) == protocol::StreamEvent::kOverflowRejected);

    constexpr std::array<std::uint8_t, 3> garbage{{2U, 0x55U, 0U}};
    assert(feed(stream, garbage.data(), garbage.size(), packet) ==
           protocol::StreamEvent::kFormatRejected);
    assert(feed(stream, valid.data(), valid_size, packet) == protocol::StreamEvent::kPacketReady);

    std::size_t ready_count = 0;
    for (int copy = 0; copy < 2; ++copy) {
        for (std::size_t index = 0; index < valid_size; ++index) {
            if (stream.push(valid[index], packet) == protocol::StreamEvent::kPacketReady) {
                ++ready_count;
            }
        }
    }
    assert(ready_count == 2U);
}

void testSequenceTracking() {
    protocol::SequenceTracker tracker;
    assert(tracker.observe(10U).event == protocol::SequenceEvent::kFirst);
    assert(tracker.observe(11U).event == protocol::SequenceEvent::kInOrder);
    const protocol::SequenceUpdate gap = tracker.observe(15U);
    assert(gap.event == protocol::SequenceEvent::kGap);
    assert(gap.missed == 3U);
    assert(tracker.observe(15U).event == protocol::SequenceEvent::kDuplicate);
    assert(tracker.observe(2U).event == protocol::SequenceEvent::kRestart);

    tracker.reset();
    assert(tracker.observe(0xFFFFFFFEU).event == protocol::SequenceEvent::kFirst);
    assert(tracker.observe(0xFFFFFFFFU).event == protocol::SequenceEvent::kInOrder);
    assert(tracker.observe(0U).event == protocol::SequenceEvent::kInOrder);
}

}  // namespace

void runRadarLinkTests() {
    constexpr std::array<std::uint8_t, 9> kCrcCheck{{'1', '2', '3', '4', '5', '6', '7', '8', '9'}};
    assert(protocol::crc16Ccitt(kCrcCheck.data(), kCrcCheck.size()) == 0x29B1U);
    testCobs();
    testHelloAndAck();
    testRadarFrames();
    testStreamRecoveryAndValidation();
    testSequenceTracking();
}
