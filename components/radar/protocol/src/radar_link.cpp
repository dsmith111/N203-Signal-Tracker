#include "n203/radar/protocol/radar_link.hpp"

#include <algorithm>

#include "n203/radar/frame_validation.hpp"

namespace n203::radar::protocol {
namespace {

void writeU16(std::uint8_t* destination, std::uint16_t value) {
    destination[0] = static_cast<std::uint8_t>(value);
    destination[1] = static_cast<std::uint8_t>(value >> 8U);
}

void writeU32(std::uint8_t* destination, std::uint32_t value) {
    destination[0] = static_cast<std::uint8_t>(value);
    destination[1] = static_cast<std::uint8_t>(value >> 8U);
    destination[2] = static_cast<std::uint8_t>(value >> 16U);
    destination[3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint16_t readU16(const std::uint8_t* source) {
    return static_cast<std::uint16_t>(source[0]) |
           static_cast<std::uint16_t>(source[1]) << 8U;
}

std::uint32_t readU32(const std::uint8_t* source) {
    return static_cast<std::uint32_t>(source[0]) |
           static_cast<std::uint32_t>(source[1]) << 8U |
           static_cast<std::uint32_t>(source[2]) << 16U |
           static_cast<std::uint32_t>(source[3]) << 24U;
}

bool validPacketType(std::uint8_t value) {
    return value >= static_cast<std::uint8_t>(PacketType::kHello) &&
           value <= static_cast<std::uint8_t>(PacketType::kRadarFrame);
}

bool validHealth(std::uint8_t value) {
    return value <= static_cast<std::uint8_t>(RadarHealth::kOffline);
}

bool validMeasurementState(std::uint8_t value) {
    return value <= static_cast<std::uint8_t>(MeasurementState::kOffline);
}

std::size_t framePayloadSize(std::uint8_t contact_count) {
    return kFramePayloadHeaderSize +
           static_cast<std::size_t>(contact_count) * kContactPayloadSize;
}

CodecError encodeEnvelope(
    PacketType type,
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if ((payload == nullptr && payload_size != 0U) || output == nullptr) {
        return CodecError::kInvalidArgument;
    }
    if (payload_size > kMaximumPayloadSize || output_capacity < 2U) {
        return CodecError::kOutputTooSmall;
    }

    std::array<std::uint8_t, kMaximumDecodedPacketSize> decoded{};
    const std::size_t decoded_size = kWireHeaderSize + payload_size + kCrcSize;
    decoded[0] = kProtocolVersion;
    decoded[1] = static_cast<std::uint8_t>(type);
    writeU16(decoded.data() + 2, static_cast<std::uint16_t>(payload_size));
    writeU32(decoded.data() + 4, packet_sequence);
    writeU32(decoded.data() + 8, sender_uptime_ms);
    if (payload_size != 0U) {
        std::copy_n(payload, payload_size, decoded.data() + kWireHeaderSize);
    }
    writeU16(decoded.data() + decoded_size - kCrcSize,
             crc16Ccitt(decoded.data(), decoded_size - kCrcSize));

    std::size_t encoded_size = 0;
    const CodecError encoded = cobsEncode(decoded.data(), decoded_size, output,
                                           output_capacity - 1U, encoded_size);
    if (encoded != CodecError::kNone) {
        return encoded;
    }
    output[encoded_size] = 0U;
    bytes_written = encoded_size + 1U;
    return CodecError::kNone;
}

CodecError decodeHelloPayload(
    const std::uint8_t* payload,
    std::size_t payload_size,
    ScannerDescriptor& descriptor) {
    if (payload_size != kHelloPayloadSize) {
        return CodecError::kInvalidLength;
    }
    if (payload[15] != 0U || payload[14] > kMaximumContacts ||
        readU16(payload + 12) > 36000U) {
        return CodecError::kInvalidPayload;
    }

    descriptor = {};
    descriptor.session_id = readU32(payload);
    descriptor.capabilities_flags = readU32(payload + 4);
    descriptor.maximum_range_mm = readU32(payload + 8);
    descriptor.horizontal_fov_cdeg = readU16(payload + 12);
    descriptor.maximum_contacts = payload[14];
    std::copy_n(reinterpret_cast<const char*>(payload + 16), kIdentityTextSize,
                descriptor.scanner_firmware.begin());
    std::copy_n(reinterpret_cast<const char*>(payload + 32), kIdentityTextSize,
                descriptor.radar_type.begin());
    return CodecError::kNone;
}

CodecError decodeHelloAckPayload(
    const std::uint8_t* payload,
    std::size_t payload_size,
    HelloAck& acknowledgment) {
    if (payload_size != kHelloAckPayloadSize) {
        return CodecError::kInvalidLength;
    }
    if (payload[4] != kProtocolVersion || payload[5] != 0U || payload[6] != 0U ||
        payload[7] != 0U) {
        return CodecError::kInvalidPayload;
    }
    acknowledgment = {};
    acknowledgment.scanner_session_id = readU32(payload);
    acknowledgment.accepted_protocol_version = payload[4];
    return CodecError::kNone;
}

CodecError decodeRadarFramePayload(
    const std::uint8_t* payload,
    std::size_t payload_size,
    RadarFrame& frame) {
    if (payload_size < kFramePayloadHeaderSize || payload_size > kMaximumPayloadSize ||
        (payload_size - kFramePayloadHeaderSize) % kContactPayloadSize != 0U) {
        return CodecError::kInvalidLength;
    }

    const std::uint8_t contact_count = payload[17];
    if (payload[18] != 0U || payload[19] != 0U || contact_count > kMaximumContacts ||
        payload_size != framePayloadSize(contact_count) || !validHealth(payload[12]) ||
        !validMeasurementState(payload[13])) {
        return CodecError::kInvalidPayload;
    }

    RadarFrame decoded{};
    decoded.sequence = readU32(payload);
    decoded.sensor_timestamp_ms = readU32(payload + 4);
    decoded.capabilities.flags = readU32(payload + 8);
    decoded.health = static_cast<RadarHealth>(payload[12]);
    decoded.measurement_state = static_cast<MeasurementState>(payload[13]);
    decoded.presence = static_cast<PresenceState>(payload[14]);
    decoded.motion = static_cast<MotionState>(payload[15]);
    decoded.motion_quality = payload[16];
    decoded.contact_count = contact_count;

    for (std::size_t index = 0; index < contact_count; ++index) {
        const std::uint8_t* encoded =
            payload + kFramePayloadHeaderSize + index * kContactPayloadSize;
        Contact& contact = decoded.contacts[index];
        contact.id = encoded[0];
        contact.confidence = encoded[1];
        contact.valid_fields = readU16(encoded + 2);
        contact.x_mm = static_cast<std::int16_t>(readU16(encoded + 4));
        contact.y_mm = static_cast<std::int16_t>(readU16(encoded + 6));
        contact.range_mm = readU16(encoded + 8);
        contact.azimuth_cdeg = static_cast<std::int16_t>(readU16(encoded + 10));
        contact.radial_velocity_mm_s = static_cast<std::int16_t>(readU16(encoded + 12));
        contact.signal = readU16(encoded + 14);
    }

    if (validateFrame(decoded) != FrameValidationError::kNone) {
        return CodecError::kInvalidPayload;
    }
    frame = decoded;
    return CodecError::kNone;
}

}  // namespace

std::uint16_t crc16Ccitt(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

CodecError cobsEncode(
    const std::uint8_t* input,
    std::size_t input_size,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if ((input == nullptr && input_size != 0U) || output == nullptr) {
        return CodecError::kInvalidArgument;
    }
    if (output_capacity == 0U) {
        return CodecError::kOutputTooSmall;
    }

    std::size_t code_index = 0;
    std::size_t write_index = 1;
    std::uint8_t code = 1U;
    for (std::size_t read_index = 0; read_index < input_size; ++read_index) {
        if (input[read_index] == 0U) {
            output[code_index] = code;
            code_index = write_index;
            if (write_index >= output_capacity) {
                return CodecError::kOutputTooSmall;
            }
            ++write_index;
            code = 1U;
            continue;
        }

        if (write_index >= output_capacity) {
            return CodecError::kOutputTooSmall;
        }
        output[write_index++] = input[read_index];
        ++code;
        if (code == 0xFFU) {
            output[code_index] = code;
            code_index = write_index;
            if (write_index >= output_capacity) {
                return CodecError::kOutputTooSmall;
            }
            ++write_index;
            code = 1U;
        }
    }
    output[code_index] = code;
    bytes_written = write_index;
    return CodecError::kNone;
}

CodecError cobsDecode(
    const std::uint8_t* input,
    std::size_t input_size,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if (input == nullptr || output == nullptr || input_size == 0U) {
        return CodecError::kInvalidArgument;
    }

    std::size_t read_index = 0;
    std::size_t write_index = 0;
    while (read_index < input_size) {
        const std::uint8_t code = input[read_index++];
        if (code == 0U) {
            return CodecError::kCobsMalformed;
        }
        const std::size_t copy_size = static_cast<std::size_t>(code - 1U);
        if (read_index + copy_size > input_size ||
            write_index + copy_size > output_capacity) {
            return read_index + copy_size > input_size
                       ? CodecError::kCobsMalformed
                       : CodecError::kOutputTooSmall;
        }
        std::copy_n(input + read_index, copy_size, output + write_index);
        read_index += copy_size;
        write_index += copy_size;
        if (code != 0xFFU && read_index < input_size) {
            if (write_index >= output_capacity) {
                return CodecError::kOutputTooSmall;
            }
            output[write_index++] = 0U;
        }
    }
    bytes_written = write_index;
    return CodecError::kNone;
}

CodecError encodeHelloPacket(
    const ScannerDescriptor& descriptor,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if (descriptor.maximum_contacts > kMaximumContacts ||
        descriptor.horizontal_fov_cdeg > 36000U) {
        return CodecError::kInvalidPayload;
    }
    std::array<std::uint8_t, kHelloPayloadSize> payload{};
    writeU32(payload.data(), descriptor.session_id);
    writeU32(payload.data() + 4, descriptor.capabilities_flags);
    writeU32(payload.data() + 8, descriptor.maximum_range_mm);
    writeU16(payload.data() + 12, descriptor.horizontal_fov_cdeg);
    payload[14] = descriptor.maximum_contacts;
    std::copy(descriptor.scanner_firmware.begin(), descriptor.scanner_firmware.end(),
              reinterpret_cast<char*>(payload.data() + 16));
    std::copy(descriptor.radar_type.begin(), descriptor.radar_type.end(),
              reinterpret_cast<char*>(payload.data() + 32));
    return encodeEnvelope(PacketType::kHello, payload.data(), payload.size(), packet_sequence,
                          sender_uptime_ms, output, output_capacity, bytes_written);
}

CodecError encodeHelloAckPacket(
    const HelloAck& acknowledgment,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if (acknowledgment.accepted_protocol_version != kProtocolVersion) {
        return CodecError::kInvalidPayload;
    }
    std::array<std::uint8_t, kHelloAckPayloadSize> payload{};
    writeU32(payload.data(), acknowledgment.scanner_session_id);
    payload[4] = acknowledgment.accepted_protocol_version;
    return encodeEnvelope(PacketType::kHelloAck, payload.data(), payload.size(), packet_sequence,
                          sender_uptime_ms, output, output_capacity, bytes_written);
}

CodecError encodeRadarFramePacket(
    const RadarFrame& frame,
    std::uint32_t packet_sequence,
    std::uint32_t sender_uptime_ms,
    std::uint8_t* output,
    std::size_t output_capacity,
    std::size_t& bytes_written) {
    bytes_written = 0;
    if (validateFrame(frame) != FrameValidationError::kNone ||
        !validHealth(static_cast<std::uint8_t>(frame.health)) ||
        !validMeasurementState(static_cast<std::uint8_t>(frame.measurement_state))) {
        return CodecError::kInvalidFrame;
    }

    std::array<std::uint8_t, kMaximumPayloadSize> payload{};
    const std::size_t payload_size = framePayloadSize(frame.contact_count);
    writeU32(payload.data(), frame.sequence);
    writeU32(payload.data() + 4, frame.sensor_timestamp_ms);
    writeU32(payload.data() + 8, frame.capabilities.flags);
    payload[12] = static_cast<std::uint8_t>(frame.health);
    payload[13] = static_cast<std::uint8_t>(frame.measurement_state);
    payload[14] = static_cast<std::uint8_t>(frame.presence);
    payload[15] = static_cast<std::uint8_t>(frame.motion);
    payload[16] = frame.motion_quality;
    payload[17] = frame.contact_count;

    for (std::size_t index = 0; index < frame.contact_count; ++index) {
        const Contact& contact = frame.contacts[index];
        std::uint8_t* encoded =
            payload.data() + kFramePayloadHeaderSize + index * kContactPayloadSize;
        encoded[0] = contact.id;
        encoded[1] = contact.confidence;
        writeU16(encoded + 2, contact.valid_fields);
        writeU16(encoded + 4, static_cast<std::uint16_t>(contact.x_mm));
        writeU16(encoded + 6, static_cast<std::uint16_t>(contact.y_mm));
        writeU16(encoded + 8, contact.range_mm);
        writeU16(encoded + 10, static_cast<std::uint16_t>(contact.azimuth_cdeg));
        writeU16(encoded + 12, static_cast<std::uint16_t>(contact.radial_velocity_mm_s));
        writeU16(encoded + 14, contact.signal);
    }

    return encodeEnvelope(PacketType::kRadarFrame, payload.data(), payload_size,
                          packet_sequence, sender_uptime_ms,
                          output, output_capacity, bytes_written);
}

CodecError decodePacket(
    const std::uint8_t* decoded_packet,
    std::size_t decoded_packet_size,
    DecodedPacket& packet) {
    if (decoded_packet == nullptr || decoded_packet_size < kWireHeaderSize + kCrcSize) {
        return CodecError::kInvalidLength;
    }

    const std::size_t payload_size = readU16(decoded_packet + 2);
    if (payload_size > kMaximumPayloadSize ||
        decoded_packet_size != kWireHeaderSize + payload_size + kCrcSize) {
        return CodecError::kInvalidLength;
    }
    const std::uint16_t expected_crc = readU16(decoded_packet + decoded_packet_size - kCrcSize);
    const std::uint16_t actual_crc = crc16Ccitt(decoded_packet, decoded_packet_size - kCrcSize);
    if (expected_crc != actual_crc) {
        return CodecError::kCrcMismatch;
    }
    if (decoded_packet[0] != kProtocolVersion) {
        return CodecError::kUnsupportedVersion;
    }
    if (!validPacketType(decoded_packet[1])) {
        return CodecError::kUnsupportedMessageType;
    }

    DecodedPacket decoded{};
    decoded.metadata.type = static_cast<PacketType>(decoded_packet[1]);
    decoded.metadata.packet_sequence = readU32(decoded_packet + 4);
    decoded.metadata.sender_uptime_ms = readU32(decoded_packet + 8);
    const std::uint8_t* payload = decoded_packet + kWireHeaderSize;

    CodecError result = CodecError::kUnsupportedMessageType;
    switch (decoded.metadata.type) {
        case PacketType::kHello:
            result = decodeHelloPayload(payload, payload_size, decoded.hello);
            break;
        case PacketType::kHelloAck:
            result = decodeHelloAckPayload(payload, payload_size, decoded.hello_ack);
            break;
        case PacketType::kRadarFrame:
            result = decodeRadarFramePayload(payload, payload_size, decoded.radar_frame);
            break;
    }
    if (result == CodecError::kNone) {
        packet = decoded;
    }
    return result;
}

SequenceUpdate SequenceTracker::observe(std::uint32_t sequence) {
    if (!observed_) {
        observed_ = true;
        last_sequence_ = sequence;
        return {SequenceEvent::kFirst, 0U};
    }

    const std::uint32_t delta = sequence - last_sequence_;
    if (delta == 0U) {
        return {SequenceEvent::kDuplicate, 0U};
    }
    last_sequence_ = sequence;
    if (delta < 0x80000000U) {
        return delta == 1U ? SequenceUpdate{SequenceEvent::kInOrder, 0U}
                           : SequenceUpdate{SequenceEvent::kGap, delta - 1U};
    }
    return {SequenceEvent::kRestart, 0U};
}

void SequenceTracker::reset() {
    last_sequence_ = 0U;
    observed_ = false;
}

StreamEvent RadarLinkStreamDecoder::push(std::uint8_t byte, DecodedPacket& packet) {
    if (byte != 0U) {
        if (overflowed_) {
            return StreamEvent::kNone;
        }
        if (encoded_size_ >= encoded_.size()) {
            overflowed_ = true;
            return StreamEvent::kNone;
        }
        encoded_[encoded_size_++] = byte;
        return StreamEvent::kNone;
    }

    if (overflowed_) {
        reset();
        return StreamEvent::kOverflowRejected;
    }
    if (encoded_size_ == 0U) {
        return StreamEvent::kNone;
    }

    std::size_t decoded_size = 0;
    const CodecError cobs_result = cobsDecode(encoded_.data(), encoded_size_, decoded_.data(),
                                               decoded_.size(), decoded_size);
    reset();
    if (cobs_result != CodecError::kNone) {
        return StreamEvent::kCobsRejected;
    }

    const CodecError result = decodePacket(decoded_.data(), decoded_size, packet);
    switch (result) {
        case CodecError::kNone:
            return StreamEvent::kPacketReady;
        case CodecError::kCrcMismatch:
            return StreamEvent::kCrcRejected;
        case CodecError::kInvalidFrame:
        case CodecError::kInvalidPayload:
            return StreamEvent::kPayloadRejected;
        default:
            return StreamEvent::kFormatRejected;
    }
}

void RadarLinkStreamDecoder::reset() {
    encoded_size_ = 0;
    overflowed_ = false;
}

}  // namespace n203::radar::protocol
