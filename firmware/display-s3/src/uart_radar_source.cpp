#include "uart_radar_source.hpp"

#include <array>
#include <limits>

#include "board_config.hpp"

namespace n203::firmware::display_s3 {
namespace {

constexpr std::size_t kReceiveBufferSize = 512U;
constexpr std::size_t kMaximumBytesPerPoll = 512U;
constexpr std::uint32_t kConnectedTimeoutMs = 2500U;

}  // namespace

void UartRadarSource::begin() {
    decoder_.reset();
    packet_sequence_.reset();
    frame_sequence_.reset();
    descriptor_ = {};
    stats_ = {};
    next_tx_packet_sequence_ = 0U;
    last_valid_packet_ms_ = 0U;
    received_packet_ = false;
    has_descriptor_ = false;
    serial_.setRxBufferSize(kReceiveBufferSize);
    serial_.begin(board::kScannerUartBaud, SERIAL_8N1,
                  board::kScannerUartRx, board::kScannerUartTx);
}

bool UartRadarSource::poll(radar::RadarFrame& frame) {
    bool frame_available = false;
    std::size_t processed = 0;
    radar::protocol::DecodedPacket packet{};
    while (serial_.available() > 0 && processed < kMaximumBytesPerPoll) {
        const int value = serial_.read();
        if (value < 0) {
            break;
        }
        ++processed;
        ++stats_.bytes_received;

        switch (decoder_.push(static_cast<std::uint8_t>(value), packet)) {
            case radar::protocol::StreamEvent::kNone:
                break;
            case radar::protocol::StreamEvent::kPacketReady: {
                ++stats_.packets_received;
                last_valid_packet_ms_ = millis();
                received_packet_ = true;
                radar::RadarFrame decoded{};
                if (handlePacket(packet, decoded)) {
                    if (frame_available) {
                        ++stats_.frames_superseded;
                    }
                    frame = decoded;
                    frame_available = true;
                }
                break;
            }
            case radar::protocol::StreamEvent::kCobsRejected:
                ++stats_.cobs_rejections;
                break;
            case radar::protocol::StreamEvent::kCrcRejected:
                ++stats_.crc_rejections;
                break;
            case radar::protocol::StreamEvent::kFormatRejected:
                ++stats_.format_rejections;
                break;
            case radar::protocol::StreamEvent::kPayloadRejected:
                ++stats_.payload_rejections;
                break;
            case radar::protocol::StreamEvent::kOverflowRejected:
                ++stats_.overflow_rejections;
                break;
        }
    }
    return frame_available;
}

bool UartRadarSource::connected(std::uint32_t now_ms) const {
    return has_descriptor_ && received_packet_ &&
           now_ms - last_valid_packet_ms_ <= kConnectedTimeoutMs;
}

std::uint32_t UartRadarSource::lastPacketAgeMs(std::uint32_t now_ms) const {
    return received_packet_ ? now_ms - last_valid_packet_ms_
                            : std::numeric_limits<std::uint32_t>::max();
}

bool UartRadarSource::handlePacket(
    const radar::protocol::DecodedPacket& packet,
    radar::RadarFrame& frame) {
    bool descriptor_changed = false;
    if (packet.metadata.type == radar::protocol::PacketType::kHello) {
        const bool new_session = !has_descriptor_ ||
                                 descriptor_.session_id != packet.hello.session_id;
        descriptor_changed = !has_descriptor_ ||
            descriptor_.session_id != packet.hello.session_id ||
            descriptor_.capabilities_flags != packet.hello.capabilities_flags ||
            descriptor_.maximum_range_mm != packet.hello.maximum_range_mm ||
            descriptor_.horizontal_fov_cdeg != packet.hello.horizontal_fov_cdeg ||
            descriptor_.maximum_contacts != packet.hello.maximum_contacts ||
            descriptor_.scanner_firmware != packet.hello.scanner_firmware ||
            descriptor_.radar_type != packet.hello.radar_type;
        if (new_session) {
            if (has_descriptor_) {
                ++stats_.session_changes;
            }
            packet_sequence_.reset();
            frame_sequence_.reset();
        }
    }

    if (!recordPacketSequence(packet.metadata.packet_sequence)) {
        if (packet.metadata.type == radar::protocol::PacketType::kHello) {
            ++stats_.hellos_received;
            sendHelloAck(packet.hello.session_id);
        }
        return false;
    }

    switch (packet.metadata.type) {
        case radar::protocol::PacketType::kHello:
            descriptor_ = packet.hello;
            has_descriptor_ = true;
            ++stats_.hellos_received;
            if (descriptor_changed) {
                ++stats_.descriptor_updates;
            }
            sendHelloAck(descriptor_.session_id);
            return false;

        case radar::protocol::PacketType::kHelloAck:
            ++stats_.unexpected_packets;
            return false;

        case radar::protocol::PacketType::kRadarFrame:
            if (!recordFrameSequence(packet.radar_frame.sequence)) {
                return false;
            }
            frame = packet.radar_frame;
            ++stats_.frames_received;
            return true;
    }
    return false;
}

bool UartRadarSource::recordPacketSequence(std::uint32_t sequence) {
    const radar::protocol::SequenceUpdate update = packet_sequence_.observe(sequence);
    switch (update.event) {
        case radar::protocol::SequenceEvent::kFirst:
        case radar::protocol::SequenceEvent::kInOrder:
            return true;
        case radar::protocol::SequenceEvent::kGap:
            stats_.packet_gaps += update.missed;
            return true;
        case radar::protocol::SequenceEvent::kDuplicate:
            ++stats_.packet_duplicates;
            return false;
        case radar::protocol::SequenceEvent::kRestart:
            ++stats_.packet_resets;
            return true;
    }
    return false;
}

bool UartRadarSource::recordFrameSequence(std::uint32_t sequence) {
    const radar::protocol::SequenceUpdate update = frame_sequence_.observe(sequence);
    switch (update.event) {
        case radar::protocol::SequenceEvent::kFirst:
        case radar::protocol::SequenceEvent::kInOrder:
            return true;
        case radar::protocol::SequenceEvent::kGap:
            stats_.sequence_gaps += update.missed;
            return true;
        case radar::protocol::SequenceEvent::kDuplicate:
            ++stats_.sequence_duplicates;
            return false;
        case radar::protocol::SequenceEvent::kRestart:
            ++stats_.sequence_resets;
            return true;
    }
    return false;
}

void UartRadarSource::sendHelloAck(std::uint32_t session_id) {
    radar::protocol::HelloAck acknowledgment{};
    acknowledgment.scanner_session_id = session_id;
    std::array<std::uint8_t, radar::protocol::kMaximumPacketSize> encoded{};
    std::size_t encoded_size = 0;
    const radar::protocol::CodecError result = radar::protocol::encodeHelloAckPacket(
        acknowledgment, next_tx_packet_sequence_, millis(), encoded.data(), encoded.size(),
        encoded_size);
    if (result != radar::protocol::CodecError::kNone ||
        serial_.availableForWrite() < static_cast<int>(encoded_size)) {
        ++stats_.hello_acks_dropped;
        return;
    }

    const std::size_t written = serial_.write(encoded.data(), encoded_size);
    stats_.bytes_transmitted += static_cast<std::uint32_t>(written);
    if (written == encoded_size) {
        ++stats_.hello_acks_sent;
        ++next_tx_packet_sequence_;
    } else {
        ++stats_.hello_acks_dropped;
    }
}

}  // namespace n203::firmware::display_s3
