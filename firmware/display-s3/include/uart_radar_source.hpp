#pragma once

#include <Arduino.h>

#include <cstdint>

#include "n203/radar/protocol/radar_link.hpp"
#include "n203/radar/radar_source.hpp"

namespace n203::firmware::display_s3 {

struct UartRadarStats {
    std::uint32_t bytes_received = 0;
    std::uint32_t bytes_transmitted = 0;
    std::uint32_t packets_received = 0;
    std::uint32_t packet_gaps = 0;
    std::uint32_t packet_duplicates = 0;
    std::uint32_t packet_resets = 0;
    std::uint32_t hellos_received = 0;
    std::uint32_t hello_acks_sent = 0;
    std::uint32_t hello_acks_dropped = 0;
    std::uint32_t descriptor_updates = 0;
    std::uint32_t session_changes = 0;
    std::uint32_t frames_received = 0;
    std::uint32_t frames_superseded = 0;
    std::uint32_t sequence_gaps = 0;
    std::uint32_t sequence_duplicates = 0;
    std::uint32_t sequence_resets = 0;
    std::uint32_t cobs_rejections = 0;
    std::uint32_t crc_rejections = 0;
    std::uint32_t format_rejections = 0;
    std::uint32_t payload_rejections = 0;
    std::uint32_t overflow_rejections = 0;
    std::uint32_t unexpected_packets = 0;
};

class UartRadarSource final : public radar::RadarSource {
public:
    explicit UartRadarSource(HardwareSerial& serial) : serial_(serial) {}

    void begin();
    bool poll(radar::RadarFrame& frame) override;

    [[nodiscard]] const UartRadarStats& stats() const { return stats_; }
    [[nodiscard]] bool hasDescriptor() const { return has_descriptor_; }
    [[nodiscard]] const radar::protocol::ScannerDescriptor& descriptor() const {
        return descriptor_;
    }
    [[nodiscard]] bool connected(std::uint32_t now_ms) const;
    [[nodiscard]] std::uint32_t lastPacketAgeMs(std::uint32_t now_ms) const;

private:
    bool handlePacket(const radar::protocol::DecodedPacket& packet, radar::RadarFrame& frame);
    bool recordPacketSequence(std::uint32_t sequence);
    bool recordFrameSequence(std::uint32_t sequence);
    void sendHelloAck(std::uint32_t session_id);

    HardwareSerial& serial_;
    radar::protocol::RadarLinkStreamDecoder decoder_{};
    radar::protocol::SequenceTracker packet_sequence_{};
    radar::protocol::SequenceTracker frame_sequence_{};
    radar::protocol::ScannerDescriptor descriptor_{};
    UartRadarStats stats_{};
    std::uint32_t next_tx_packet_sequence_ = 0;
    std::uint32_t last_valid_packet_ms_ = 0;
    bool received_packet_ = false;
    bool has_descriptor_ = false;
};

}  // namespace n203::firmware::display_s3
