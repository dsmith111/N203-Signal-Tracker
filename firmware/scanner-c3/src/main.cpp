#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "board_config.hpp"
#include "mpu6050_motion_provider.hpp"
#include "n203/radar/protocol/radar_link.hpp"
#if (defined(N203_RD03D_CAPTURE) + defined(N203_RD03D_RANGE_LIVE) + \
     defined(N203_RD03D_POSITION_VERIFY) + \
     defined(N203_RD03D_POSITION_LIVE) + \
     defined(N203_C4001_CAPTURE) + \
     defined(N203_C4001_LIVE) + defined(N203_C4001_RANGE_LIVE)) > 1
#error "Select only one radar backend"
#endif
#if defined(N203_RD03D_CAPTURE) || defined(N203_RD03D_RANGE_LIVE) || \
    defined(N203_RD03D_POSITION_VERIFY) || defined(N203_RD03D_POSITION_LIVE)
#define N203_RD03D_BACKEND 1
#endif
#if defined(N203_C4001_CAPTURE) || defined(N203_C4001_LIVE) || \
    defined(N203_C4001_RANGE_LIVE)
#define N203_C4001_BACKEND 1
#endif
#if defined(N203_RD03D_CAPTURE) || defined(N203_C4001_CAPTURE)
#define N203_SENSOR_CAPTURE 1
#endif
#if defined(N203_RD03D_BACKEND) || defined(N203_C4001_BACKEND)
#define N203_PHYSICAL_SENSOR 1
#endif
#if defined(N203_C4001_BACKEND)
#include "n203/radar/c4001/report_stream.hpp"
#endif
#if defined(N203_C4001_LIVE)
#include "n203/radar/c4001/presence_pipeline.hpp"
#endif
#if defined(N203_C4001_RANGE_LIVE)
#include "n203/radar/c4001/range_pipeline.hpp"
#endif
#if defined(N203_RD03D_BACKEND)
#include "n203/radar/rd03d/raw_frame_stream.hpp"
#endif
#if defined(N203_RD03D_RANGE_LIVE) || defined(N203_RD03D_POSITION_VERIFY) || \
    defined(N203_RD03D_POSITION_LIVE)
#include "n203/radar/rd03d/range_pipeline.hpp"
#endif
#if defined(N203_RD03D_POSITION_LIVE)
#include "n203/radar/rd03d/position_tracker.hpp"
#endif
#include "n203/radar/simulated_radar_source.hpp"

namespace {

namespace board = n203::firmware::scanner_c3::board;
namespace protocol = n203::radar::protocol;

constexpr std::size_t kUartBufferSize = 512U;
constexpr std::size_t kMaximumLinkRxBytesPerLoop = 256U;
constexpr std::size_t kMaximumRadarRxBytesPerLoop = 128U;
constexpr std::uint32_t kHelloRetryMs = 500U;
#if defined(N203_PHYSICAL_SENSOR)
// Keep the negotiated link alive even when a physical sensor is silent or its
// reports are being rejected.
constexpr std::uint32_t kHelloConnectedMs = 1000U;
#else
constexpr std::uint32_t kHelloConnectedMs = 5000U;
#endif
constexpr std::uint32_t kAckTimeoutMs = 7500U;
#if defined(N203_C4001_CAPTURE)
constexpr std::uint32_t kDiagnosticsIntervalMs = 5000U;
#else
constexpr std::uint32_t kDiagnosticsIntervalMs = 1000U;
#endif
constexpr std::uint32_t kI2cProbeIntervalMs = 5U;
#if defined(N203_RD03D_BACKEND)
constexpr std::uint32_t kRadarUartBaud = board::kRd03dUartBaud;
#elif defined(N203_C4001_BACKEND)
constexpr std::uint32_t kRadarUartBaud = board::kC4001UartBaud;
#else
constexpr std::uint32_t kRadarUartBaud = board::kRadarInitialBaud;
#endif
#if defined(N203_SENSOR_CAPTURE)
constexpr std::uint32_t kCaptureBatchItems = 32U;
#if defined(N203_C4001_CAPTURE)
// Default to a quiet monitor. Raw reports remain available on demand with `c`.
constexpr std::uint32_t kInitialCaptureItems = 0U;
#else
constexpr std::uint32_t kInitialCaptureItems = kCaptureBatchItems;
#endif
#endif
#if defined(N203_RD03D_POSITION_VERIFY)
constexpr std::uint32_t kPositionVerificationBatchFrames = 8U;
#endif

struct RuntimeStats {
    std::uint32_t link_tx_bytes = 0;
    std::uint32_t link_tx_packets = 0;
    std::uint32_t link_tx_frames = 0;
    std::uint32_t link_tx_drops = 0;
    std::uint32_t hello_sent = 0;
    std::uint32_t hello_ack_received = 0;
    std::uint32_t hello_ack_timeouts = 0;
    std::uint32_t link_rx_bytes = 0;
    std::uint32_t link_rx_packets = 0;
    std::uint32_t link_rx_duplicates = 0;
    std::uint32_t link_rx_gaps = 0;
    std::uint32_t link_rx_resets = 0;
    std::uint32_t cobs_rejections = 0;
    std::uint32_t crc_rejections = 0;
    std::uint32_t format_rejections = 0;
    std::uint32_t payload_rejections = 0;
    std::uint32_t overflow_rejections = 0;
    std::uint32_t unexpected_packets = 0;
    std::uint32_t radar_rx_bytes = 0;
    std::uint32_t i2c_devices = 0;
    std::uint32_t i2c_errors = 0;
};

// Arduino owns these controller objects globally. Reusing them avoids having
// two HardwareSerial C++ objects refer to the same ESP-IDF UART driver.
HardwareSerial& link_uart = Serial1;
HardwareSerial& radar_uart = Serial0;
n203::radar::SimulatedRadarSource radar_source;
n203::firmware::scanner_c3::Mpu6050MotionProvider imu_provider;
#if defined(N203_C4001_BACKEND)
n203::radar::c4001::ReportStream c4001_stream;
#endif
#if defined(N203_RD03D_BACKEND)
n203::radar::rd03d::RawFrameStream rd03d_stream;
#endif
protocol::RadarLinkStreamDecoder link_decoder;
protocol::SequenceTracker link_rx_sequence;
protocol::ScannerDescriptor descriptor;
RuntimeStats stats;
n203::radar::RadarFrame latest_frame;

std::uint32_t next_tx_packet_sequence = 0;
std::uint32_t next_hello_ms = 0;
std::uint32_t last_ack_ms = 0;
std::uint32_t diagnostics_started_ms = 0;
std::uint32_t loop_worst_us = 0;
std::uint32_t last_imu_diagnostic_ms = 0U;
std::uint32_t last_imu_diagnostic_samples = 0U;
std::size_t last_phase_index = std::numeric_limits<std::size_t>::max();
std::uint8_t next_i2c_address = 1U;
std::uint32_t next_i2c_probe_ms = 0;
bool hello_acked = false;
bool received_ack = false;
bool i2c_scan_complete = false;
bool link_pins_configured = false;
bool radar_pins_configured = false;
bool link_rx_delivery_configured = false;
bool radar_rx_delivery_configured = false;
n203::firmware::scanner_c3::Mpu6050State last_imu_state =
    n203::firmware::scanner_c3::Mpu6050State::kDisabled;
volatile std::uint32_t link_rx_falling_edges = 0;
#if defined(N203_RD03D_RANGE_LIVE) || defined(N203_RD03D_POSITION_VERIFY) || \
    defined(N203_RD03D_POSITION_LIVE)
n203::radar::rd03d::RangeReport rd03d_latest_report;
std::uint32_t rd03d_valid_range_reports = 0U;
std::uint32_t rd03d_malformed_reports = 0U;
std::uint32_t rd03d_out_of_range_reports = 0U;
std::uint32_t rd03d_next_frame_sequence = 0U;
int rd03d_last_published_target_count = -1;
bool rd03d_frame_pending = false;
#if defined(N203_RD03D_POSITION_VERIFY) || defined(N203_RD03D_POSITION_LIVE)
bool rd03d_verbose_diagnostics = false;
#endif
#if defined(N203_RD03D_POSITION_VERIFY)
std::uint32_t rd03d_position_frames_remaining = 0U;
std::uint32_t rd03d_position_sample_index = 0U;
#elif defined(N203_RD03D_POSITION_LIVE)
n203::radar::rd03d::PositionTracker rd03d_position_tracker;
n203::radar::rd03d::TrackedPositionReport rd03d_latest_tracked_report;
#endif
#elif defined(N203_C4001_CAPTURE)
std::uint32_t c4001_recognized_reports = 0U;
int c4001_last_presence = -1;
bool c4001_valid_report_seen = false;
bool c4001_verbose_diagnostics = false;
#elif defined(N203_C4001_LIVE)
n203::radar::c4001::PresenceFilter c4001_presence_filter;
std::uint32_t c4001_valid_presence_reports = 0U;
std::uint32_t c4001_malformed_reports = 0U;
std::uint32_t c4001_unsupported_reports = 0U;
std::uint32_t c4001_next_frame_sequence = 0U;
n203::radar::PresenceState c4001_last_published_presence =
    n203::radar::PresenceState::kUnknown;
bool c4001_valid_report_seen = false;
bool c4001_frame_pending = false;
#elif defined(N203_C4001_RANGE_LIVE)
n203::radar::c4001::RangeFilter c4001_range_filter;
std::uint32_t c4001_valid_range_reports = 0U;
std::uint32_t c4001_malformed_reports = 0U;
std::uint32_t c4001_unsupported_reports = 0U;
std::uint32_t c4001_out_of_range_reports = 0U;
std::uint32_t c4001_next_frame_sequence = 0U;
int c4001_last_published_target_count = -1;
bool c4001_valid_report_seen = false;
bool c4001_frame_pending = false;
#endif
#if defined(N203_SENSOR_CAPTURE)
std::uint32_t capture_items_remaining = kInitialCaptureItems;
#endif

void ARDUINO_ISR_ATTR onLinkRxFallingEdge() {
    ++link_rx_falling_edges;
}

template <std::size_t Size>
void setText(std::array<char, Size>& destination, const char* value) {
    destination.fill('\0');
    for (std::size_t index = 0;
         index + 1U < Size && value[index] != '\0'; ++index) {
        destination[index] = value[index];
    }
}

const char* presenceName(n203::radar::PresenceState presence) {
    switch (presence) {
        case n203::radar::PresenceState::kUnknown: return "unknown";
        case n203::radar::PresenceState::kClear: return "clear";
        case n203::radar::PresenceState::kDetected: return "detected";
    }
    return "invalid";
}

const char* motionName(n203::radar::MotionState motion) {
    switch (motion) {
        case n203::radar::MotionState::kUnknown: return "unknown";
        case n203::radar::MotionState::kTracking: return "tracking";
        case n203::radar::MotionState::kDopplerWarning: return "doppler-warning";
        case n203::radar::MotionState::kStabilizing: return "stabilizing";
    }
    return "invalid";
}

void configureDescriptor() {
    descriptor = {};
    descriptor.session_id = esp_random();
    if (descriptor.session_id == 0U) {
        descriptor.session_id = 0x4E323033U;
    }
#if defined(N203_RD03D_CAPTURE)
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange | n203::radar::kPosition |
        n203::radar::kAzimuth | n203::radar::kVelocity |
        n203::radar::kMultipleContacts;
    descriptor.maximum_range_mm = 8000U;
    descriptor.horizontal_fov_cdeg = 12000U;
    descriptor.maximum_contacts = 3U;
    setText(descriptor.scanner_firmware, "c3-rd03d-cap");
    setText(descriptor.radar_type, "rd-03d/raw");
#elif defined(N203_RD03D_RANGE_LIVE)
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange |
        n203::radar::kVelocity | n203::radar::kMultipleContacts;
    descriptor.maximum_range_mm = n203::radar::rd03d::kMaximumRangeMm;
    descriptor.horizontal_fov_cdeg = 0U;
    descriptor.maximum_contacts =
        static_cast<std::uint8_t>(n203::radar::rd03d::kMaximumTargets);
    setText(descriptor.scanner_firmware, "c3-rd03d-range");
    setText(descriptor.radar_type, "rd-03d/range");
#elif defined(N203_RD03D_POSITION_VERIFY)
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange |
        n203::radar::kPosition | n203::radar::kAzimuth |
        n203::radar::kVelocity | n203::radar::kMultipleContacts;
    descriptor.maximum_range_mm = n203::radar::rd03d::kMaximumRangeMm;
    descriptor.horizontal_fov_cdeg = 12000U;
    descriptor.maximum_contacts =
        static_cast<std::uint8_t>(n203::radar::rd03d::kMaximumTargets);
    setText(descriptor.scanner_firmware, "c3-rd03d-pos-v");
    setText(descriptor.radar_type, "rd-03d/xy-v");
#elif defined(N203_RD03D_POSITION_LIVE)
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange |
        n203::radar::kPosition | n203::radar::kAzimuth |
        n203::radar::kVelocity | n203::radar::kTrackIdentity |
        n203::radar::kMultipleContacts;
    descriptor.maximum_range_mm = n203::radar::rd03d::kMaximumRangeMm;
    descriptor.horizontal_fov_cdeg = 12000U;
    descriptor.maximum_contacts =
        static_cast<std::uint8_t>(n203::radar::rd03d::kMaximumTargets);
    setText(descriptor.scanner_firmware, "c3-rd03d-pos");
    setText(descriptor.radar_type, "rd-03d/track");
#elif defined(N203_C4001_CAPTURE)
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange | n203::radar::kVelocity;
    descriptor.maximum_range_mm = 25000U;
    descriptor.horizontal_fov_cdeg = 10000U;
    descriptor.maximum_contacts = 1U;
    setText(descriptor.scanner_firmware, "c3-c4001-cap");
    setText(descriptor.radar_type, "c4001/raw");
#elif defined(N203_C4001_LIVE)
    descriptor.capabilities_flags = n203::radar::kPresence;
    descriptor.maximum_range_mm = 16000U;
    descriptor.horizontal_fov_cdeg = 10000U;
    descriptor.maximum_contacts = 0U;
    setText(descriptor.scanner_firmware, "c3-c4001-live");
    setText(descriptor.radar_type, "c4001/presence");
#elif defined(N203_C4001_RANGE_LIVE)
    descriptor.capabilities_flags =
        n203::radar::kRange | n203::radar::kVelocity;
    descriptor.maximum_range_mm = 25000U;
    descriptor.horizontal_fov_cdeg = 0U;
    descriptor.maximum_contacts = 1U;
    setText(descriptor.scanner_firmware, "c3-c4001-range");
    setText(descriptor.radar_type, "c4001/range");
#else
    descriptor.capabilities_flags =
        n203::radar::kPresence | n203::radar::kRange | n203::radar::kPosition |
        n203::radar::kAzimuth | n203::radar::kVelocity |
        n203::radar::kTrackIdentity | n203::radar::kMultipleContacts;
    descriptor.maximum_range_mm = 20000U;
    descriptor.horizontal_fov_cdeg = 8000U;
    descriptor.maximum_contacts =
        static_cast<std::uint8_t>(n203::radar::kMaximumContacts);
    setText(descriptor.scanner_firmware, "c3-sim-v0.1");
    setText(descriptor.radar_type, "simulated");
#endif
}

bool writeLinkPacket(const std::uint8_t* packet, std::size_t packet_size) {
    if (link_uart.availableForWrite() < static_cast<int>(packet_size)) {
        ++stats.link_tx_drops;
        return false;
    }
    const std::size_t written = link_uart.write(packet, packet_size);
    stats.link_tx_bytes += static_cast<std::uint32_t>(written);
    if (written != 0U) {
        ++next_tx_packet_sequence;
    }
    if (written != packet_size) {
        ++stats.link_tx_drops;
        return false;
    }
    ++stats.link_tx_packets;
    return true;
}

void sendHello(std::uint32_t now_ms) {
    std::array<std::uint8_t, protocol::kMaximumPacketSize> packet{};
    std::size_t packet_size = 0;
    if (protocol::encodeHelloPacket(descriptor, next_tx_packet_sequence, now_ms,
                                    packet.data(), packet.size(), packet_size) !=
        protocol::CodecError::kNone) {
        ++stats.link_tx_drops;
        return;
    }
    if (writeLinkPacket(packet.data(), packet_size)) {
        ++stats.hello_sent;
    }
}

bool sendRadarFrame(const n203::radar::RadarFrame& frame, std::uint32_t now_ms) {
    std::array<std::uint8_t, protocol::kMaximumPacketSize> packet{};
    std::size_t packet_size = 0;
    if (protocol::encodeRadarFramePacket(frame, next_tx_packet_sequence, now_ms,
                                         packet.data(), packet.size(), packet_size) !=
        protocol::CodecError::kNone) {
        ++stats.link_tx_drops;
        return false;
    }
    if (writeLinkPacket(packet.data(), packet_size)) {
        ++stats.link_tx_frames;
        return true;
    }
    return false;
}

void recordLinkRxSequence(std::uint32_t sequence) {
    const protocol::SequenceUpdate update = link_rx_sequence.observe(sequence);
    switch (update.event) {
        case protocol::SequenceEvent::kFirst:
        case protocol::SequenceEvent::kInOrder:
            break;
        case protocol::SequenceEvent::kGap:
            stats.link_rx_gaps += update.missed;
            break;
        case protocol::SequenceEvent::kDuplicate:
            ++stats.link_rx_duplicates;
            break;
        case protocol::SequenceEvent::kRestart:
            ++stats.link_rx_resets;
            break;
    }
}

void handleLinkPacket(const protocol::DecodedPacket& packet, std::uint32_t now_ms) {
    ++stats.link_rx_packets;
    recordLinkRxSequence(packet.metadata.packet_sequence);
    if (packet.metadata.type != protocol::PacketType::kHelloAck ||
        packet.hello_ack.scanner_session_id != descriptor.session_id ||
        packet.hello_ack.accepted_protocol_version != protocol::kProtocolVersion) {
        ++stats.unexpected_packets;
        return;
    }

    const bool newly_connected = !hello_acked;
    hello_acked = true;
    received_ack = true;
    last_ack_ms = now_ms;
    ++stats.hello_ack_received;
    if (newly_connected
#if defined(N203_C4001_CAPTURE)
        && c4001_verbose_diagnostics
#endif
    ) {
        Serial.printf("[link] HelloAck accepted for session=%08lX\n",
                      static_cast<unsigned long>(descriptor.session_id));
    }
}

void serviceLinkRx(std::uint32_t now_ms) {
    protocol::DecodedPacket packet{};
    std::size_t processed = 0;
    while (link_uart.available() > 0 && processed < kMaximumLinkRxBytesPerLoop) {
        const int value = link_uart.read();
        if (value < 0) {
            break;
        }
        ++processed;
        ++stats.link_rx_bytes;
        switch (link_decoder.push(static_cast<std::uint8_t>(value), packet)) {
            case protocol::StreamEvent::kNone: break;
            case protocol::StreamEvent::kPacketReady: handleLinkPacket(packet, now_ms); break;
            case protocol::StreamEvent::kCobsRejected: ++stats.cobs_rejections; break;
            case protocol::StreamEvent::kCrcRejected: ++stats.crc_rejections; break;
            case protocol::StreamEvent::kFormatRejected: ++stats.format_rejections; break;
            case protocol::StreamEvent::kPayloadRejected: ++stats.payload_rejections; break;
            case protocol::StreamEvent::kOverflowRejected: ++stats.overflow_rejections; break;
        }
    }
}

void serviceLinkTx(std::uint32_t now_ms) {
    if (hello_acked && received_ack && now_ms - last_ack_ms > kAckTimeoutMs) {
        hello_acked = false;
        next_hello_ms = now_ms;
        ++stats.hello_ack_timeouts;
#if defined(N203_C4001_CAPTURE)
        if (c4001_verbose_diagnostics) {
#endif
            Serial.println("[link] HelloAck timeout; returning to fast negotiation");
#if defined(N203_C4001_CAPTURE)
        }
#endif
    }

    if (static_cast<std::int32_t>(now_ms - next_hello_ms) >= 0) {
        sendHello(now_ms);
        next_hello_ms = now_ms + (hello_acked ? kHelloConnectedMs : kHelloRetryMs);
    }

#if defined(N203_RD03D_RANGE_LIVE) || defined(N203_RD03D_POSITION_VERIFY) || \
    defined(N203_RD03D_POSITION_LIVE)
    if (hello_acked && rd03d_frame_pending &&
        sendRadarFrame(latest_frame, now_ms)) {
        rd03d_frame_pending = false;
    }
#elif defined(N203_C4001_LIVE) || defined(N203_C4001_RANGE_LIVE)
    if (hello_acked && c4001_frame_pending && sendRadarFrame(latest_frame, now_ms)) {
        c4001_frame_pending = false;
    }
#elif !defined(N203_PHYSICAL_SENSOR)
    radar_source.update(now_ms);
    n203::radar::RadarFrame frame{};
    if (radar_source.poll(frame)) {
        latest_frame = frame;
        sendRadarFrame(frame, now_ms);
    }
#endif
}

#if defined(N203_RD03D_CAPTURE)
void printRawFrame(const char* kind, std::uint32_t sequence,
                   const n203::radar::rd03d::RawFrame& frame) {
    constexpr char kHex[] = "0123456789ABCDEF";
    std::array<char, n203::radar::rd03d::kRawFrameSize * 2U + 1U> encoded{};
    for (std::size_t index = 0; index < frame.size(); ++index) {
        encoded[index * 2U] = kHex[frame[index] >> 4U];
        encoded[index * 2U + 1U] = kHex[frame[index] & 0x0FU];
    }
    Serial.printf("[rd03d.%s] frame=%lu data=%s\n", kind,
                  static_cast<unsigned long>(sequence), encoded.data());
}

void handleRd03dEvent(n203::radar::rd03d::StreamEvent event,
                      const n203::radar::rd03d::RawFrame& frame) {
    if (event == n203::radar::rd03d::StreamEvent::kNone ||
        capture_items_remaining == 0U) {
        return;
    }
    const auto& capture_stats = rd03d_stream.stats();
    printRawFrame(event == n203::radar::rd03d::StreamEvent::kFrameReady
                      ? "raw"
                      : "reject",
                  event == n203::radar::rd03d::StreamEvent::kFrameReady
                      ? capture_stats.frames
                      : capture_stats.rejected_frames,
                  frame);
    --capture_items_remaining;
    if (capture_items_remaining == 0U) {
        Serial.println("[rd03d] capture batch complete; type c to capture 32 more frames");
    }
}
#endif

#if defined(N203_RD03D_RANGE_LIVE) || defined(N203_RD03D_POSITION_VERIFY) || \
    defined(N203_RD03D_POSITION_LIVE)
const n203::radar::rd03d::TargetMeasurement* firstPresentRd03dTarget(
    const n203::radar::rd03d::RangeReport& report) {
    for (const auto& target : report.targets) {
        if (target.present) {
            return &target;
        }
    }
    return nullptr;
}

#if defined(N203_RD03D_POSITION_VERIFY)
const n203::radar::Contact* contactForRd03dSlot(std::uint8_t slot) {
    const std::uint8_t contact_id = static_cast<std::uint8_t>(slot + 1U);
    for (std::size_t index = 0U; index < latest_frame.contact_count; ++index) {
        if (latest_frame.contacts[index].id == contact_id) {
            return &latest_frame.contacts[index];
        }
    }
    return nullptr;
}

void printRd03dPositionSample(
    const n203::radar::rd03d::RangeReport& report) {
    ++rd03d_position_sample_index;
    if (report.target_count == 0U) {
        Serial.printf("[rd03d.xy] sample=%lu contacts=0\n",
                      static_cast<unsigned long>(
                          rd03d_position_sample_index));
        return;
    }

    for (const auto& target : report.targets) {
        if (!target.present) {
            continue;
        }
        const auto* contact = contactForRd03dSlot(target.slot);
        Serial.printf(
            "[rd03d.xy] sample=%lu contacts=%u slot=%u native_x_mm=%d "
            "x_mm=%d y_mm=%d range_mm=%u azimuth_cdeg=%d "
            "velocity_mm_s=%d resolution_mm=%u\n",
            static_cast<unsigned long>(rd03d_position_sample_index),
            static_cast<unsigned int>(report.target_count),
            static_cast<unsigned int>(target.slot + 1U),
            static_cast<int>(target.x_mm),
            static_cast<int>(contact != nullptr ? contact->x_mm : 0),
            static_cast<int>(target.y_mm),
            static_cast<unsigned int>(target.range_mm),
            static_cast<int>(contact != nullptr ? contact->azimuth_cdeg : 0),
            static_cast<int>(target.radial_velocity_mm_s),
            static_cast<unsigned int>(target.distance_resolution_mm));
    }
}
#endif

void handleRd03dRangeEvent(
    n203::radar::rd03d::StreamEvent event,
    const n203::radar::rd03d::RawFrame& frame,
    std::uint32_t now_ms) {
    if (event != n203::radar::rd03d::StreamEvent::kFrameReady) {
        return;
    }

    n203::radar::rd03d::RangeReport report{};
    switch (n203::radar::rd03d::decodeRangeReport(frame, report)) {
        case n203::radar::rd03d::RangeReportError::kNone:
            ++rd03d_valid_range_reports;
            break;
        case n203::radar::rd03d::RangeReportError::kMalformedFrame:
            ++rd03d_malformed_reports;
            return;
        case n203::radar::rd03d::RangeReportError::kOutOfRange:
            ++rd03d_out_of_range_reports;
            return;
    }

    rd03d_latest_report = report;
#if defined(N203_RD03D_POSITION_VERIFY)
    latest_frame = n203::radar::rd03d::makePositionFrame(
        report, rd03d_next_frame_sequence++, now_ms);
#elif defined(N203_RD03D_POSITION_LIVE)
    rd03d_latest_tracked_report = rd03d_position_tracker.update(report, now_ms);
    latest_frame = n203::radar::rd03d::makeTrackedPositionFrame(
        rd03d_latest_tracked_report, rd03d_next_frame_sequence++, now_ms);
#else
    latest_frame = n203::radar::rd03d::makeRangeFrame(
        report, rd03d_next_frame_sequence++, now_ms);
#endif
    rd03d_frame_pending = true;
#if defined(N203_RD03D_POSITION_VERIFY)
    if (rd03d_position_frames_remaining > 0U) {
        printRd03dPositionSample(report);
        --rd03d_position_frames_remaining;
        if (rd03d_position_frames_remaining == 0U) {
            Serial.println(
                "[rd03d.xy] batch complete; reposition target and type p for "
                "8 more frames");
        }
    }
#endif
    const int published_target_count =
#if defined(N203_RD03D_POSITION_LIVE)
        static_cast<int>(rd03d_latest_tracked_report.target_count);
#else
        static_cast<int>(report.target_count);
#endif
    if (published_target_count !=
        rd03d_last_published_target_count) {
        rd03d_last_published_target_count = published_target_count;
        const auto* first = firstPresentRd03dTarget(report);
#if defined(N203_RD03D_POSITION_VERIFY)
        const auto* first_contact = first != nullptr
            ? contactForRd03dSlot(first->slot)
            : nullptr;
        Serial.printf(
            "[rd03d.position] contacts=%u first_native_x_mm=%d first_x_mm=%d "
            "first_y_mm=%d first_range_mm=%u first_azimuth_cdeg=%d "
            "first_velocity_mm_s=%d first_resolution_mm=%u\n",
            static_cast<unsigned int>(report.target_count),
            static_cast<int>(first != nullptr ? first->x_mm : 0),
            static_cast<int>(
                first_contact != nullptr ? first_contact->x_mm : 0),
            static_cast<int>(first != nullptr ? first->y_mm : 0),
            static_cast<unsigned int>(first != nullptr ? first->range_mm : 0U),
            static_cast<int>(first_contact != nullptr
                                 ? first_contact->azimuth_cdeg
                                 : 0),
            static_cast<int>(
                first != nullptr ? first->radial_velocity_mm_s : 0),
            static_cast<unsigned int>(
                first != nullptr ? first->distance_resolution_mm : 0U));
#elif defined(N203_RD03D_POSITION_LIVE)
        const auto* first_track =
            rd03d_latest_tracked_report.target_count > 0U
                ? &rd03d_latest_tracked_report.targets[0]
                : nullptr;
        Serial.printf(
            "[rd03d.track] contacts=%u observed=%u first_id=%u "
            "first_x_mm=%d first_y_mm=%d first_range_mm=%u "
            "first_confidence=%u\n",
            static_cast<unsigned int>(
                rd03d_latest_tracked_report.target_count),
            static_cast<unsigned int>(
                rd03d_latest_tracked_report.observed_count),
            static_cast<unsigned int>(
                first_track != nullptr ? first_track->id : 0U),
            static_cast<int>(first_track != nullptr ? first_track->x_mm : 0),
            static_cast<int>(first_track != nullptr ? first_track->y_mm : 0),
            static_cast<unsigned int>(
                first_track != nullptr ? first_track->range_mm : 0U),
            static_cast<unsigned int>(
                first_track != nullptr ? first_track->confidence : 0U));
#else
        Serial.printf(
            "[rd03d.range] contacts=%u first_range_mm=%u "
            "first_velocity_mm_s=%d first_resolution_mm=%u\n",
            static_cast<unsigned int>(report.target_count),
            static_cast<unsigned int>(first != nullptr ? first->range_mm : 0U),
            static_cast<int>(
                first != nullptr ? first->radial_velocity_mm_s : 0),
            static_cast<unsigned int>(
                first != nullptr ? first->distance_resolution_mm : 0U));
#endif
    }
}
#endif

#if defined(N203_C4001_CAPTURE)
bool reportStartsWith(const n203::radar::c4001::Report& report, const char* prefix) {
    std::size_t index = 0U;
    while (prefix[index] != '\0') {
        if (index >= report.size || report.text[index] != prefix[index]) {
            return false;
        }
        ++index;
    }
    return true;
}

void handleC4001Event(n203::radar::c4001::StreamEvent event,
                      const n203::radar::c4001::Report& report) {
    if (event == n203::radar::c4001::StreamEvent::kNone) {
        return;
    }

    const auto& capture_stats = c4001_stream.stats();
    if (event == n203::radar::c4001::StreamEvent::kReportReady) {
        const bool presence_report = reportStartsWith(report, "$DFHPD,");
        const bool recognized = presence_report || reportStartsWith(report, "$DFDMD,");
        if (recognized) {
            ++c4001_recognized_reports;
            if (!c4001_valid_report_seen) {
                c4001_valid_report_seen = true;
                Serial.printf("[c4001] valid report; baud=%lu\n",
                              static_cast<unsigned long>(kRadarUartBaud));
            }
        }
        if (presence_report && report.size > 7U &&
            (report.text[7] == '0' || report.text[7] == '1')) {
            const int presence = report.text[7] == '1' ? 1 : 0;
            if (presence != c4001_last_presence) {
                c4001_last_presence = presence;
                if (presence != 0 || c4001_verbose_diagnostics) {
                    Serial.printf(
                        "[c4001.presence] state=%s out_sample=%d\n",
                        presence != 0 ? "detected" : "clear",
                        digitalRead(board::kPins.radar_aux_input));
                }
            }
        }
        if (capture_items_remaining > 0U) {
            Serial.printf("[c4001.raw] report=%lu out_sample=%d data=%.*s\n",
                          static_cast<unsigned long>(capture_stats.reports),
                          digitalRead(board::kPins.radar_aux_input),
                          static_cast<int>(report.size), report.text.data());
        }
    } else if (capture_items_remaining > 0U) {
        Serial.printf("[c4001.reject] overflow=%lu\n",
                      static_cast<unsigned long>(capture_stats.overflow_rejections));
    }

    if (capture_items_remaining == 0U) {
        return;
    }
    --capture_items_remaining;
    if (capture_items_remaining == 0U) {
        Serial.println(
            "[c4001] raw capture paused; positive detection logging remains active; "
            "type c for 32 raw reports");
    }
}

#elif defined(N203_C4001_LIVE)
void handleC4001Event(n203::radar::c4001::StreamEvent event,
                      const n203::radar::c4001::Report& report,
                      std::uint32_t now_ms) {
    if (event == n203::radar::c4001::StreamEvent::kNone) {
        return;
    }
    if (event == n203::radar::c4001::StreamEvent::kOverflowRejected) {
        ++c4001_malformed_reports;
        return;
    }

    n203::radar::PresenceState observed = n203::radar::PresenceState::kUnknown;
    switch (n203::radar::c4001::decodePresenceReport(report, observed)) {
        case n203::radar::c4001::PresenceReportError::kNone:
            ++c4001_valid_presence_reports;
            break;
        case n203::radar::c4001::PresenceReportError::kUnsupportedReport:
            ++c4001_unsupported_reports;
            return;
        case n203::radar::c4001::PresenceReportError::kMalformedReport:
            ++c4001_malformed_reports;
            return;
    }

    if (!c4001_valid_report_seen) {
        c4001_valid_report_seen = true;
        Serial.printf("[c4001] valid presence report; baud=%lu\n",
                      static_cast<unsigned long>(kRadarUartBaud));
    }

    n203::radar::PresenceState filtered = n203::radar::PresenceState::kUnknown;
    if (!c4001_presence_filter.observe(observed, filtered)) {
        return;
    }

    latest_frame = n203::radar::c4001::makePresenceFrame(
        filtered, c4001_next_frame_sequence++, now_ms);
    c4001_frame_pending = true;
    if (filtered != c4001_last_published_presence) {
        c4001_last_published_presence = filtered;
        Serial.printf("[c4001.live] presence=%s sequence=%lu out_sample=%d\n",
                      presenceName(filtered),
                      static_cast<unsigned long>(latest_frame.sequence),
                      digitalRead(board::kPins.radar_aux_input));
    }
}

#elif defined(N203_C4001_RANGE_LIVE)
void handleC4001Event(n203::radar::c4001::StreamEvent event,
                      const n203::radar::c4001::Report& report,
                      std::uint32_t now_ms) {
    if (event == n203::radar::c4001::StreamEvent::kNone) {
        return;
    }
    if (event == n203::radar::c4001::StreamEvent::kOverflowRejected) {
        ++c4001_malformed_reports;
        return;
    }

    n203::radar::c4001::RangeMeasurement observed{};
    switch (n203::radar::c4001::decodeRangeReport(report, observed)) {
        case n203::radar::c4001::RangeReportError::kNone:
            ++c4001_valid_range_reports;
            break;
        case n203::radar::c4001::RangeReportError::kUnsupportedReport:
            ++c4001_unsupported_reports;
            return;
        case n203::radar::c4001::RangeReportError::kMalformedReport:
            ++c4001_malformed_reports;
            return;
        case n203::radar::c4001::RangeReportError::kOutOfRange:
            ++c4001_out_of_range_reports;
            return;
    }

    if (!c4001_valid_report_seen) {
        c4001_valid_report_seen = true;
        Serial.printf("[c4001] valid range report; baud=%lu\n",
                      static_cast<unsigned long>(kRadarUartBaud));
    }

    n203::radar::c4001::RangeMeasurement filtered{};
    if (!c4001_range_filter.observe(observed, filtered)) {
        return;
    }

    latest_frame = n203::radar::c4001::makeRangeFrame(
        filtered, c4001_next_frame_sequence++, now_ms);
    c4001_frame_pending = true;
    if (static_cast<int>(filtered.target_count) !=
        c4001_last_published_target_count) {
        c4001_last_published_target_count = filtered.target_count;
        Serial.printf(
            "[c4001.range] contacts=%u range_mm=%u velocity_mm_s=%d signal=%u\n",
            static_cast<unsigned int>(filtered.target_count),
            static_cast<unsigned int>(filtered.range_mm),
            static_cast<int>(filtered.radial_velocity_mm_s),
            static_cast<unsigned int>(filtered.signal));
    }
}

#endif

#if defined(N203_SENSOR_CAPTURE)
void serviceUsbCommands() {
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value == 'c' || value == 'C') {
            capture_items_remaining = kCaptureBatchItems;
            Serial.println("[radar] capture armed for 32 reports/frames");
#if defined(N203_C4001_CAPTURE)
        } else if (value == 'q' || value == 'Q') {
            capture_items_remaining = 0U;
            Serial.println(
                "[c4001] raw capture paused; positive detection logging remains active");
        } else if (value == 'v' || value == 'V') {
            c4001_verbose_diagnostics = !c4001_verbose_diagnostics;
            Serial.printf("[c4001] verbose diagnostics=%s\n",
                          c4001_verbose_diagnostics ? "on" : "off");
#endif
        } else if (value == 'h' || value == 'H' || value == '?') {
#if defined(N203_C4001_CAPTURE)
            Serial.println(
                "[c4001] commands: c=32 raw reports, q=pause raw, "
                "v=verbose diagnostics, h=help");
#else
            Serial.println("[radar] commands: c=capture next 32 frames, h=help");
#endif
        }
    }
}
#endif

#if defined(N203_RD03D_POSITION_VERIFY)
void servicePositionVerificationCommands() {
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value == 'p' || value == 'P') {
            rd03d_position_frames_remaining =
                kPositionVerificationBatchFrames;
            rd03d_position_sample_index = 0U;
            Serial.println(
                "[rd03d.xy] capture armed for 8 position frames");
        } else if (value == 'q' || value == 'Q') {
            rd03d_position_frames_remaining = 0U;
            Serial.println("[rd03d.xy] capture cancelled");
        } else if (value == 'v' || value == 'V') {
            rd03d_verbose_diagnostics = !rd03d_verbose_diagnostics;
            Serial.printf("[rd03d] verbose diagnostics=%s\n",
                          rd03d_verbose_diagnostics ? "on" : "off");
        } else if (value == 'h' || value == 'H' || value == '?') {
            Serial.println(
                "[rd03d] commands: p=8 X/Y frames, q=cancel capture, "
                "v=verbose diagnostics, h=help");
        }
    }
}
#endif

#if defined(N203_RD03D_POSITION_LIVE)
void serviceRd03dPositionLiveCommands() {
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value == 'v' || value == 'V') {
            rd03d_verbose_diagnostics = !rd03d_verbose_diagnostics;
            Serial.printf("[rd03d] verbose diagnostics=%s\n",
                          rd03d_verbose_diagnostics ? "on" : "off");
        } else if (value == 'h' || value == 'H' || value == '?') {
            Serial.println(
                "[rd03d] commands: v=verbose diagnostics, h=help");
        }
    }
}
#endif

void serviceRadarInput(std::uint32_t now_ms) {
    std::size_t processed = 0;
    while (radar_uart.available() > 0 && processed < kMaximumRadarRxBytesPerLoop) {
        const int value = radar_uart.read();
        if (value < 0) {
            break;
        }
        ++processed;
        ++stats.radar_rx_bytes;
#if defined(N203_RD03D_CAPTURE)
        n203::radar::rd03d::RawFrame frame{};
        handleRd03dEvent(
            rd03d_stream.push(static_cast<std::uint8_t>(value), frame), frame);
#elif defined(N203_RD03D_RANGE_LIVE) || defined(N203_RD03D_POSITION_VERIFY) || \
    defined(N203_RD03D_POSITION_LIVE)
        n203::radar::rd03d::RawFrame frame{};
        handleRd03dRangeEvent(
            rd03d_stream.push(static_cast<std::uint8_t>(value), frame), frame,
            now_ms);
#elif defined(N203_C4001_CAPTURE)
        n203::radar::c4001::Report report{};
        handleC4001Event(
            c4001_stream.push(static_cast<std::uint8_t>(value), report), report);
#elif defined(N203_C4001_LIVE) || defined(N203_C4001_RANGE_LIVE)
        n203::radar::c4001::Report report{};
        handleC4001Event(
            c4001_stream.push(static_cast<std::uint8_t>(value), report), report,
            now_ms);
#endif
    }
}

void serviceI2cScan(std::uint32_t now_ms) {
    if (i2c_scan_complete || static_cast<std::int32_t>(now_ms - next_i2c_probe_ms) < 0) {
        return;
    }
    Wire.beginTransmission(next_i2c_address);
    const std::uint8_t result = Wire.endTransmission(true);
    if (result == 0U) {
        ++stats.i2c_devices;
        Serial.printf("[i2c] device address=0x%02X\n",
                      static_cast<unsigned int>(next_i2c_address));
    } else if (result == 4U) {
        ++stats.i2c_errors;
    }

    ++next_i2c_address;
    next_i2c_probe_ms = now_ms + kI2cProbeIntervalMs;
    if (next_i2c_address >= 127U) {
        i2c_scan_complete = true;
        Serial.printf("[i2c] scan complete devices=%lu errors=%lu\n",
                      static_cast<unsigned long>(stats.i2c_devices),
                      static_cast<unsigned long>(stats.i2c_errors));
    }
}

void serviceImu(std::uint32_t now_ms) {
    imu_provider.service(now_ms);
    const auto state = imu_provider.state();
    if (state == last_imu_state) {
        return;
    }
    last_imu_state = state;
    const auto& metrics = imu_provider.metrics();
    Serial.printf("[imu] state=%s address=0x%02X\n",
                  n203::firmware::scanner_c3::mpu6050StateName(state),
                  static_cast<unsigned int>(imu_provider.address()));
    if (state == n203::firmware::scanner_c3::Mpu6050State::kCalibrating) {
        Serial.printf(
            "[imu] keep scanner carrier stationary for %u startup samples\n",
            static_cast<unsigned int>(
                n203::motion::MotionEstimator::kStartupBiasSamples));
    } else if (state == n203::firmware::scanner_c3::Mpu6050State::kReady) {
        Serial.printf(
            "[imu] bias complete gx=%.3f gy=%.3f gz=%.3f dps; "
            "telemetry only, display motion state unchanged\n",
            static_cast<double>(metrics.gyro_bias_x_dps),
            static_cast<double>(metrics.gyro_bias_y_dps),
            static_cast<double>(metrics.gyro_bias_z_dps));
    }
}

void printDiagnostics(std::uint32_t now_ms) {
    if (now_ms - diagnostics_started_ms < kDiagnosticsIntervalMs) {
        return;
    }

#if defined(N203_C4001_CAPTURE)
    if (!c4001_verbose_diagnostics) {
        diagnostics_started_ms = now_ms;
        loop_worst_us = 0U;
        return;
    }
#endif
#if defined(N203_RD03D_POSITION_VERIFY) || defined(N203_RD03D_POSITION_LIVE)
    if (!rd03d_verbose_diagnostics) {
        diagnostics_started_ms = now_ms;
        loop_worst_us = 0U;
        return;
    }
#endif

#if !defined(N203_PHYSICAL_SENSOR)
    if (radar_source.phaseIndex() != last_phase_index) {
        Serial.printf("[sim] phase=%s\n", radar_source.phaseName());
        last_phase_index = radar_source.phaseIndex();
    }
#endif
    const std::uint32_t ack_age_ms = received_ack
        ? now_ms - last_ack_ms
        : std::numeric_limits<std::uint32_t>::max();
    Serial.printf(
        "[link] connected=%s session=%08lX ack_age_ms=%lu tx_bytes=%lu packets=%lu "
        "frames=%lu drops=%lu hello=%lu ack=%lu timeouts=%lu rx_bytes=%lu rx_packets=%lu\n",
        hello_acked ? "yes" : "no", static_cast<unsigned long>(descriptor.session_id),
        static_cast<unsigned long>(ack_age_ms), static_cast<unsigned long>(stats.link_tx_bytes),
        static_cast<unsigned long>(stats.link_tx_packets),
        static_cast<unsigned long>(stats.link_tx_frames),
        static_cast<unsigned long>(stats.link_tx_drops),
        static_cast<unsigned long>(stats.hello_sent),
        static_cast<unsigned long>(stats.hello_ack_received),
        static_cast<unsigned long>(stats.hello_ack_timeouts),
        static_cast<unsigned long>(stats.link_rx_bytes),
        static_cast<unsigned long>(stats.link_rx_packets));
    Serial.printf(
        "[link] reject_cobs=%lu crc=%lu format=%lu payload=%lu overflow=%lu "
        "gap=%lu duplicate=%lu reset=%lu unexpected=%lu\n",
        static_cast<unsigned long>(stats.cobs_rejections),
        static_cast<unsigned long>(stats.crc_rejections),
        static_cast<unsigned long>(stats.format_rejections),
        static_cast<unsigned long>(stats.payload_rejections),
        static_cast<unsigned long>(stats.overflow_rejections),
        static_cast<unsigned long>(stats.link_rx_gaps),
        static_cast<unsigned long>(stats.link_rx_duplicates),
        static_cast<unsigned long>(stats.link_rx_resets),
        static_cast<unsigned long>(stats.unexpected_packets));
#if defined(N203_RD03D_CAPTURE)
    const auto& capture_stats = rd03d_stream.stats();
    Serial.printf(
        "[radar] mode=rd03d-capture baud=%lu raw_rx_bytes=%lu frames=%lu "
        "rejected=%lu capture_remaining=%lu\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.frames),
        static_cast<unsigned long>(capture_stats.rejected_frames),
        static_cast<unsigned long>(capture_items_remaining));
#elif defined(N203_RD03D_POSITION_VERIFY)
    const auto& capture_stats = rd03d_stream.stats();
    const auto* first = firstPresentRd03dTarget(rd03d_latest_report);
    const auto* first_contact = first != nullptr
        ? contactForRd03dSlot(first->slot)
        : nullptr;
    Serial.printf(
        "[radar] mode=rd03d-position-verify baud=%lu raw_rx_bytes=%lu "
        "frames=%lu rejected=%lu valid_position=%lu malformed=%lu "
        "out_of_range=%lu pending=%s contacts=%u first_native_x_mm=%d "
        "first_x_mm=%d first_y_mm=%d first_range_mm=%u "
        "first_azimuth_cdeg=%d "
        "first_velocity_mm_s=%d first_resolution_mm=%u seq=%lu\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.frames),
        static_cast<unsigned long>(capture_stats.rejected_frames),
        static_cast<unsigned long>(rd03d_valid_range_reports),
        static_cast<unsigned long>(rd03d_malformed_reports),
        static_cast<unsigned long>(rd03d_out_of_range_reports),
        rd03d_frame_pending ? "yes" : "no",
        static_cast<unsigned int>(rd03d_latest_report.target_count),
        static_cast<int>(first != nullptr ? first->x_mm : 0),
        static_cast<int>(
            first_contact != nullptr ? first_contact->x_mm : 0),
        static_cast<int>(first != nullptr ? first->y_mm : 0),
        static_cast<unsigned int>(first != nullptr ? first->range_mm : 0U),
        static_cast<int>(first_contact != nullptr
                             ? first_contact->azimuth_cdeg
                             : 0),
        static_cast<int>(
            first != nullptr ? first->radial_velocity_mm_s : 0),
        static_cast<unsigned int>(
            first != nullptr ? first->distance_resolution_mm : 0U),
        static_cast<unsigned long>(latest_frame.sequence));
#elif defined(N203_RD03D_POSITION_LIVE)
    const auto& capture_stats = rd03d_stream.stats();
    const auto* first_track = rd03d_latest_tracked_report.target_count > 0U
        ? &rd03d_latest_tracked_report.targets[0]
        : nullptr;
    Serial.printf(
        "[radar] mode=rd03d-position-live baud=%lu raw_rx_bytes=%lu "
        "frames=%lu rejected=%lu valid_position=%lu malformed=%lu "
        "out_of_range=%lu pending=%s observed=%u contacts=%u "
        "first_id=%u first_x_mm=%d first_y_mm=%d first_range_mm=%u "
        "first_confidence=%u seq=%lu\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.frames),
        static_cast<unsigned long>(capture_stats.rejected_frames),
        static_cast<unsigned long>(rd03d_valid_range_reports),
        static_cast<unsigned long>(rd03d_malformed_reports),
        static_cast<unsigned long>(rd03d_out_of_range_reports),
        rd03d_frame_pending ? "yes" : "no",
        static_cast<unsigned int>(rd03d_latest_tracked_report.observed_count),
        static_cast<unsigned int>(rd03d_latest_tracked_report.target_count),
        static_cast<unsigned int>(
            first_track != nullptr ? first_track->id : 0U),
        static_cast<int>(first_track != nullptr ? first_track->x_mm : 0),
        static_cast<int>(first_track != nullptr ? first_track->y_mm : 0),
        static_cast<unsigned int>(
            first_track != nullptr ? first_track->range_mm : 0U),
        static_cast<unsigned int>(
            first_track != nullptr ? first_track->confidence : 0U),
        static_cast<unsigned long>(latest_frame.sequence));
#elif defined(N203_RD03D_RANGE_LIVE)
    const auto& capture_stats = rd03d_stream.stats();
    const auto* first = firstPresentRd03dTarget(rd03d_latest_report);
    Serial.printf(
        "[radar] mode=rd03d-range-live baud=%lu raw_rx_bytes=%lu frames=%lu "
        "rejected=%lu valid_range=%lu malformed=%lu out_of_range=%lu "
        "pending=%s contacts=%u first_range_mm=%u first_velocity_mm_s=%d "
        "first_resolution_mm=%u seq=%lu\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.frames),
        static_cast<unsigned long>(capture_stats.rejected_frames),
        static_cast<unsigned long>(rd03d_valid_range_reports),
        static_cast<unsigned long>(rd03d_malformed_reports),
        static_cast<unsigned long>(rd03d_out_of_range_reports),
        rd03d_frame_pending ? "yes" : "no",
        static_cast<unsigned int>(rd03d_latest_report.target_count),
        static_cast<unsigned int>(first != nullptr ? first->range_mm : 0U),
        static_cast<int>(
            first != nullptr ? first->radial_velocity_mm_s : 0),
        static_cast<unsigned int>(
            first != nullptr ? first->distance_resolution_mm : 0U),
        static_cast<unsigned long>(latest_frame.sequence));
#elif defined(N203_C4001_CAPTURE)
    const auto& capture_stats = c4001_stream.stats();
    Serial.printf(
        "[radar] mode=c4001-capture baud=%lu raw_rx_bytes=%lu reports=%lu "
        "recognized=%lu valid_seen=%s overflow=%lu interrupted=%lu capture_remaining=%lu "
        "out_sample=%d\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.reports),
        static_cast<unsigned long>(c4001_recognized_reports),
        c4001_valid_report_seen ? "yes" : "no",
        static_cast<unsigned long>(capture_stats.overflow_rejections),
        static_cast<unsigned long>(capture_stats.interrupted_reports),
        static_cast<unsigned long>(capture_items_remaining),
        digitalRead(board::kPins.radar_aux_input));
#elif defined(N203_C4001_LIVE)
    const auto& capture_stats = c4001_stream.stats();
    Serial.printf(
        "[radar] mode=c4001-live baud=%lu raw_rx_bytes=%lu reports=%lu "
        "valid_presence=%lu malformed=%lu unsupported=%lu stream_overflow=%lu "
        "interrupted=%lu pending=%s presence=%s seq=%lu out_sample=%d\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.reports),
        static_cast<unsigned long>(c4001_valid_presence_reports),
        static_cast<unsigned long>(c4001_malformed_reports),
        static_cast<unsigned long>(c4001_unsupported_reports),
        static_cast<unsigned long>(capture_stats.overflow_rejections),
        static_cast<unsigned long>(capture_stats.interrupted_reports),
        c4001_frame_pending ? "yes" : "no", presenceName(latest_frame.presence),
        static_cast<unsigned long>(latest_frame.sequence),
        digitalRead(board::kPins.radar_aux_input));
#elif defined(N203_C4001_RANGE_LIVE)
    const auto& capture_stats = c4001_stream.stats();
    const auto& contact = latest_frame.contacts[0];
    Serial.printf(
        "[radar] mode=c4001-range-live baud=%lu raw_rx_bytes=%lu reports=%lu "
        "valid_range=%lu malformed=%lu unsupported=%lu out_of_range=%lu "
        "stream_overflow=%lu interrupted=%lu pending=%s contacts=%u "
        "range_mm=%u velocity_mm_s=%d signal=%u seq=%lu\n",
        static_cast<unsigned long>(kRadarUartBaud),
        static_cast<unsigned long>(stats.radar_rx_bytes),
        static_cast<unsigned long>(capture_stats.reports),
        static_cast<unsigned long>(c4001_valid_range_reports),
        static_cast<unsigned long>(c4001_malformed_reports),
        static_cast<unsigned long>(c4001_unsupported_reports),
        static_cast<unsigned long>(c4001_out_of_range_reports),
        static_cast<unsigned long>(capture_stats.overflow_rejections),
        static_cast<unsigned long>(capture_stats.interrupted_reports),
        c4001_frame_pending ? "yes" : "no",
        static_cast<unsigned int>(latest_frame.contact_count),
        static_cast<unsigned int>(latest_frame.contact_count == 0U
                                      ? 0U : contact.range_mm),
        static_cast<int>(latest_frame.contact_count == 0U
                             ? 0 : contact.radial_velocity_mm_s),
        static_cast<unsigned int>(latest_frame.contact_count == 0U
                                      ? 0U : contact.signal),
        static_cast<unsigned long>(latest_frame.sequence));
#else
    Serial.printf(
        "[radar] mode=simulated raw_rx_bytes=%lu presence=%s contacts=%u motion=%s quality=%u seq=%lu\n",
        static_cast<unsigned long>(stats.radar_rx_bytes), presenceName(latest_frame.presence),
        static_cast<unsigned int>(latest_frame.contact_count), motionName(latest_frame.motion),
        static_cast<unsigned int>(latest_frame.motion_quality),
        static_cast<unsigned long>(latest_frame.sequence));
#endif
    Serial.printf("[system] uptime_ms=%lu heap=%lu heap_min=%lu loop_worst_us=%lu\n",
                  static_cast<unsigned long>(now_ms),
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(ESP.getMinFreeHeap()),
                  static_cast<unsigned long>(loop_worst_us));
    Serial.printf(
        "[uart] link_controller=%d baud=%lu rx_pin=%d rx_level=%d edges=%lu pins=%s "
        "delivery=%s radar_controller=%d radar_baud=%lu radar_rx=%d radar_level=%d "
        "radar_pins=%s radar_delivery=%s\n",
                  board::kLinkUartNumber,
                  static_cast<unsigned long>(link_uart.baudRate()),
                  board::kPins.link_rx, digitalRead(board::kPins.link_rx),
                  static_cast<unsigned long>(link_rx_falling_edges),
                  link_pins_configured ? "ready" : "failed",
                  link_rx_delivery_configured ? "ready" : "failed",
                  board::kRadarUartNumber,
                  static_cast<unsigned long>(radar_uart.baudRate()),
                  board::kPins.radar_rx_candidate_a,
                  digitalRead(board::kPins.radar_rx_candidate_a),
                  radar_pins_configured ? "ready" : "failed",
                  radar_rx_delivery_configured ? "ready" : "failed");
    const auto& imu_stats = imu_provider.stats();
    const auto& imu_metrics = imu_provider.metrics();
    const std::uint32_t imu_elapsed_ms = now_ms - last_imu_diagnostic_ms;
    const std::uint32_t imu_sample_delta =
        imu_stats.samples - last_imu_diagnostic_samples;
    const float imu_sample_hz = imu_elapsed_ms == 0U
        ? 0.0F
        : static_cast<float>(imu_sample_delta) * 1000.0F /
              static_cast<float>(imu_elapsed_ms);
    const std::uint32_t imu_age_ms = imu_stats.samples == 0U
        ? std::numeric_limits<std::uint32_t>::max()
        : now_ms - imu_stats.last_sample_ms;
    Serial.printf(
        "[imu] state=%s address=0x%02X hz=%.1f samples=%lu errors=%lu/%lu "
        "disconnects=%lu reconnects=%lu age_ms=%lu calibrated=%s "
        "accel=%.3f,%.3f,%.3f g "
        "gyro=%.2f,%.2f,%.2f dps gyro_mag=%.2f dynamic_g=%.3f\n",
        n203::firmware::scanner_c3::mpu6050StateName(imu_provider.state()),
        static_cast<unsigned int>(imu_provider.address()),
        static_cast<double>(imu_sample_hz),
        static_cast<unsigned long>(imu_stats.samples),
        static_cast<unsigned long>(imu_stats.read_errors),
        static_cast<unsigned long>(imu_stats.write_errors),
        static_cast<unsigned long>(imu_stats.disconnects),
        static_cast<unsigned long>(imu_stats.reconnects),
        static_cast<unsigned long>(imu_age_ms),
        imu_metrics.calibrated ? "yes" : "no",
        static_cast<double>(imu_metrics.accel_x_g),
        static_cast<double>(imu_metrics.accel_y_g),
        static_cast<double>(imu_metrics.accel_z_g),
        static_cast<double>(imu_metrics.gyro_x_dps),
        static_cast<double>(imu_metrics.gyro_y_dps),
        static_cast<double>(imu_metrics.gyro_z_dps),
        static_cast<double>(imu_metrics.gyro_magnitude_dps),
        static_cast<double>(imu_metrics.dynamic_acceleration_g));
    last_imu_diagnostic_ms = now_ms;
    last_imu_diagnostic_samples = imu_stats.samples;
    diagnostics_started_ms = now_ms;
    loop_worst_us = 0U;
}

}  // namespace

void setup() {
    Serial.begin(board::kUsbDiagnosticBaud);
    const std::uint32_t serial_wait_started_ms = millis();
    while (!Serial && millis() - serial_wait_started_ms < 1500U) {
        delay(10);
    }

    configureDescriptor();
    pinMode(board::kPins.link_rx, INPUT_PULLUP);
    pinMode(board::kPins.radar_rx_candidate_a, INPUT_PULLUP);
#if defined(N203_C4001_BACKEND)
    pinMode(board::kPins.radar_aux_input, INPUT_PULLDOWN);
#else
    pinMode(board::kPins.radar_aux_input, INPUT_PULLUP);
#endif

    // Initialize UART0 first. On ESP32-C3 its ROM/default pins are GPIO20/21;
    // moving it to the radar pins may reset those pads. Apply the UART1 link
    // routing last so it remains authoritative.
    radar_uart.setRxBufferSize(kUartBufferSize);
    radar_uart.setTxBufferSize(kUartBufferSize);
    radar_uart.begin(kRadarUartBaud, SERIAL_8N1,
                     board::kPins.radar_rx_candidate_a, board::kPins.radar_tx);
    radar_pins_configured = radar_uart.setPins(
        board::kPins.radar_rx_candidate_a, board::kPins.radar_tx);
    radar_rx_delivery_configured =
        radar_uart.setRxFIFOFull(1U) && radar_uart.setRxTimeout(1U);
    link_uart.setRxBufferSize(kUartBufferSize);
    link_uart.setTxBufferSize(kUartBufferSize);
    link_uart.begin(board::kLinkUartBaud, SERIAL_8N1,
                    board::kPins.link_rx, board::kPins.link_tx);
    link_pins_configured = link_uart.setPins(
        board::kPins.link_rx, board::kPins.link_tx);
    link_rx_delivery_configured =
        link_uart.setRxFIFOFull(1U) && link_uart.setRxTimeout(1U);
    attachInterrupt(digitalPinToInterrupt(board::kPins.link_rx),
                    onLinkRxFallingEdge, FALLING);
    Wire.setTimeOut(10U);
    const bool i2c_started = Wire.begin(
        board::kPins.imu_sda, board::kPins.imu_scl, board::kI2cFrequencyHz);

    const std::uint32_t now_ms = millis();
#if !defined(N203_PHYSICAL_SENSOR)
    radar_source.begin(now_ms);
#endif
    diagnostics_started_ms = now_ms;
    next_hello_ms = now_ms;
    next_i2c_probe_ms = now_ms;
    last_imu_diagnostic_ms = now_ms;
    imu_provider.begin(Wire, i2c_started, now_ms);

    Serial.println();
#if defined(N203_RD03D_CAPTURE)
    Serial.println("N203 Signal Tracker - ESP32-C3 RD-03D raw capture");
#elif defined(N203_RD03D_POSITION_LIVE)
    Serial.println("N203 Signal Tracker - ESP32-C3 RD-03D live position");
#elif defined(N203_RD03D_POSITION_VERIFY)
    Serial.println("N203 Signal Tracker - ESP32-C3 RD-03D position verify");
#elif defined(N203_RD03D_RANGE_LIVE)
    Serial.println("N203 Signal Tracker - ESP32-C3 RD-03D live range");
#elif defined(N203_C4001_CAPTURE)
    Serial.println("N203 Signal Tracker - ESP32-C3 C4001 raw capture");
#elif defined(N203_C4001_LIVE)
    Serial.println("N203 Signal Tracker - ESP32-C3 C4001 live presence");
#elif defined(N203_C4001_RANGE_LIVE)
    Serial.println("N203 Signal Tracker - ESP32-C3 C4001 live range");
#else
    Serial.println("N203 Signal Tracker - ESP32-C3 simulated scanner");
#endif
    Serial.printf("[boot] board=%s reset_reason=%d protocol=v%u session=%08lX\n",
                  board::kBoardName, static_cast<int>(esp_reset_reason()),
                  protocol::kProtocolVersion,
                  static_cast<unsigned long>(descriptor.session_id));
    Serial.printf(
        "[pins] link_rx=%d link_tx=%d radar_rx=%d radar_aux=%d radar_tx=%d "
        "i2c_sda=%d i2c_scl=%d\n",
        board::kPins.link_rx, board::kPins.link_tx,
        board::kPins.radar_rx_candidate_a, board::kPins.radar_aux_input,
        board::kPins.radar_tx, board::kPins.imu_sda, board::kPins.imu_scl);
    Serial.printf(
        "[uart] link_controller=%d link=%lu pins=%s delivery=%s radar_controller=%d "
        "radar_baud=%lu pins=%s delivery=%s i2c=%s wireless=not-initialized\n",
                  board::kLinkUartNumber,
                  static_cast<unsigned long>(board::kLinkUartBaud),
                  link_pins_configured ? "ready" : "failed",
                  link_rx_delivery_configured ? "ready" : "failed",
                  board::kRadarUartNumber,
                  static_cast<unsigned long>(kRadarUartBaud),
                  radar_pins_configured ? "ready" : "failed",
                  radar_rx_delivery_configured ? "ready" : "failed",
                  i2c_started ? "ready" : "failed");
    Serial.printf(
        "[imu] provider=mpu6050 addresses=0x68,0x69 sample_hz=100 "
        "sda=%d scl=%d telemetry_only=yes\n",
        board::kPins.imu_sda, board::kPins.imu_scl);
#if defined(N203_RD03D_CAPTURE)
    Serial.println(
        "[rd03d] read-only capture; module RX must remain disconnected; "
        "type c for another 32 frames");
#elif defined(N203_RD03D_POSITION_LIVE)
    Serial.println(
        "[rd03d] tracked X/Y frames enabled; canonical positive X is right "
        "and positive Y is forward; stable software IDs and brief dropout "
        "retention are active; module RX must remain disconnected");
    Serial.println(
        "[rd03d] MPU telemetry is optional and is not used for contact "
        "tracking; quiet diagnostics; commands: v=verbose, h=help");
#elif defined(N203_RD03D_POSITION_VERIFY)
    Serial.println(
        "[rd03d] live X/Y verification enabled; verified native X inversion "
        "maps canonical positive X right; positive Y is forward; slot IDs are "
        "not stable tracks; module RX must remain disconnected");
    Serial.println(
        "[rd03d] quiet diagnostics; commands: p=8 X/Y frames, q=cancel, "
        "v=verbose, h=help");
#elif defined(N203_RD03D_RANGE_LIVE)
    Serial.println(
        "[rd03d] live derived-range frames enabled; UART input is read-only; "
        "X/Y display remains gated on physical capture verification");
#elif defined(N203_C4001_CAPTURE)
    Serial.println(
        "[c4001] read-only capture; sensor RX must remain disconnected; "
        "positive UART detections log automatically; GPIO5 OUT is sampled; "
        "commands: c=raw, q=pause raw, v=verbose");
#elif defined(N203_C4001_LIVE)
    Serial.println(
        "[c4001] live presence frames enabled; strict UART reports are authoritative; "
        "sensor RX remains disconnected; GPIO5 OUT is diagnostic only");
#elif defined(N203_C4001_RANGE_LIVE)
    Serial.println(
        "[c4001] live range frames enabled; strict $DFDMD reports are authoritative; "
        "bearing is unknown; sensor RX and GPIO5 OUT are not required");
#endif
}

void loop() {
    const std::uint32_t loop_started_us = micros();
    const std::uint32_t now_ms = millis();
    serviceLinkRx(now_ms);
#if defined(N203_SENSOR_CAPTURE)
    serviceUsbCommands();
#endif
#if defined(N203_RD03D_POSITION_VERIFY)
    servicePositionVerificationCommands();
#elif defined(N203_RD03D_POSITION_LIVE)
    serviceRd03dPositionLiveCommands();
#endif
    serviceRadarInput(now_ms);
    serviceI2cScan(now_ms);
    serviceImu(now_ms);
    serviceLinkTx(now_ms);
    printDiagnostics(millis());

    const std::uint32_t elapsed_us = micros() - loop_started_us;
    if (elapsed_us > loop_worst_us) {
        loop_worst_us = elapsed_us;
    }
    delay(1);
}
