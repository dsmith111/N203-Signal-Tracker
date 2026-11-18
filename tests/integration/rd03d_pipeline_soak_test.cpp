#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>

#include "n203/display/display_app.hpp"
#include "n203/display/radar_geometry.hpp"
#include "n203/radar/frame_validation.hpp"
#include "n203/radar/protocol/radar_link.hpp"
#include "n203/radar/rd03d/position_tracker.hpp"
#include "n203/radar/rd03d/raw_frame_stream.hpp"

namespace {

namespace display = n203::display;
namespace protocol = n203::radar::protocol;
namespace rd03d = n203::radar::rd03d;

constexpr std::uint32_t kFrameIntervalMs = 100U;
constexpr std::uint32_t kSimulatedMinutes = 30U;
constexpr std::uint32_t kFrameCount =
    kSimulatedMinutes * 60U * 1000U / kFrameIntervalMs;

struct PhysicalTarget {
    std::int16_t canonical_x_mm = 0;
    std::int16_t y_mm = 0;
};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::uint16_t encodeSignMagnitude(std::int16_t value) {
    if (value == 0) {
        return 0U;
    }
    const std::uint16_t magnitude = static_cast<std::uint16_t>(
        value > 0 ? value : -static_cast<std::int32_t>(value));
    return value > 0 ? static_cast<std::uint16_t>(0x8000U | magnitude)
                     : magnitude;
}

void writeU16(rd03d::RawFrame& frame, std::size_t offset,
              std::uint16_t value) {
    frame[offset] = static_cast<std::uint8_t>(value);
    frame[offset + 1U] = static_cast<std::uint8_t>(value >> 8U);
}

rd03d::RawFrame makeRawFrame(
    const std::array<PhysicalTarget, rd03d::kMaximumTargets>& targets,
    std::uint32_t frame_index) {
    rd03d::RawFrame frame{};
    frame[0] = 0xAAU;
    frame[1] = 0xFFU;
    frame[2] = 0x03U;
    frame[3] = 0x00U;
    frame[28] = 0x55U;
    frame[29] = 0xCCU;

    const bool full_dropout =
        frame_index % 997U == 500U || frame_index % 997U == 501U;
    const std::size_t single_dropout_target =
        static_cast<std::size_t>((frame_index / 211U) % targets.size());
    const bool single_dropout =
        frame_index > 0U &&
        (frame_index % 211U == 100U || frame_index % 211U == 101U);
    const std::size_t slot_offset =
        static_cast<std::size_t>((frame_index / 25U) % targets.size());

    for (std::size_t physical_index = 0U; physical_index < targets.size();
         ++physical_index) {
        if (full_dropout ||
            (single_dropout && physical_index == single_dropout_target)) {
            continue;
        }
        const std::size_t slot =
            (physical_index + slot_offset) % targets.size();
        const std::size_t offset = 4U + slot * 8U;
        const std::int16_t native_x_mm = static_cast<std::int16_t>(
            -targets[physical_index].canonical_x_mm);
        writeU16(frame, offset, encodeSignMagnitude(native_x_mm));
        writeU16(frame, offset + 2U,
                 encodeSignMagnitude(targets[physical_index].y_mm));
        writeU16(frame, offset + 4U, 0U);
        writeU16(frame, offset + 6U, 360U);
    }
    return frame;
}

std::array<PhysicalTarget, rd03d::kMaximumTargets> targetsFor(
    std::uint32_t frame_index) {
    const std::int32_t cycle = static_cast<std::int32_t>(frame_index % 400U);
    const std::int32_t triangular = cycle <= 200 ? cycle : 400 - cycle;
    const std::int16_t crossing_x = static_cast<std::int16_t>(
        -1200 + triangular * 12);
    const std::int32_t third_cycle =
        static_cast<std::int32_t>(frame_index % 320U);
    const std::int32_t third_triangular =
        third_cycle <= 160 ? third_cycle : 320 - third_cycle;
    const std::int16_t third_x = static_cast<std::int16_t>(
        -640 + third_triangular * 8);
    return {{{crossing_x, 2600},
             {static_cast<std::int16_t>(-crossing_x), 2760},
             {third_x, 4300}}};
}

const rd03d::TrackedTarget* findTrack(
    const rd03d::TrackedPositionReport& report,
    std::uint8_t id) {
    for (std::size_t index = 0U; index < report.target_count; ++index) {
        if (report.targets[index].id == id) {
            return &report.targets[index];
        }
    }
    return nullptr;
}

protocol::StreamEvent feedPacket(
    protocol::RadarLinkStreamDecoder& decoder,
    const std::uint8_t* bytes,
    std::size_t size,
    protocol::DecodedPacket& packet) {
    protocol::StreamEvent result = protocol::StreamEvent::kNone;
    for (std::size_t index = 0U; index < size; ++index) {
        const auto event = decoder.push(bytes[index], packet);
        if (event != protocol::StreamEvent::kNone) {
            result = event;
        }
    }
    return result;
}

int runSoak() {
    rd03d::RawFrameStream raw_stream;
    rd03d::PositionTracker tracker;
    protocol::RadarLinkStreamDecoder link_decoder;
    protocol::SequenceTracker sequence_tracker;
    display::DisplayApp app;
    std::array<std::uint8_t, protocol::kMaximumPacketSize> wire{};

    for (std::uint32_t frame_index = 0U; frame_index < kFrameCount;
         ++frame_index) {
        const std::uint32_t now_ms = frame_index * kFrameIntervalMs;
        const auto physical_targets = targetsFor(frame_index);
        const auto raw_source = makeRawFrame(physical_targets, frame_index);

        rd03d::RawFrame captured{};
        rd03d::StreamEvent raw_event = rd03d::StreamEvent::kNone;
        for (const std::uint8_t byte : raw_source) {
            const auto event = raw_stream.push(byte, captured);
            if (event != rd03d::StreamEvent::kNone) {
                raw_event = event;
            }
        }
        require(raw_event == rd03d::StreamEvent::kFrameReady,
                "RD-03D framing failed during soak");

        rd03d::RangeReport range_report{};
        require(rd03d::decodeRangeReport(captured, range_report) ==
                    rd03d::RangeReportError::kNone,
                "RD-03D decode failed during soak");
        const auto tracked = tracker.update(range_report, now_ms);
        require(tracked.target_count == rd03d::kMaximumTargets,
                "brief dropout removed a tracked contact");
        if (findTrack(tracked, 1U) == nullptr ||
            findTrack(tracked, 2U) == nullptr ||
            findTrack(tracked, 3U) == nullptr) {
            std::cerr << "identity failure at frame=" << frame_index
                      << " contacts="
                      << static_cast<unsigned int>(tracked.target_count)
                      << " measurements=";
            for (const auto& measurement : range_report.targets) {
                if (measurement.present) {
                    std::cerr << static_cast<unsigned int>(measurement.slot)
                              << '@' << rd03d::canonicalXFromNative(
                                     measurement.x_mm)
                              << ',' << measurement.y_mm << ' ';
                }
            }
            std::cerr << "ids=";
            for (std::size_t index = 0U; index < tracked.target_count; ++index) {
                std::cerr << static_cast<unsigned int>(tracked.targets[index].id)
                          << '@' << tracked.targets[index].x_mm << ','
                          << tracked.targets[index].y_mm << ' ';
            }
            std::cerr << '\n';
            require(false,
                    "software track identity changed during slot permutation");
        }

        const auto frame = rd03d::makeTrackedPositionFrame(
            tracked, frame_index, now_ms);
        require(n203::radar::validateFrame(frame) ==
                    n203::radar::FrameValidationError::kNone,
                "tracker emitted an invalid canonical frame");

        std::size_t wire_size = 0U;
        require(protocol::encodeRadarFramePacket(
                    frame, frame_index, now_ms, wire.data(), wire.size(),
                    wire_size) == protocol::CodecError::kNone,
                "link encoding failed during soak");
        protocol::DecodedPacket packet{};
        require(feedPacket(link_decoder, wire.data(), wire_size, packet) ==
                    protocol::StreamEvent::kPacketReady,
                "link decoder failed during soak");
        require(packet.metadata.type == protocol::PacketType::kRadarFrame,
                "unexpected packet type during soak");
        const auto sequence = sequence_tracker.observe(
            packet.metadata.packet_sequence);
        require(frame_index == 0U
                    ? sequence.event == protocol::SequenceEvent::kFirst
                    : sequence.event == protocol::SequenceEvent::kInOrder,
                "packet sequence was not continuous during soak");

        app.acceptRadarFrame(packet.radar_frame, now_ms);
        app.update(now_ms);
        require(app.state().scanner ==
                    n203::display::ScannerPresentation::kScanning,
                "display left scanning state during soak");

        if (frame_index == 0U) {
            display::Point contact_point{};
            require(display::radarPointForContact(
                        packet.radar_frame.contacts[0],
                        app.state().display_range_mm, contact_point),
                    "initial contact was not displayable");
            app.acceptInput(display::DisplayInput::touchReleased(contact_point));
            app.acceptInput(display::DisplayInput::touchReleased(contact_point));
            require(app.state().has_selected_contact &&
                        app.state().selected_contact_id == 1U &&
                        app.state().target_locked,
                    "display failed to lock the initial software track");
        } else {
            require(app.state().has_selected_contact &&
                        app.state().selected_contact_id == 1U &&
                        app.state().target_locked,
                    "display lock did not survive tracking updates");
        }
    }

    require(raw_stream.stats().frames == kFrameCount,
            "raw frame count did not match soak input");
    require(raw_stream.stats().rejected_frames == 0U,
            "valid soak input was rejected");
    std::cout << "RD-03D pipeline soak passed: frames=" << kFrameCount
              << " simulated_minutes=" << kSimulatedMinutes << '\n';
    return 0;
}

}  // namespace

int main() {
    try {
        return runSoak();
    } catch (const std::exception& error) {
        std::cerr << "RD-03D pipeline soak failed: " << error.what() << '\n';
        return 1;
    }
}
