#include "n203/radar/rd03d/range_pipeline.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace n203::radar::rd03d {
namespace {

constexpr std::size_t kHeaderSize = 4U;
constexpr std::size_t kTargetSize = 8U;
constexpr double kRadiansToCentidegrees = 18000.0 / 3.14159265358979323846;

std::uint16_t decodeLittleEndianU16(std::uint8_t low, std::uint8_t high) {
    return static_cast<std::uint16_t>(low) |
           (static_cast<std::uint16_t>(high) << 8U);
}

std::int16_t decodeSignMagnitude(std::uint16_t encoded) {
    const std::int16_t magnitude =
        static_cast<std::int16_t>(encoded & 0x7FFFU);
    if (magnitude == 0) {
        return 0;
    }
    return (encoded & 0x8000U) != 0U ? magnitude
                                     : static_cast<std::int16_t>(-magnitude);
}

bool slotIsEmpty(const RawFrame& frame, std::size_t offset) {
    for (std::size_t index = 0U; index < kTargetSize; ++index) {
        if (frame[offset + index] != 0U) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::int16_t canonicalXFromNative(std::int16_t native_x_mm) {
    // Valid decoded targets are constrained to an 8 m radial range, so the
    // negation cannot encounter INT16_MIN.
    return static_cast<std::int16_t>(-native_x_mm);
}

RangeReportError decodeRangeReport(const RawFrame& frame, RangeReport& report) {
    report = {};
    if (frame[0] != 0xAAU || frame[1] != 0xFFU || frame[2] != 0x03U ||
        frame[3] != 0x00U || frame[28] != 0x55U || frame[29] != 0xCCU) {
        return RangeReportError::kMalformedFrame;
    }

    for (std::size_t slot = 0U; slot < kMaximumTargets; ++slot) {
        const std::size_t offset = kHeaderSize + slot * kTargetSize;
        if (slotIsEmpty(frame, offset)) {
            continue;
        }

        TargetMeasurement target{};
        target.present = true;
        target.slot = static_cast<std::uint8_t>(slot);
        target.x_mm = decodeSignMagnitude(
            decodeLittleEndianU16(frame[offset], frame[offset + 1U]));
        target.y_mm = decodeSignMagnitude(
            decodeLittleEndianU16(frame[offset + 2U], frame[offset + 3U]));
        const std::int16_t speed_cm_s = decodeSignMagnitude(
            decodeLittleEndianU16(frame[offset + 4U], frame[offset + 5U]));
        target.distance_resolution_mm = decodeLittleEndianU16(
            frame[offset + 6U], frame[offset + 7U]);

        const std::int32_t speed_mm_s =
            static_cast<std::int32_t>(speed_cm_s) * 10;
        if (speed_mm_s < std::numeric_limits<std::int16_t>::min() ||
            speed_mm_s > std::numeric_limits<std::int16_t>::max()) {
            return RangeReportError::kOutOfRange;
        }
        target.radial_velocity_mm_s = static_cast<std::int16_t>(speed_mm_s);

        const double x = static_cast<double>(target.x_mm);
        const double y = static_cast<double>(target.y_mm);
        const long range_mm = std::lround(std::sqrt(x * x + y * y));
        if (range_mm <= 0L ||
            range_mm > static_cast<long>(kMaximumRangeMm)) {
            return RangeReportError::kOutOfRange;
        }
        target.range_mm = static_cast<std::uint16_t>(range_mm);
        report.targets[slot] = target;
        ++report.target_count;
    }
    return RangeReportError::kNone;
}

RadarFrame makeRangeFrame(
    const RangeReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms) {
    RadarFrame frame{};
    frame.sequence = sequence;
    frame.sensor_timestamp_ms = sensor_timestamp_ms;
    frame.capabilities.flags =
        kPresence | kRange | kVelocity | kMultipleContacts;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kRangeOnly;
    frame.presence = report.target_count == 0U
                         ? PresenceState::kClear
                         : PresenceState::kDetected;
    frame.motion = report.target_count == 0U
                       ? MotionState::kUnknown
                       : MotionState::kTracking;
    frame.motion_quality = 0U;

    for (const TargetMeasurement& target : report.targets) {
        if (!target.present || frame.contact_count >= frame.contacts.size()) {
            continue;
        }
        Contact& contact = frame.contacts[frame.contact_count++];
        contact.id = static_cast<std::uint8_t>(target.slot + 1U);
        contact.valid_fields = kContactRange | kContactRadialVelocity;
        contact.range_mm = target.range_mm;
        contact.radial_velocity_mm_s = target.radial_velocity_mm_s;
    }
    return frame;
}

RadarFrame makePositionFrame(
    const RangeReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms) {
    RadarFrame frame{};
    frame.sequence = sequence;
    frame.sensor_timestamp_ms = sensor_timestamp_ms;
    frame.capabilities.flags =
        kPresence | kRange | kPosition | kAzimuth | kVelocity |
        kMultipleContacts;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kScanning;
    frame.presence = report.target_count == 0U
                         ? PresenceState::kClear
                         : PresenceState::kDetected;
    frame.motion = report.target_count == 0U
                       ? MotionState::kUnknown
                       : MotionState::kTracking;
    frame.motion_quality = 0U;

    for (const TargetMeasurement& target : report.targets) {
        if (!target.present || frame.contact_count >= frame.contacts.size()) {
            continue;
        }
        Contact& contact = frame.contacts[frame.contact_count++];
        contact.id = static_cast<std::uint8_t>(target.slot + 1U);
        contact.valid_fields =
            kContactPosition | kContactRange | kContactAzimuth |
            kContactRadialVelocity;
        contact.x_mm = canonicalXFromNative(target.x_mm);
        contact.y_mm = target.y_mm;
        contact.range_mm = target.range_mm;
        contact.azimuth_cdeg = static_cast<std::int16_t>(std::lround(
            std::atan2(static_cast<double>(contact.x_mm),
                       static_cast<double>(target.y_mm)) *
            kRadiansToCentidegrees));
        contact.radial_velocity_mm_s = target.radial_velocity_mm_s;
    }
    return frame;
}

}  // namespace n203::radar::rd03d
