#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/radar/radar_frame.hpp"
#include "n203/radar/rd03d/raw_frame_stream.hpp"

namespace n203::radar::rd03d {

inline constexpr std::size_t kMaximumTargets = 3U;
inline constexpr std::uint16_t kMaximumRangeMm = 8000U;

struct TargetMeasurement {
    bool present = false;
    std::uint8_t slot = 0U;
    std::int16_t x_mm = 0;
    std::int16_t y_mm = 0;
    std::uint16_t range_mm = 0U;
    std::int16_t radial_velocity_mm_s = 0;
    std::uint16_t distance_resolution_mm = 0U;
};

struct RangeReport {
    std::uint8_t target_count = 0U;
    std::array<TargetMeasurement, kMaximumTargets> targets{};
};

enum class RangeReportError : std::uint8_t {
    kNone,
    kMalformedFrame,
    kOutOfRange,
};

[[nodiscard]] std::int16_t canonicalXFromNative(std::int16_t native_x_mm);

// Decodes the official three-slot report. Coordinates and speed use the
// module's unusual sign-magnitude representation: high bit set is positive,
// high bit clear is negative, and the remaining 15 bits are magnitude. Speed
// is converted from cm/s to canonical mm/s.
[[nodiscard]] RangeReportError decodeRangeReport(
    const RawFrame& frame,
    RangeReport& report);

// This staged frame intentionally exposes derived range rather than X/Y. Real
// hardware captures must confirm coordinate orientation before position and
// azimuth capabilities are enabled.
[[nodiscard]] RadarFrame makeRangeFrame(
    const RangeReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms);

// Maps the physically verified module coordinates into the canonical display
// convention. Native X is inverted so canonical positive X is right when the
// observer and radar face the same direction; positive Y remains forward.
// This verification frame exposes slot-local IDs only and intentionally does
// not advertise stable track identity.
[[nodiscard]] RadarFrame makePositionFrame(
    const RangeReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms);

}  // namespace n203::radar::rd03d
