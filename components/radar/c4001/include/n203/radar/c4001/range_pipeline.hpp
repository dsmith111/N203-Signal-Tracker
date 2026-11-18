#pragma once

#include <cstdint>

#include "n203/radar/c4001/report_stream.hpp"
#include "n203/radar/radar_frame.hpp"

namespace n203::radar::c4001 {

struct RangeMeasurement {
    std::uint8_t target_count = 0U;
    std::uint16_t range_mm = 0U;
    std::int16_t radial_velocity_mm_s = 0;
    std::uint16_t signal = 0U;
};

enum class RangeReportError : std::uint8_t {
    kNone,
    kUnsupportedReport,
    kMalformedReport,
    kOutOfRange,
};

// Parses C4001 distance/speed reports shaped as:
// $DFDMD,target_count,status,range_m,speed_m_s,energy, , *
// The sensor supplies range but no bearing, so the normalized contact remains
// range-only and is not assigned a fabricated azimuth or X/Y position. Raw
// energy values wider than the canonical 16-bit signal field are saturated.
[[nodiscard]] RangeReportError decodeRangeReport(
    const Report& report,
    RangeMeasurement& measurement);

// Requires two consecutive reports to establish or change target-count state.
// Once a target is confirmed, its range, velocity, and signal update on every
// valid report. This prevents one corrupted target-count byte from creating or
// clearing the display contact.
class RangeFilter final {
public:
    [[nodiscard]] bool observe(
        const RangeMeasurement& observed,
        RangeMeasurement& filtered);
    void reset();

private:
    RangeMeasurement confirmed_{};
    RangeMeasurement candidate_{};
    std::uint8_t candidate_count_ = 0U;
    bool has_confirmed_ = false;
};

[[nodiscard]] RadarFrame makeRangeFrame(
    const RangeMeasurement& measurement,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms);

}  // namespace n203::radar::c4001
