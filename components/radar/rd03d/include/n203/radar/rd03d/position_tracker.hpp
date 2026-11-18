#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/radar/radar_frame.hpp"
#include "n203/radar/rd03d/range_pipeline.hpp"

namespace n203::radar::rd03d {

inline constexpr std::uint32_t kTrackVisibleHoldMs = 300U;
inline constexpr std::uint32_t kTrackRetentionMs = 1200U;

struct TrackedTarget {
    bool observed = false;
    std::uint8_t id = 0U;
    std::uint8_t source_slot = 0U;
    std::int16_t x_mm = 0;
    std::int16_t y_mm = 0;
    std::uint16_t range_mm = 0U;
    std::int16_t azimuth_cdeg = 0;
    std::int16_t radial_velocity_mm_s = 0;
    std::uint8_t confidence = 0U;
};

struct TrackedPositionReport {
    std::uint8_t target_count = 0U;
    std::uint8_t observed_count = 0U;
    std::array<TrackedTarget, kMaximumTargets> targets{};
};

// Associates snapshot-local RD-03D slots with software tracks, filters small
// position noise, and retains contacts briefly across isolated empty reports.
// The implementation is bounded and allocation-free for the C3 hot path.
class PositionTracker {
public:
    void reset();

    [[nodiscard]] TrackedPositionReport update(
        const RangeReport& report,
        std::uint32_t sensor_timestamp_ms);

private:
    struct TrackState {
        bool active = false;
        bool observed = false;
        std::uint8_t id = 0U;
        std::uint8_t source_slot = 0U;
        std::int16_t x_mm = 0;
        std::int16_t y_mm = 0;
        std::int16_t last_measurement_x_mm = 0;
        std::int16_t last_measurement_y_mm = 0;
        std::int16_t velocity_x_mm_s = 0;
        std::int16_t velocity_y_mm_s = 0;
        std::int16_t radial_velocity_mm_s = 0;
        std::uint16_t hits = 0U;
        std::uint32_t last_seen_ms = 0U;
    };

    [[nodiscard]] std::uint8_t allocateTrackId();

    std::array<TrackState, kMaximumTargets> tracks_{};
    std::uint8_t next_track_id_ = 1U;
};

[[nodiscard]] RadarFrame makeTrackedPositionFrame(
    const TrackedPositionReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms);

}  // namespace n203::radar::rd03d
