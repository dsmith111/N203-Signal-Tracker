#include "n203/radar/rd03d/position_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace n203::radar::rd03d {
namespace {

constexpr std::uint32_t kBaseAssociationGateMm = 700U;
constexpr std::uint32_t kMaximumAssociationGateMm = 1800U;
constexpr std::int32_t kFastFilterThresholdMm = 600;
constexpr std::int32_t kPositionDeadbandMm = 15;
constexpr double kRadiansToCentidegrees = 18000.0 / 3.14159265358979323846;

struct CandidatePair {
    std::size_t track_index = 0U;
    std::size_t measurement_index = 0U;
    std::uint32_t distance_squared = 0U;
};

std::uint32_t squaredDistance(
    std::int32_t left_x,
    std::int32_t left_y,
    std::int32_t right_x,
    std::int32_t right_y) {
    const std::int32_t delta_x = left_x - right_x;
    const std::int32_t delta_y = left_y - right_y;
    return static_cast<std::uint32_t>(delta_x * delta_x + delta_y * delta_y);
}

std::int16_t filteredPosition(
    std::int16_t previous,
    std::int16_t measurement) {
    const std::int32_t delta =
        static_cast<std::int32_t>(measurement) - previous;
    if (std::abs(delta) <= kPositionDeadbandMm) {
        return previous;
    }
    if (std::abs(delta) >= kFastFilterThresholdMm) {
        return static_cast<std::int16_t>(
            (static_cast<std::int32_t>(previous) * 1 +
             static_cast<std::int32_t>(measurement) * 2) /
            3);
    }
    return static_cast<std::int16_t>(
        (static_cast<std::int32_t>(previous) * 3 +
         static_cast<std::int32_t>(measurement) * 2) /
        5);
}

std::int16_t filteredVelocity(
    std::int16_t previous,
    std::int32_t measurement) {
    const std::int32_t limited = std::max<std::int32_t>(
        std::numeric_limits<std::int16_t>::min(),
        std::min<std::int32_t>(
            std::numeric_limits<std::int16_t>::max(), measurement));
    return static_cast<std::int16_t>(
        (static_cast<std::int32_t>(previous) * 3 + limited * 2) / 5);
}

std::uint16_t rangeFor(std::int16_t x_mm, std::int16_t y_mm) {
    const double x = static_cast<double>(x_mm);
    const double y = static_cast<double>(y_mm);
    return static_cast<std::uint16_t>(
        std::lround(std::sqrt(x * x + y * y)));
}

std::int16_t azimuthFor(std::int16_t x_mm, std::int16_t y_mm) {
    return static_cast<std::int16_t>(std::lround(
        std::atan2(static_cast<double>(x_mm), static_cast<double>(y_mm)) *
        kRadiansToCentidegrees));
}

std::uint8_t confidenceFor(std::uint16_t hits, std::uint32_t age_ms) {
    const std::uint16_t established = static_cast<std::uint16_t>(
        50U + std::min<std::uint16_t>(hits, 5U) * 10U);
    const std::uint16_t decay = static_cast<std::uint16_t>(
        std::min<std::uint32_t>(age_ms / 5U, 60U));
    return static_cast<std::uint8_t>(established > decay
                                         ? established - decay
                                         : 1U);
}

}  // namespace

void PositionTracker::reset() {
    tracks_ = {};
    next_track_id_ = 1U;
}

std::uint8_t PositionTracker::allocateTrackId() {
    for (std::uint16_t attempt = 0U; attempt < 255U; ++attempt) {
        const std::uint8_t candidate = next_track_id_;
        ++next_track_id_;
        if (next_track_id_ == 0U) {
            next_track_id_ = 1U;
        }

        bool in_use = false;
        for (const TrackState& track : tracks_) {
            if (track.active && track.id == candidate) {
                in_use = true;
                break;
            }
        }
        if (!in_use && candidate != 0U) {
            return candidate;
        }
    }
    return 1U;
}

TrackedPositionReport PositionTracker::update(
    const RangeReport& report,
    std::uint32_t sensor_timestamp_ms) {
    for (TrackState& track : tracks_) {
        track.observed = false;
        if (track.active &&
            sensor_timestamp_ms - track.last_seen_ms > kTrackRetentionMs) {
            track = {};
        }
    }

    std::array<std::int16_t, kMaximumTargets> measurement_x{};
    for (std::size_t index = 0U; index < report.targets.size(); ++index) {
        if (report.targets[index].present) {
            measurement_x[index] =
                canonicalXFromNative(report.targets[index].x_mm);
        }
    }

    std::array<CandidatePair, kMaximumTargets * kMaximumTargets> candidates{};
    std::size_t candidate_count = 0U;
    for (std::size_t track_index = 0U; track_index < tracks_.size();
         ++track_index) {
        const TrackState& track = tracks_[track_index];
        if (!track.active) {
            continue;
        }
        const std::uint32_t age_ms = sensor_timestamp_ms - track.last_seen_ms;
        const std::uint32_t prediction_ms = std::min<std::uint32_t>(age_ms, 500U);
        const std::int32_t predicted_x = track.x_mm +
            static_cast<std::int32_t>(track.velocity_x_mm_s) *
                static_cast<std::int32_t>(prediction_ms) / 1000;
        const std::int32_t predicted_y = track.y_mm +
            static_cast<std::int32_t>(track.velocity_y_mm_s) *
                static_cast<std::int32_t>(prediction_ms) / 1000;
        const std::uint32_t gate_mm = std::min(
            kMaximumAssociationGateMm,
            kBaseAssociationGateMm +
                std::min<std::uint32_t>(age_ms * 2U, 1100U));
        const std::uint32_t gate_squared = gate_mm * gate_mm;

        for (std::size_t measurement_index = 0U;
             measurement_index < report.targets.size(); ++measurement_index) {
            const TargetMeasurement& measurement =
                report.targets[measurement_index];
            if (!measurement.present) {
                continue;
            }
            const std::uint32_t distance_squared = squaredDistance(
                predicted_x, predicted_y, measurement_x[measurement_index],
                measurement.y_mm);
            if (distance_squared <= gate_squared) {
                candidates[candidate_count++] = {
                    track_index, measurement_index, distance_squared};
            }
        }
    }
    std::sort(candidates.begin(), candidates.begin() + candidate_count,
              [](const CandidatePair& left, const CandidatePair& right) {
                  return left.distance_squared < right.distance_squared;
              });

    std::array<int, kMaximumTargets> measurement_tracks{};
    measurement_tracks.fill(-1);
    std::array<bool, kMaximumTargets> track_used{};
    for (std::size_t index = 0U; index < candidate_count; ++index) {
        const CandidatePair& candidate = candidates[index];
        if (track_used[candidate.track_index] ||
            measurement_tracks[candidate.measurement_index] >= 0) {
            continue;
        }
        track_used[candidate.track_index] = true;
        measurement_tracks[candidate.measurement_index] =
            static_cast<int>(candidate.track_index);
    }

    for (std::size_t measurement_index = 0U;
         measurement_index < report.targets.size(); ++measurement_index) {
        const TargetMeasurement& measurement = report.targets[measurement_index];
        if (!measurement.present) {
            continue;
        }

        int assigned_index = measurement_tracks[measurement_index];
        bool new_track = assigned_index < 0;
        if (new_track) {
            for (std::size_t index = 0U; index < tracks_.size(); ++index) {
                if (!tracks_[index].active) {
                    assigned_index = static_cast<int>(index);
                    break;
                }
            }
        }
        if (new_track && assigned_index < 0) {
            std::uint32_t oldest_age = 0U;
            for (std::size_t index = 0U; index < tracks_.size(); ++index) {
                if (track_used[index]) {
                    continue;
                }
                const std::uint32_t age =
                    sensor_timestamp_ms - tracks_[index].last_seen_ms;
                if (assigned_index < 0 || age > oldest_age) {
                    assigned_index = static_cast<int>(index);
                    oldest_age = age;
                }
            }
        }
        if (assigned_index < 0) {
            continue;
        }

        TrackState& track = tracks_[static_cast<std::size_t>(assigned_index)];
        const std::int16_t canonical_x = measurement_x[measurement_index];
        if (new_track) {
            track = {};
            track.active = true;
            track.id = allocateTrackId();
            track.x_mm = canonical_x;
            track.y_mm = measurement.y_mm;
            track.last_measurement_x_mm = canonical_x;
            track.last_measurement_y_mm = measurement.y_mm;
            track.radial_velocity_mm_s = measurement.radial_velocity_mm_s;
            track.hits = 1U;
        } else {
            const std::uint32_t elapsed_ms = std::max<std::uint32_t>(
                1U, sensor_timestamp_ms - track.last_seen_ms);
            const std::int32_t measured_velocity_x =
                (static_cast<std::int32_t>(canonical_x) -
                 track.last_measurement_x_mm) *
                1000 / static_cast<std::int32_t>(elapsed_ms);
            const std::int32_t measured_velocity_y =
                (static_cast<std::int32_t>(measurement.y_mm) -
                 track.last_measurement_y_mm) *
                1000 / static_cast<std::int32_t>(elapsed_ms);
            track.velocity_x_mm_s = filteredVelocity(
                track.velocity_x_mm_s, measured_velocity_x);
            track.velocity_y_mm_s = filteredVelocity(
                track.velocity_y_mm_s, measured_velocity_y);
            track.x_mm = filteredPosition(track.x_mm, canonical_x);
            track.y_mm = filteredPosition(track.y_mm, measurement.y_mm);
            track.radial_velocity_mm_s = filteredVelocity(
                track.radial_velocity_mm_s,
                measurement.radial_velocity_mm_s);
            track.last_measurement_x_mm = canonical_x;
            track.last_measurement_y_mm = measurement.y_mm;
            if (track.hits < std::numeric_limits<std::uint16_t>::max()) {
                ++track.hits;
            }
        }
        track.source_slot = measurement.slot;
        track.last_seen_ms = sensor_timestamp_ms;
        track.observed = true;
        track_used[static_cast<std::size_t>(assigned_index)] = true;
    }

    TrackedPositionReport tracked{};
    for (const TrackState& track : tracks_) {
        if (!track.active) {
            continue;
        }
        const std::uint32_t age_ms = sensor_timestamp_ms - track.last_seen_ms;
        if (!track.observed && age_ms > kTrackVisibleHoldMs) {
            continue;
        }
        TrackedTarget& target = tracked.targets[tracked.target_count++];
        target.observed = track.observed;
        target.id = track.id;
        target.source_slot = track.source_slot;
        target.x_mm = track.x_mm;
        target.y_mm = track.y_mm;
        target.range_mm = rangeFor(track.x_mm, track.y_mm);
        target.azimuth_cdeg = azimuthFor(track.x_mm, track.y_mm);
        target.radial_velocity_mm_s = track.radial_velocity_mm_s;
        target.confidence = confidenceFor(track.hits, age_ms);
        if (track.observed) {
            ++tracked.observed_count;
        }
    }
    return tracked;
}

RadarFrame makeTrackedPositionFrame(
    const TrackedPositionReport& report,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms) {
    RadarFrame frame{};
    frame.sequence = sequence;
    frame.sensor_timestamp_ms = sensor_timestamp_ms;
    frame.capabilities.flags =
        kPresence | kRange | kPosition | kAzimuth | kVelocity |
        kTrackIdentity | kMultipleContacts;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kScanning;
    frame.presence = report.target_count == 0U
                         ? PresenceState::kClear
                         : PresenceState::kDetected;
    frame.motion = report.target_count == 0U
                       ? MotionState::kUnknown
                       : MotionState::kTracking;

    for (const TrackedTarget& target : report.targets) {
        if (target.id == 0U || frame.contact_count >= frame.contacts.size()) {
            continue;
        }
        Contact& contact = frame.contacts[frame.contact_count++];
        contact.id = target.id;
        contact.valid_fields =
            kContactPosition | kContactRange | kContactAzimuth |
            kContactRadialVelocity | kContactConfidence;
        contact.x_mm = target.x_mm;
        contact.y_mm = target.y_mm;
        contact.range_mm = target.range_mm;
        contact.azimuth_cdeg = target.azimuth_cdeg;
        contact.radial_velocity_mm_s = target.radial_velocity_mm_s;
        contact.confidence = target.confidence;
    }
    return frame;
}

}  // namespace n203::radar::rd03d
