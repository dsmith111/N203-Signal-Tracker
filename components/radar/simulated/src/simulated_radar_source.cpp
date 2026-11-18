#include "n203/radar/simulated_radar_source.hpp"

#include <array>

#include "n203/radar/capabilities.hpp"
#include "n203/radar/contact.hpp"

namespace n203::radar {
namespace {

constexpr std::array<const char*, SimulatedRadarSource::kPhaseCount> kPhaseNames{{
    "warmup",
    "tracking-empty",
    "approaching-contact",
    "lateral-crossing",
    "multiple-contacts",
    "range-only",
    "presence-only",
    "doppler-warning",
    "stabilizing",
    "stale-data",
    "scanner-offline",
}};

constexpr std::uint32_t kTrackingCapabilities =
    kPresence | kRange | kPosition | kAzimuth | kVelocity |
    kTrackIdentity | kMultipleContacts;

Contact trackedContact(
    std::uint8_t id,
    std::int16_t x_mm,
    std::int16_t y_mm,
    std::uint16_t range_mm,
    std::int16_t azimuth_cdeg,
    std::int16_t velocity_mm_s,
    std::uint16_t signal,
    std::uint8_t confidence) {
    Contact contact{};
    contact.id = id;
    contact.valid_fields =
        kContactPosition | kContactRange | kContactAzimuth |
        kContactRadialVelocity | kContactSignal | kContactConfidence;
    contact.x_mm = x_mm;
    contact.y_mm = y_mm;
    contact.range_mm = range_mm;
    contact.azimuth_cdeg = azimuth_cdeg;
    contact.radial_velocity_mm_s = velocity_mm_s;
    contact.signal = signal;
    contact.confidence = confidence;
    return contact;
}

void configureTrackingFrame(RadarFrame& frame, PresenceState presence) {
    frame.capabilities.flags = kTrackingCapabilities;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kScanning;
    frame.presence = presence;
    frame.motion = MotionState::kTracking;
    frame.motion_quality = 100U;
}

}  // namespace

void SimulatedRadarSource::begin(std::uint32_t now_ms) {
    phase_index_ = 0U;
    phase_started_ms_ = now_ms;
    next_frame_ms_ = now_ms;
    begun_ = true;
    frame_available_ = false;
}

void SimulatedRadarSource::update(std::uint32_t now_ms) {
    if (!begun_) {
        begin(now_ms);
    }

    while (now_ms - phase_started_ms_ >= kPhaseDurationMs) {
        phase_index_ = (phase_index_ + 1U) % kPhaseCount;
        phase_started_ms_ += kPhaseDurationMs;
        next_frame_ms_ = now_ms;
    }
    if (static_cast<std::int32_t>(now_ms - next_frame_ms_) < 0) {
        return;
    }

    next_frame_ms_ = now_ms + kFrameIntervalMs;
    buildFrame(now_ms);
    frame_available_ = true;
}

bool SimulatedRadarSource::poll(RadarFrame& frame) {
    if (!frame_available_) {
        return false;
    }
    frame = frame_;
    frame_available_ = false;
    return true;
}

const char* SimulatedRadarSource::phaseName() const {
    return kPhaseNames[phase_index_];
}

void SimulatedRadarSource::buildFrame(std::uint32_t now_ms) {
    const std::uint32_t next_sequence = frame_.sequence + 1U;
    frame_ = {};
    frame_.sequence = next_sequence;
    frame_.sensor_timestamp_ms = now_ms;
    const std::uint32_t phase_elapsed_ms = now_ms - phase_started_ms_;

    switch (phase_index_) {
        case 0:  // Warm-up.
            frame_.capabilities.flags = kTrackingCapabilities;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kInitializing;
            break;

        case 1:  // Healthy scanner, empty field.
            configureTrackingFrame(frame_, PresenceState::kClear);
            break;

        case 2: {  // One contact approaches over the phase.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            const std::int16_t y_mm = static_cast<std::int16_t>(
                7000 - static_cast<std::int32_t>(phase_elapsed_ms * 4000U / kPhaseDurationMs));
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(
                7, -700, y_mm, static_cast<std::uint16_t>(y_mm + 70),
                -700, -850, 780, 94);
            break;
        }

        case 3: {  // Lateral crossing at nearly constant range.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            const std::int16_t x_mm = static_cast<std::int16_t>(
                -3000 + static_cast<std::int32_t>(phase_elapsed_ms * 6000U / kPhaseDurationMs));
            const std::int16_t azimuth_cdeg = static_cast<std::int16_t>(
                -3550 + static_cast<std::int32_t>(phase_elapsed_ms * 7100U / kPhaseDurationMs));
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(8, x_mm, 4300, 5200, azimuth_cdeg, 0, 690, 88);
            break;
        }

        case 4:  // Multiple stable tracks.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            frame_.contact_count = 3;
            frame_.contacts[0] = trackedContact(1, -2600, 5200, 5814, -2657, -180, 820, 96);
            frame_.contacts[1] = trackedContact(2, 1800, 3400, 3847, 2790, 75, 610, 83);
            frame_.contacts[2] = trackedContact(3, 3200, 6100, 6889, 2768, 230, 430, 67);
            break;

        case 5:  // Presence and range without bearing.
            frame_.capabilities.flags = kPresence | kRange;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kRangeOnly;
            frame_.presence = PresenceState::kDetected;
            frame_.motion = MotionState::kTracking;
            frame_.motion_quality = 100U;
            frame_.contact_count = 1;
            frame_.contacts[0].id = 1;
            frame_.contacts[0].valid_fields =
                kContactRange | kContactSignal | kContactConfidence;
            frame_.contacts[0].range_mm = 6200;
            frame_.contacts[0].signal = 540;
            frame_.contacts[0].confidence = 72;
            break;

        case 6:  // Presence remains meaningful without a range contact.
            frame_.capabilities.flags = kPresence;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kPresenceOnly;
            frame_.presence = PresenceState::kDetected;
            frame_.motion = MotionState::kTracking;
            frame_.motion_quality = 100U;
            break;

        case 7:  // Carrier movement makes relative Doppler unreliable.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            frame_.motion = MotionState::kDopplerWarning;
            frame_.motion_quality = 32U;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(5, -1400, 4400, 4617, -1765, -260, 590, 74);
            break;

        case 8:  // Recovery while confidence settles.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            frame_.motion = MotionState::kStabilizing;
            frame_.motion_quality = 68U;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(5, -1200, 4550, 4705, -1477, -90, 630, 81);
            break;

        case 9:  // Explicit stale data still carries the last snapshot.
            configureTrackingFrame(frame_, PresenceState::kDetected);
            frame_.health = RadarHealth::kDegraded;
            frame_.measurement_state = MeasurementState::kStale;
            frame_.motion = MotionState::kUnknown;
            frame_.motion_quality = 0U;
            frame_.sensor_timestamp_ms = now_ms - 1600U;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(4, 1200, 4700, 4851, 1432, 0, 310, 48);
            break;

        default:  // Scanner offline.
            frame_.capabilities.flags = kTrackingCapabilities;
            frame_.health = RadarHealth::kOffline;
            frame_.measurement_state = MeasurementState::kOffline;
            break;
    }
}

}  // namespace n203::radar
