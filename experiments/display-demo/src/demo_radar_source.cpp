#include "demo_radar_source.hpp"

#include <array>

#include "n203/radar/capabilities.hpp"
#include "n203/radar/contact.hpp"

namespace n203::experiments::display_demo {
namespace {

using radar::Contact;
using radar::MeasurementState;
using radar::MotionState;
using radar::PresenceState;
using radar::RadarCapabilities;
using radar::RadarFrame;
using radar::RadarHealth;

constexpr std::uint32_t kPhaseDurationMs = 4500U;
constexpr std::uint32_t kFrameIntervalMs = 100U;
constexpr std::size_t kPhaseCount = 10U;

constexpr std::array<const char*, kPhaseCount> kPhaseNames{{
    "warmup",
    "tracking-empty",
    "tracking-one",
    "tracking-three",
    "range-only",
    "presence-only",
    "doppler-warning",
    "stabilizing",
    "stale-data",
    "scanner-offline",
}};

constexpr std::uint32_t kTrackingCapabilities =
    radar::kPresence | radar::kRange | radar::kPosition | radar::kAzimuth |
    radar::kVelocity | radar::kTrackIdentity | radar::kMultipleContacts;

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
        radar::kContactPosition | radar::kContactRange | radar::kContactAzimuth |
        radar::kContactRadialVelocity | radar::kContactSignal | radar::kContactConfidence;
    contact.x_mm = x_mm;
    contact.y_mm = y_mm;
    contact.range_mm = range_mm;
    contact.azimuth_cdeg = azimuth_cdeg;
    contact.radial_velocity_mm_s = velocity_mm_s;
    contact.signal = signal;
    contact.confidence = confidence;
    return contact;
}

void configureTrackingFrame(RadarFrame& frame) {
    frame.capabilities.flags = kTrackingCapabilities;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kScanning;
    frame.presence = PresenceState::kClear;
    frame.motion = MotionState::kTracking;
    frame.motion_quality = 100U;
}

}  // namespace

void DemoRadarSource::begin(std::uint32_t now_ms) {
    phase_index_ = 0;
    phase_started_ms_ = now_ms;
    next_frame_ms_ = now_ms;
    begun_ = true;
    frame_available_ = false;
}

void DemoRadarSource::update(std::uint32_t now_ms) {
    if (!begun_) {
        begin(now_ms);
    }

    if (now_ms - phase_started_ms_ >= kPhaseDurationMs) {
        phase_index_ = (phase_index_ + 1U) % kPhaseCount;
        phase_started_ms_ = now_ms;
        next_frame_ms_ = now_ms;
    }

    if (static_cast<std::int32_t>(now_ms - next_frame_ms_) < 0) {
        return;
    }

    next_frame_ms_ = now_ms + kFrameIntervalMs;
    buildFrame(now_ms);
    frame_available_ = true;
}

bool DemoRadarSource::poll(RadarFrame& frame) {
    if (!frame_available_) {
        return false;
    }
    frame = frame_;
    frame_available_ = false;
    return true;
}

const char* DemoRadarSource::phaseName() const {
    return kPhaseNames[phase_index_];
}

void DemoRadarSource::buildFrame(std::uint32_t now_ms) {
    const std::uint32_t next_sequence = frame_.sequence + 1U;
    frame_ = {};
    frame_.sequence = next_sequence;
    frame_.sensor_timestamp_ms = now_ms;

    switch (phase_index_) {
        case 0:  // Warmup.
            frame_.capabilities.flags = kTrackingCapabilities;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kInitializing;
            break;

        case 1:  // Empty tracking field.
            configureTrackingFrame(frame_);
            break;

        case 2:  // One selected/locked contact.
            configureTrackingFrame(frame_);
            frame_.presence = PresenceState::kDetected;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(7, -900, 5100, 5179, -1001, -120, 780, 94);
            break;

        case 3:  // Three contacts with distinct motion histories.
            configureTrackingFrame(frame_);
            frame_.presence = PresenceState::kDetected;
            frame_.contact_count = 3;
            frame_.contacts[0] = trackedContact(1, -2600, 5200, 5814, -2657, -180, 820, 96);
            frame_.contacts[1] = trackedContact(2, 1800, 3400, 3847, 2790, 75, 610, 83);
            frame_.contacts[2] = trackedContact(3, 3200, 6100, 6889, 2768, 230, 430, 67);
            break;

        case 4:  // A sensor that reports range without bearing.
            frame_.capabilities.flags = radar::kPresence | radar::kRange;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kRangeOnly;
            frame_.presence = PresenceState::kDetected;
            frame_.motion = MotionState::kTracking;
            frame_.motion_quality = 100U;
            frame_.contact_count = 1;
            frame_.contacts[0].id = 1;
            frame_.contacts[0].valid_fields =
                radar::kContactRange | radar::kContactSignal | radar::kContactConfidence;
            frame_.contacts[0].range_mm = 6200;
            frame_.contacts[0].signal = 540;
            frame_.contacts[0].confidence = 72;
            break;

        case 5:  // Presence-only capability.
            frame_.capabilities.flags = radar::kPresence;
            frame_.health = RadarHealth::kHealthy;
            frame_.measurement_state = MeasurementState::kPresenceOnly;
            frame_.motion = MotionState::kTracking;
            frame_.motion_quality = 100U;
            switch ((now_ms - phase_started_ms_) / (kPhaseDurationMs / 3U)) {
                case 0: frame_.presence = PresenceState::kUnknown; break;
                case 1: frame_.presence = PresenceState::kClear; break;
                default: frame_.presence = PresenceState::kDetected; break;
            }
            break;

        case 6:  // Carrier movement can make relative Doppler unreliable.
            configureTrackingFrame(frame_);
            frame_.presence = PresenceState::kDetected;
            frame_.motion = MotionState::kDopplerWarning;
            frame_.motion_quality = 32U;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(5, -1400, 4400, 4617, -1765, -260, 590, 74);
            break;

        case 7:  // Recovery keeps returns visible while confidence settles.
            configureTrackingFrame(frame_);
            frame_.presence = PresenceState::kDetected;
            frame_.motion = MotionState::kStabilizing;
            frame_.motion_quality = 68U;
            frame_.contact_count = 1;
            frame_.contacts[0] = trackedContact(5, -1200, 4550, 4705, -1477, -90, 630, 81);
            break;

        case 8:  // Explicit stale frame with the last known target.
            configureTrackingFrame(frame_);
            frame_.health = RadarHealth::kDegraded;
            frame_.measurement_state = MeasurementState::kStale;
            frame_.presence = PresenceState::kDetected;
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
            frame_.presence = PresenceState::kUnknown;
            frame_.motion = MotionState::kUnknown;
            frame_.motion_quality = 0U;
            break;
    }
}

}  // namespace n203::experiments::display_demo
