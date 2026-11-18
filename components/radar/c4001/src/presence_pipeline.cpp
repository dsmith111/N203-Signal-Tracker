#include "n203/radar/c4001/presence_pipeline.hpp"

#include <cstddef>

namespace n203::radar::c4001 {
namespace {

bool equals(const Report& report, const char* expected) {
    std::size_t index = 0U;
    while (expected[index] != '\0') {
        if (index >= report.size || report.text[index] != expected[index]) {
            return false;
        }
        ++index;
    }
    return index == report.size;
}

bool startsWith(const Report& report, const char* prefix) {
    std::size_t index = 0U;
    while (prefix[index] != '\0') {
        if (index >= report.size || report.text[index] != prefix[index]) {
            return false;
        }
        ++index;
    }
    return true;
}

}  // namespace

PresenceReportError decodePresenceReport(
    const Report& report,
    PresenceState& presence) {
    if (equals(report, "$DFHPD,0, , , *")) {
        presence = PresenceState::kClear;
        return PresenceReportError::kNone;
    }
    if (equals(report, "$DFHPD,1, , , *")) {
        presence = PresenceState::kDetected;
        return PresenceReportError::kNone;
    }
    if (startsWith(report, "$DFDMD,")) {
        return PresenceReportError::kUnsupportedReport;
    }
    return PresenceReportError::kMalformedReport;
}

bool PresenceFilter::observe(PresenceState observed, PresenceState& filtered) {
    if (observed != PresenceState::kClear && observed != PresenceState::kDetected) {
        return false;
    }

    if (!has_confirmed_) {
        if (candidate_count_ == 0U || candidate_ != observed) {
            candidate_ = observed;
            candidate_count_ = 1U;
            return false;
        }
        has_confirmed_ = true;
        confirmed_ = observed;
        candidate_count_ = 0U;
        filtered = confirmed_;
        return true;
    }

    if (observed == confirmed_) {
        candidate_count_ = 0U;
        filtered = confirmed_;
        return true;
    }

    if (candidate_count_ == 0U || candidate_ != observed) {
        candidate_ = observed;
        candidate_count_ = 1U;
        filtered = confirmed_;
        return true;
    }

    confirmed_ = observed;
    candidate_count_ = 0U;
    filtered = confirmed_;
    return true;
}

void PresenceFilter::reset() {
    confirmed_ = PresenceState::kUnknown;
    candidate_ = PresenceState::kUnknown;
    candidate_count_ = 0U;
    has_confirmed_ = false;
}

RadarFrame makePresenceFrame(
    PresenceState presence,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms) {
    RadarFrame frame{};
    frame.sequence = sequence;
    frame.sensor_timestamp_ms = sensor_timestamp_ms;
    frame.capabilities.flags = kPresence;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kPresenceOnly;
    frame.presence = presence;
    frame.motion = MotionState::kUnknown;
    frame.motion_quality = 0U;
    frame.contact_count = 0U;
    return frame;
}

}  // namespace n203::radar::c4001
