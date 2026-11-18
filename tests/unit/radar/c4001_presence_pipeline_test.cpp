#include <cassert>
#include <cstring>

#include "n203/radar/c4001/presence_pipeline.hpp"
#include "n203/radar/frame_validation.hpp"

namespace {

namespace c4001 = n203::radar::c4001;

c4001::Report reportFrom(const char* text) {
    c4001::Report report{};
    report.size = std::strlen(text);
    assert(report.size <= c4001::kMaximumReportSize);
    std::memcpy(report.text.data(), text, report.size);
    return report;
}

}  // namespace

void runC4001PresencePipelineTests() {
    using n203::radar::PresenceState;

    PresenceState presence = PresenceState::kUnknown;
    assert(c4001::decodePresenceReport(reportFrom("$DFHPD,0, , , *"), presence) ==
           c4001::PresenceReportError::kNone);
    assert(presence == PresenceState::kClear);
    assert(c4001::decodePresenceReport(reportFrom("$DFHPD,1, , , *"), presence) ==
           c4001::PresenceReportError::kNone);
    assert(presence == PresenceState::kDetected);

    const char* malformed[] = {
        "$DfHPD,0, , , *",
        "$DFHPD,9, , , *",
        "$DFHPD,1, , *",
        "$DFHPD,1, , , **",
        "$DFHPD,1,`, , *",
    };
    for (const char* text : malformed) {
        presence = PresenceState::kUnknown;
        assert(c4001::decodePresenceReport(reportFrom(text), presence) ==
               c4001::PresenceReportError::kMalformedReport);
        assert(presence == PresenceState::kUnknown);
    }
    assert(c4001::decodePresenceReport(
               reportFrom("$DFDMD,1,1,1.817,0.129,15304, , *"), presence) ==
           c4001::PresenceReportError::kUnsupportedReport);

    c4001::PresenceFilter filter;
    PresenceState filtered = PresenceState::kUnknown;
    assert(!filter.observe(PresenceState::kClear, filtered));
    assert(filter.observe(PresenceState::kClear, filtered));
    assert(filtered == PresenceState::kClear);

    // One contrary observation is treated as an outlier and refreshes the
    // last confirmed state. A second consecutive observation commits it.
    assert(filter.observe(PresenceState::kDetected, filtered));
    assert(filtered == PresenceState::kClear);
    assert(filter.observe(PresenceState::kClear, filtered));
    assert(filtered == PresenceState::kClear);
    assert(filter.observe(PresenceState::kDetected, filtered));
    assert(filtered == PresenceState::kClear);
    assert(filter.observe(PresenceState::kDetected, filtered));
    assert(filtered == PresenceState::kDetected);

    filter.reset();
    assert(!filter.observe(PresenceState::kUnknown, filtered));
    assert(!filter.observe(PresenceState::kDetected, filtered));
    assert(filter.observe(PresenceState::kDetected, filtered));
    assert(filtered == PresenceState::kDetected);

    const auto frame = c4001::makePresenceFrame(PresenceState::kDetected, 7U, 1234U);
    assert(frame.sequence == 7U);
    assert(frame.sensor_timestamp_ms == 1234U);
    assert(frame.capabilities.flags == n203::radar::kPresence);
    assert(frame.health == n203::radar::RadarHealth::kHealthy);
    assert(frame.measurement_state == n203::radar::MeasurementState::kPresenceOnly);
    assert(frame.presence == PresenceState::kDetected);
    assert(frame.motion == n203::radar::MotionState::kUnknown);
    assert(frame.motion_quality == 0U);
    assert(frame.contact_count == 0U);
    assert(n203::radar::validateFrame(frame) == n203::radar::FrameValidationError::kNone);
}
