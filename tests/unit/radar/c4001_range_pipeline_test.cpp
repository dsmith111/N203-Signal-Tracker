#include <cassert>
#include <cstring>

#include "n203/radar/c4001/range_pipeline.hpp"
#include "n203/radar/frame_validation.hpp"

namespace {

namespace c4001 = n203::radar::c4001;

c4001::Report rangeReportFrom(const char* text) {
    c4001::Report report{};
    report.size = std::strlen(text);
    assert(report.size <= c4001::kMaximumReportSize);
    std::memcpy(report.text.data(), text, report.size);
    return report;
}

}  // namespace

void runC4001RangePipelineTests() {
    c4001::RangeMeasurement measurement{};
    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,1,1,1.817,0.129,15304, , *"), measurement) ==
           c4001::RangeReportError::kNone);
    assert(measurement.target_count == 1U);
    assert(measurement.range_mm == 1817U);
    assert(measurement.radial_velocity_mm_s == 129);
    assert(measurement.signal == 15304U);

    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,1,1,25.000,-10.000,65535, , *"), measurement) ==
           c4001::RangeReportError::kNone);
    assert(measurement.range_mm == 25000U);
    assert(measurement.radial_velocity_mm_s == -10000);

    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,1,1,0.430,0.197,83450681, , *"), measurement) ==
           c4001::RangeReportError::kNone);
    assert(measurement.range_mm == 430U);
    assert(measurement.radial_velocity_mm_s == 197);
    assert(measurement.signal == 65535U);

    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,0, , , , , , *"), measurement) ==
           c4001::RangeReportError::kNone);
    assert(measurement.target_count == 0U);

    const char* malformed[] = {
        "$DFDMD,2,1,1.817,0.129,15304, , *",
        "$DFDMD,1,1,1.817x,0.129,15304, , *",
        "$DFDMD,1,1,1.817,0.129,4294967296, , *",
        "$DFDMD,1,1,1.817,0.129,15304, *",
        "$DFDMD,1,1,1.817,0.129,15304, , *,extra",
    };
    for (const char* text : malformed) {
        assert(c4001::decodeRangeReport(rangeReportFrom(text), measurement) ==
               c4001::RangeReportError::kMalformedReport);
    }
    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,1,1,25.001,0.129,15304, , *"), measurement) ==
           c4001::RangeReportError::kOutOfRange);
    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFDMD,1,1,1.817,10.001,15304, , *"), measurement) ==
           c4001::RangeReportError::kOutOfRange);
    assert(c4001::decodeRangeReport(
               rangeReportFrom("$DFHPD,1, , , *"), measurement) ==
           c4001::RangeReportError::kUnsupportedReport);

    c4001::RangeFilter filter;
    c4001::RangeMeasurement filtered{};
    const c4001::RangeMeasurement first_target{1U, 1800U, 100, 1000U};
    const c4001::RangeMeasurement moved_target{1U, 1900U, 120, 1100U};
    const c4001::RangeMeasurement no_target{};
    assert(!filter.observe(first_target, filtered));
    assert(filter.observe(moved_target, filtered));
    assert(filtered.target_count == 1U);
    assert(filtered.range_mm == 1900U);
    assert(filter.observe(no_target, filtered));
    assert(filtered.target_count == 1U);
    assert(filter.observe(moved_target, filtered));
    assert(filtered.target_count == 1U);
    assert(filter.observe(no_target, filtered));
    assert(filtered.target_count == 1U);
    assert(filter.observe(no_target, filtered));
    assert(filtered.target_count == 0U);

    filter.reset();
    assert(!filter.observe(no_target, filtered));
    assert(filter.observe(no_target, filtered));
    assert(filtered.target_count == 0U);

    measurement = {1U, 1817U, 129, 15304U};
    const auto target_frame = c4001::makeRangeFrame(measurement, 8U, 4321U);
    assert(target_frame.sequence == 8U);
    assert(target_frame.sensor_timestamp_ms == 4321U);
    assert(target_frame.capabilities.flags ==
           (n203::radar::kRange | n203::radar::kVelocity));
    assert(target_frame.measurement_state == n203::radar::MeasurementState::kRangeOnly);
    assert(target_frame.presence == n203::radar::PresenceState::kUnknown);
    assert(target_frame.motion == n203::radar::MotionState::kTracking);
    assert(target_frame.contact_count == 1U);
    assert(target_frame.contacts[0].valid_fields ==
           (n203::radar::kContactRange |
            n203::radar::kContactRadialVelocity |
            n203::radar::kContactSignal));
    assert(target_frame.contacts[0].range_mm == 1817U);
    assert(target_frame.contacts[0].radial_velocity_mm_s == 129);
    assert(target_frame.contacts[0].signal == 15304U);
    assert(n203::radar::validateFrame(target_frame) ==
           n203::radar::FrameValidationError::kNone);

    measurement = {};
    const auto empty_frame = c4001::makeRangeFrame(measurement, 9U, 5000U);
    assert(empty_frame.contact_count == 0U);
    assert(empty_frame.motion == n203::radar::MotionState::kUnknown);
    assert(n203::radar::validateFrame(empty_frame) ==
           n203::radar::FrameValidationError::kNone);
}
