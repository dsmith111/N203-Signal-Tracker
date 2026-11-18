#include <cassert>

#include "n203/radar/frame_validation.hpp"
#include "n203/radar/rd03d/range_pipeline.hpp"

namespace {

namespace rd03d = n203::radar::rd03d;

constexpr rd03d::RawFrame kDocumentedExample{{
    0xAA, 0xFF, 0x03, 0x00,
    0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x68, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x55, 0xCC,
}};

}  // namespace

void runRd03dRangePipelineTests() {
    rd03d::RangeReport report{};
    assert(rd03d::decodeRangeReport(kDocumentedExample, report) ==
           rd03d::RangeReportError::kNone);
    assert(report.target_count == 1U);
    assert(report.targets[0].present);
    assert(report.targets[0].slot == 0U);
    assert(report.targets[0].x_mm == -782);
    assert(report.targets[0].y_mm == 1713);
    assert(report.targets[0].range_mm == 1883U);
    assert(report.targets[0].radial_velocity_mm_s == -160);
    assert(report.targets[0].distance_resolution_mm == 360U);

    const auto frame = rd03d::makeRangeFrame(report, 12U, 3456U);
    assert(frame.sequence == 12U);
    assert(frame.sensor_timestamp_ms == 3456U);
    assert(frame.capabilities.flags ==
           (n203::radar::kPresence | n203::radar::kRange |
            n203::radar::kVelocity | n203::radar::kMultipleContacts));
    assert(frame.measurement_state == n203::radar::MeasurementState::kRangeOnly);
    assert(frame.presence == n203::radar::PresenceState::kDetected);
    assert(frame.contact_count == 1U);
    assert(frame.contacts[0].id == 1U);
    assert(frame.contacts[0].valid_fields ==
           (n203::radar::kContactRange |
            n203::radar::kContactRadialVelocity));
    assert(frame.contacts[0].range_mm == 1883U);
    assert(frame.contacts[0].radial_velocity_mm_s == -160);
    assert(n203::radar::validateFrame(frame) ==
           n203::radar::FrameValidationError::kNone);

    const auto position_frame = rd03d::makePositionFrame(report, 15U, 3756U);
    assert(position_frame.sequence == 15U);
    assert(position_frame.measurement_state ==
           n203::radar::MeasurementState::kScanning);
    assert(position_frame.capabilities.flags ==
           (n203::radar::kPresence | n203::radar::kRange |
            n203::radar::kPosition | n203::radar::kAzimuth |
            n203::radar::kVelocity | n203::radar::kMultipleContacts));
    assert(!position_frame.capabilities.supports(n203::radar::kTrackIdentity));
    assert(position_frame.contact_count == 1U);
    assert(position_frame.contacts[0].valid_fields ==
           (n203::radar::kContactPosition | n203::radar::kContactRange |
            n203::radar::kContactAzimuth |
            n203::radar::kContactRadialVelocity));
    assert(position_frame.contacts[0].x_mm == 782);
    assert(position_frame.contacts[0].y_mm == 1713);
    assert(position_frame.contacts[0].range_mm == 1883U);
    assert(position_frame.contacts[0].azimuth_cdeg == 2454);
    assert(position_frame.contacts[0].radial_velocity_mm_s == -160);
    assert(n203::radar::validateFrame(position_frame) ==
           n203::radar::FrameValidationError::kNone);

    rd03d::RawFrame empty = kDocumentedExample;
    for (std::size_t index = 4U; index < 28U; ++index) {
        empty[index] = 0U;
    }
    assert(rd03d::decodeRangeReport(empty, report) ==
           rd03d::RangeReportError::kNone);
    assert(report.target_count == 0U);
    const auto empty_frame = rd03d::makeRangeFrame(report, 13U, 3556U);
    assert(empty_frame.presence == n203::radar::PresenceState::kClear);
    assert(empty_frame.contact_count == 0U);
    assert(n203::radar::validateFrame(empty_frame) ==
           n203::radar::FrameValidationError::kNone);

    rd03d::RawFrame second_slot = empty;
    second_slot[12] = 0x2CU;  // +300 mm X.
    second_slot[13] = 0x81U;
    second_slot[14] = 0x90U;  // +400 mm Y.
    second_slot[15] = 0x81U;
    second_slot[16] = 0x05U;  // +5 cm/s.
    second_slot[17] = 0x80U;
    second_slot[18] = 0x64U;  // 100 mm resolution.
    second_slot[19] = 0x00U;
    assert(rd03d::decodeRangeReport(second_slot, report) ==
           rd03d::RangeReportError::kNone);
    assert(report.target_count == 1U);
    assert(!report.targets[0].present);
    assert(report.targets[1].present);
    assert(report.targets[1].range_mm == 500U);
    assert(report.targets[1].radial_velocity_mm_s == 50);
    const auto second_slot_frame = rd03d::makeRangeFrame(report, 14U, 3656U);
    assert(second_slot_frame.contact_count == 1U);
    assert(second_slot_frame.contacts[0].id == 2U);
    assert(second_slot_frame.contacts[0].range_mm == 500U);
    assert(n203::radar::validateFrame(second_slot_frame) ==
           n203::radar::FrameValidationError::kNone);
    const auto second_slot_position =
        rd03d::makePositionFrame(report, 16U, 3856U);
    assert(second_slot_position.contact_count == 1U);
    assert(second_slot_position.contacts[0].id == 2U);
    assert(second_slot_position.contacts[0].x_mm == -300);
    assert(second_slot_position.contacts[0].y_mm == 400);
    assert(second_slot_position.contacts[0].azimuth_cdeg == -3687);
    assert(n203::radar::validateFrame(second_slot_position) ==
           n203::radar::FrameValidationError::kNone);

    rd03d::RawFrame malformed = kDocumentedExample;
    malformed[29] = 0U;
    assert(rd03d::decodeRangeReport(malformed, report) ==
           rd03d::RangeReportError::kMalformedFrame);

    rd03d::RawFrame too_far = empty;
    too_far[4] = 0x00U;
    too_far[5] = 0x80U;
    too_far[6] = 0x28U;
    too_far[7] = 0xA3U;  // +9000 mm on Y.
    assert(rd03d::decodeRangeReport(too_far, report) ==
           rd03d::RangeReportError::kOutOfRange);

    rd03d::RawFrame too_fast = kDocumentedExample;
    too_fast[8] = 0xA0U;
    too_fast[9] = 0x8FU;  // +4000 cm/s cannot fit canonical mm/s.
    assert(rd03d::decodeRangeReport(too_fast, report) ==
           rd03d::RangeReportError::kOutOfRange);
}
