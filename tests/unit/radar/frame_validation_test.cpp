#include <cassert>
#include <iostream>

#include "n203/radar/frame_validation.hpp"

void runDisplayAppTests();
void runMotionEstimatorTests();
void runC4001PresencePipelineTests();
void runC4001RangePipelineTests();
void runC4001ReportStreamTests();
void runRadarLinkTests();
void runRd03dPositionTrackerTests();
void runRd03dRangePipelineTests();
void runRd03dRawFrameStreamTests();
void runSimulatedRadarSourceTests();

int main() {
    n203::radar::RadarFrame valid{};
    valid.capabilities.flags =
        n203::radar::kPosition | n203::radar::kRange | n203::radar::kMultipleContacts;
    valid.contact_count = 1;
    valid.contacts[0].id = 3;
    valid.contacts[0].valid_fields = n203::radar::kContactPosition | n203::radar::kContactRange;
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kNone);

    valid.contact_count = 2;
    valid.contacts[1] = valid.contacts[0];
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kDuplicateContactId);

    valid.contact_count = static_cast<std::uint8_t>(n203::radar::kMaximumContacts + 1U);
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kTooManyContacts);

    n203::radar::RadarFrame invalid_enum{};
    invalid_enum.health = static_cast<n203::radar::RadarHealth>(0x7F);
    assert(n203::radar::validateFrame(invalid_enum) ==
           n203::radar::FrameValidationError::kInvalidHealth);
    invalid_enum = {};
    invalid_enum.measurement_state = static_cast<n203::radar::MeasurementState>(0x7F);
    assert(n203::radar::validateFrame(invalid_enum) ==
           n203::radar::FrameValidationError::kInvalidMeasurementState);

    n203::radar::RadarFrame presence{};
    presence.capabilities.flags = n203::radar::kPresence;
    presence.presence = n203::radar::PresenceState::kDetected;
    assert(n203::radar::validateFrame(presence) == n203::radar::FrameValidationError::kNone);

    presence.capabilities.flags = 0;
    assert(n203::radar::validateFrame(presence) ==
           n203::radar::FrameValidationError::kPresenceUnsupported);
    presence.capabilities.flags = n203::radar::kPresence;

    presence.presence = n203::radar::PresenceState::kClear;
    presence.contact_count = 1;
    presence.contacts[0].id = 1;
    assert(n203::radar::validateFrame(presence) ==
           n203::radar::FrameValidationError::kPresenceClearWithContacts);

    presence = {};
    presence.motion = n203::radar::MotionState::kUnknown;
    presence.motion_quality = 1;
    assert(n203::radar::validateFrame(presence) ==
           n203::radar::FrameValidationError::kInvalidMotionQuality);

    n203::radar::RadarFrame range_only{};
    range_only.capabilities.flags = n203::radar::kRange | n203::radar::kPosition;
    range_only.measurement_state = n203::radar::MeasurementState::kRangeOnly;
    range_only.contact_count = 1;
    range_only.contacts[0].id = 1;
    range_only.contacts[0].valid_fields = n203::radar::kContactPosition;
    assert(n203::radar::validateFrame(range_only) ==
           n203::radar::FrameValidationError::kRangeOnlyHasDirectionalField);

    presence.motion = n203::radar::MotionState::kDopplerWarning;
    presence.motion_quality = 101;
    assert(n203::radar::validateFrame(presence) ==
           n203::radar::FrameValidationError::kInvalidMotionQuality);

    runDisplayAppTests();
    runMotionEstimatorTests();
    runC4001PresencePipelineTests();
    runC4001RangePipelineTests();
    runC4001ReportStreamTests();
    runRadarLinkTests();
    runRd03dPositionTrackerTests();
    runRd03dRangePipelineTests();
    runRd03dRawFrameStreamTests();
    runSimulatedRadarSourceTests();
    std::cout << "N203 unit tests passed.\n";
    return 0;
}
