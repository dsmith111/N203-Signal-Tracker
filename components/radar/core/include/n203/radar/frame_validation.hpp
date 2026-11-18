#pragma once

#include <cstdint>

#include "n203/radar/radar_frame.hpp"

namespace n203::radar {

enum class FrameValidationError : std::uint8_t {
    kNone,
    kTooManyContacts,
    kMultipleContactsUnsupported,
    kDuplicateContactId,
    kInvalidHealth,
    kInvalidMeasurementState,
    kMissingPositionFields,
    kMissingRangeField,
    kAzimuthUnsupported,
    kVelocityUnsupported,
    kInvalidPresenceState,
    kPresenceUnsupported,
    kPresenceClearWithContacts,
    kInvalidMotionState,
    kInvalidMotionQuality,
    kRangeOnlyHasDirectionalField,
};

[[nodiscard]] FrameValidationError validateFrame(const RadarFrame& frame);

}  // namespace n203::radar
