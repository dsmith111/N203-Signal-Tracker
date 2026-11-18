#include "n203/radar/frame_validation.hpp"

#include <array>

namespace n203::radar {

FrameValidationError validateFrame(const RadarFrame& frame) {
    if (frame.contact_count > kMaximumContacts) {
        return FrameValidationError::kTooManyContacts;
    }
    if (frame.contact_count > 1U && !frame.capabilities.supports(kMultipleContacts)) {
        return FrameValidationError::kMultipleContactsUnsupported;
    }
    if (static_cast<std::uint8_t>(frame.health) >
        static_cast<std::uint8_t>(RadarHealth::kOffline)) {
        return FrameValidationError::kInvalidHealth;
    }
    if (static_cast<std::uint8_t>(frame.measurement_state) >
        static_cast<std::uint8_t>(MeasurementState::kOffline)) {
        return FrameValidationError::kInvalidMeasurementState;
    }
    if (static_cast<std::uint8_t>(frame.presence) >
        static_cast<std::uint8_t>(PresenceState::kDetected)) {
        return FrameValidationError::kInvalidPresenceState;
    }
    if (frame.presence != PresenceState::kUnknown &&
        !frame.capabilities.supports(kPresence)) {
        return FrameValidationError::kPresenceUnsupported;
    }
    if (frame.presence == PresenceState::kClear && frame.contact_count != 0U) {
        return FrameValidationError::kPresenceClearWithContacts;
    }
    if (static_cast<std::uint8_t>(frame.motion) >
        static_cast<std::uint8_t>(MotionState::kStabilizing)) {
        return FrameValidationError::kInvalidMotionState;
    }
    if (frame.motion_quality > 100U ||
        (frame.motion == MotionState::kUnknown && frame.motion_quality != 0U)) {
        return FrameValidationError::kInvalidMotionQuality;
    }

    std::array<bool, 256> seen_ids{};
    for (std::size_t index = 0; index < frame.contact_count; ++index) {
        const Contact& contact = frame.contacts[index];
        if (seen_ids[contact.id]) {
            return FrameValidationError::kDuplicateContactId;
        }
        seen_ids[contact.id] = true;

        if (contact.has(kContactPosition) && !frame.capabilities.supports(kPosition)) {
            return FrameValidationError::kMissingPositionFields;
        }
        if (contact.has(kContactRange) && !frame.capabilities.supports(kRange)) {
            return FrameValidationError::kMissingRangeField;
        }
        if (contact.has(kContactAzimuth) && !frame.capabilities.supports(kAzimuth)) {
            return FrameValidationError::kAzimuthUnsupported;
        }
        if (contact.has(kContactRadialVelocity) && !frame.capabilities.supports(kVelocity)) {
            return FrameValidationError::kVelocityUnsupported;
        }
        if (frame.measurement_state == MeasurementState::kRangeOnly &&
            (contact.has(kContactPosition) || contact.has(kContactAzimuth))) {
            return FrameValidationError::kRangeOnlyHasDirectionalField;
        }
    }
    return FrameValidationError::kNone;
}

}  // namespace n203::radar
