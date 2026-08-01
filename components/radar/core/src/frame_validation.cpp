#include "n203/radar/frame_validation.hpp"

#include <array>

namespace n203::radar {

FrameValidationError validateFrame(const RadarFrame& frame) {
    if (frame.contact_count > kMaximumContacts) {
        return FrameValidationError::kTooManyContacts;
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
    }
    return FrameValidationError::kNone;
}

}  // namespace n203::radar
