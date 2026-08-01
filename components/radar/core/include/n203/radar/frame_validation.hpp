#pragma once

#include <cstdint>

#include "n203/radar/radar_frame.hpp"

namespace n203::radar {

enum class FrameValidationError : std::uint8_t {
    kNone,
    kTooManyContacts,
    kDuplicateContactId,
    kMissingPositionFields,
    kMissingRangeField,
};

[[nodiscard]] FrameValidationError validateFrame(const RadarFrame& frame);

}  // namespace n203::radar
