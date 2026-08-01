#pragma once

#include <cstdint>

namespace n203::radar {

enum ContactField : std::uint16_t {
    kContactPosition = 1U << 0U,
    kContactRange = 1U << 1U,
    kContactAzimuth = 1U << 2U,
    kContactRadialVelocity = 1U << 3U,
    kContactSignal = 1U << 4U,
    kContactConfidence = 1U << 5U,
};

struct Contact {
    std::uint8_t id = 0;
    std::uint16_t valid_fields = 0;

    std::int16_t x_mm = 0;
    std::int16_t y_mm = 0;
    std::uint16_t range_mm = 0;
    std::int16_t azimuth_cdeg = 0;
    std::int16_t radial_velocity_mm_s = 0;
    std::uint16_t signal = 0;
    std::uint8_t confidence = 0;

    [[nodiscard]] bool has(ContactField field) const {
        return (valid_fields & static_cast<std::uint16_t>(field)) != 0U;
    }
};

}  // namespace n203::radar
