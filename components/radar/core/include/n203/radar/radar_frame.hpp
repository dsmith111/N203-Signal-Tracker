#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/radar/capabilities.hpp"
#include "n203/radar/contact.hpp"

namespace n203::radar {

constexpr std::size_t kMaximumContacts = 8;

struct RadarFrame {
    std::uint32_t sequence = 0;
    std::uint32_t sensor_timestamp_ms = 0;
    RadarCapabilities capabilities{};
    RadarHealth health = RadarHealth::kUnknown;
    MeasurementState measurement_state = MeasurementState::kInitializing;
    std::uint8_t contact_count = 0;
    std::array<Contact, kMaximumContacts> contacts{};
};

}  // namespace n203::radar
