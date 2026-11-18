#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/radar/capabilities.hpp"
#include "n203/radar/contact.hpp"

namespace n203::radar {

constexpr std::size_t kMaximumContacts = 8;

enum class PresenceState : std::uint8_t {
    kUnknown,
    kClear,
    kDetected,
};

enum class MotionState : std::uint8_t {
    kUnknown,
    kTracking,
    kDopplerWarning,
    kStabilizing,
};

struct RadarFrame {
    std::uint32_t sequence = 0;
    std::uint32_t sensor_timestamp_ms = 0;
    RadarCapabilities capabilities{};
    RadarHealth health = RadarHealth::kUnknown;
    MeasurementState measurement_state = MeasurementState::kInitializing;
    PresenceState presence = PresenceState::kUnknown;
    MotionState motion = MotionState::kUnknown;
    std::uint8_t motion_quality = 0;
    std::uint8_t contact_count = 0;
    std::array<Contact, kMaximumContacts> contacts{};
};

}  // namespace n203::radar
