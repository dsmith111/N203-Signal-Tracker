#pragma once

#include <cstdint>

namespace n203::radar {

enum RadarCapability : std::uint32_t {
    kPresence = 1U << 0U,
    kRange = 1U << 1U,
    kPosition = 1U << 2U,
    kAzimuth = 1U << 3U,
    kVelocity = 1U << 4U,
    kTrackIdentity = 1U << 5U,
    kMultipleContacts = 1U << 6U,
};

struct RadarCapabilities {
    std::uint32_t flags = 0;

    [[nodiscard]] bool supports(RadarCapability capability) const {
        return (flags & static_cast<std::uint32_t>(capability)) != 0U;
    }
};

enum class RadarHealth : std::uint8_t {
    kUnknown,
    kHealthy,
    kDegraded,
    kOffline,
};

enum class MeasurementState : std::uint8_t {
    kInitializing,
    kScanning,
    kRangeOnly,
    kPresenceOnly,
    kStale,
    kOffline,
};

}  // namespace n203::radar
