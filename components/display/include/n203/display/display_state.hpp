#pragma once

#include <cstdint>

#include "n203/radar/radar_frame.hpp"

namespace n203::display {

enum class ScannerPresentation : std::uint8_t {
    kStarting,
    kScanning,
    kRangeOnly,
    kPresenceOnly,
    kStale,
    kOffline,
    kInvalidFrame,
};

struct DisplayState {
    radar::RadarFrame frame{};
    ScannerPresentation scanner = ScannerPresentation::kStarting;
    std::uint16_t display_range_mm = 10000;
    std::uint8_t selected_contact_id = 0;
    bool has_selected_contact = false;
    bool target_locked = false;
};

}  // namespace n203::display
