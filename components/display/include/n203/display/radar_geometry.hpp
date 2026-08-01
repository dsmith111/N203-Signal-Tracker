#pragma once

#include "n203/display/layout.hpp"
#include "n203/radar/contact.hpp"

namespace n203::display {

[[nodiscard]] bool radarPointForContact(
    const radar::Contact& contact,
    std::uint16_t display_range_mm,
    Point& point);

[[nodiscard]] int squaredDistance(Point left, Point right);

}  // namespace n203::display
