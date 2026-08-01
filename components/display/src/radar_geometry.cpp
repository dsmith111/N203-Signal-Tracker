#include "n203/display/radar_geometry.hpp"

#include <cmath>

namespace n203::display {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfFieldOfViewDegrees = 40.0;
}

bool radarPointForContact(const radar::Contact& contact, std::uint16_t display_range_mm, Point& point) {
    if (display_range_mm == 0U) {
        return false;
    }

    double x_mm = 0.0;
    double y_mm = 0.0;
    if (contact.has(radar::kContactPosition)) {
        x_mm = contact.x_mm;
        y_mm = contact.y_mm;
    } else if (contact.has(radar::kContactRange) && contact.has(radar::kContactAzimuth)) {
        const double azimuth_rad = static_cast<double>(contact.azimuth_cdeg) * kPi / 18000.0;
        x_mm = std::sin(azimuth_rad) * contact.range_mm;
        y_mm = std::cos(azimuth_rad) * contact.range_mm;
    } else if (contact.has(radar::kContactRange)) {
        // A range-only source has no defensible bearing. Keep its marker on the
        // forward axis rather than inventing a left/right position.
        y_mm = contact.range_mm;
    } else {
        return false;
    }

    const double range = std::sqrt(x_mm * x_mm + y_mm * y_mm);
    const double bearing_degrees = std::atan2(x_mm, y_mm) * 180.0 / kPi;
    if (range > display_range_mm || y_mm < 0.0 || std::abs(bearing_degrees) > kHalfFieldOfViewDegrees) {
        return false;
    }

    const Point center = DisplayLayout::radarCenter();
    const double pixels_per_mm = static_cast<double>(DisplayLayout::radarRadius()) / display_range_mm;
    point.x = center.x + static_cast<int>(std::lround(x_mm * pixels_per_mm));
    point.y = center.y - static_cast<int>(std::lround(y_mm * pixels_per_mm));
    return true;
}

int squaredDistance(Point left, Point right) {
    const int dx = left.x - right.x;
    const int dy = left.y - right.y;
    return dx * dx + dy * dy;
}

}  // namespace n203::display
