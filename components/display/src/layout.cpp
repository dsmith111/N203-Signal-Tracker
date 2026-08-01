#include "n203/display/layout.hpp"

namespace n203::display {

bool Rect::contains(Point point) const {
    return point.x >= x && point.x < x + width && point.y >= y && point.y < y + height;
}

Point DisplayLayout::radarCenter() {
    return {240, 276};
}

int DisplayLayout::radarRadius() {
    return 260;
}

Rect DisplayLayout::radarBounds() {
    const Point center = radarCenter();
    const int radius = radarRadius();
    return {center.x - radius, center.y - radius, radius * 2, radius * 2};
}

Rect DisplayLayout::rangeDecreaseBounds() {
    return {8, 286, 48, 26};
}

Rect DisplayLayout::rangeIncreaseBounds() {
    return {62, 286, 48, 26};
}

}  // namespace n203::display
