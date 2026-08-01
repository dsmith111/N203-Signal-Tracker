#pragma once

namespace n203::display {

struct Point {
    int x = 0;
    int y = 0;
};

struct Rect {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;

    [[nodiscard]] bool contains(Point point) const;
};

struct DisplayLayout {
    static constexpr int kLogicalWidth = 480;
    static constexpr int kLogicalHeight = 320;

    [[nodiscard]] static Point radarCenter();
    [[nodiscard]] static int radarRadius();
    [[nodiscard]] static Rect radarBounds();
    [[nodiscard]] static Rect rangeDecreaseBounds();
    [[nodiscard]] static Rect rangeIncreaseBounds();
};

}  // namespace n203::display
