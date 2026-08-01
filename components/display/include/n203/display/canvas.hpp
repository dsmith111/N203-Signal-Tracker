#pragma once

#include <string_view>

#include "n203/display/theme.hpp"

namespace n203::display {

class DisplayCanvas {
public:
    virtual ~DisplayCanvas() = default;
    virtual void clear(Color color) = 0;
    virtual void fillRect(int x, int y, int width, int height, Color color) = 0;
    virtual void strokeRect(int x, int y, int width, int height, Color color) = 0;
    virtual void drawLine(int x0, int y0, int x1, int y1, Color color) = 0;
    virtual void fillCircle(int x, int y, int radius, Color color) = 0;
    virtual void strokeCircle(int x, int y, int radius, Color color) = 0;
    virtual void drawText(int x, int y, std::string_view text, Color color, int scale = 1) = 0;
};

}  // namespace n203::display
