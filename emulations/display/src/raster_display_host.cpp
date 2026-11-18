#include "n203/emulation/display/raster_display_host.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "n203/display/font3x5.hpp"

namespace n203::emulation::display {

RasterDisplayHost::RasterDisplayHost()
    : pixels_(n203::display::DisplayLayout::kLogicalWidth * n203::display::DisplayLayout::kLogicalHeight * 3U, 0) {}

void RasterDisplayHost::clear(n203::display::Color color) {
    for (std::size_t index = 0; index < pixels_.size(); index += 3U) {
        pixels_[index] = color.red;
        pixels_[index + 1U] = color.green;
        pixels_[index + 2U] = color.blue;
    }
}

void RasterDisplayHost::fillRect(int x, int y, int width, int height, n203::display::Color color) {
    const int x_start = std::max(0, x);
    const int y_start = std::max(0, y);
    const int x_end = std::min(n203::display::DisplayLayout::kLogicalWidth, x + width);
    const int y_end = std::min(n203::display::DisplayLayout::kLogicalHeight, y + height);
    for (int row = y_start; row < y_end; ++row) {
        for (int column = x_start; column < x_end; ++column) {
            setPixel(column, row, color);
        }
    }
}

void RasterDisplayHost::strokeRect(int x, int y, int width, int height, n203::display::Color color) {
    drawLine(x, y, x + width - 1, y, color);
    drawLine(x, y, x, y + height - 1, color);
    drawLine(x + width - 1, y, x + width - 1, y + height - 1, color);
    drawLine(x, y + height - 1, x + width - 1, y + height - 1, color);
}

void RasterDisplayHost::drawLine(int x0, int y0, int x1, int y1, n203::display::Color color) {
    const int delta_x = std::abs(x1 - x0);
    const int delta_y = -std::abs(y1 - y0);
    const int step_x = x0 < x1 ? 1 : -1;
    const int step_y = y0 < y1 ? 1 : -1;
    int error = delta_x + delta_y;
    while (true) {
        setPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int twice_error = 2 * error;
        if (twice_error >= delta_y) {
            error += delta_y;
            x0 += step_x;
        }
        if (twice_error <= delta_x) {
            error += delta_x;
            y0 += step_y;
        }
    }
}

void RasterDisplayHost::fillCircle(int x, int y, int radius, n203::display::Color color) {
    for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
        const int half_width = static_cast<int>(std::sqrt(radius * radius - offset_y * offset_y));
        drawLine(x - half_width, y + offset_y, x + half_width, y + offset_y, color);
    }
}

void RasterDisplayHost::strokeCircle(int x, int y, int radius, n203::display::Color color) {
    int offset_x = radius;
    int offset_y = 0;
    int decision = 1 - radius;
    while (offset_x >= offset_y) {
        setPixel(x + offset_x, y + offset_y, color);
        setPixel(x + offset_y, y + offset_x, color);
        setPixel(x - offset_y, y + offset_x, color);
        setPixel(x - offset_x, y + offset_y, color);
        setPixel(x - offset_x, y - offset_y, color);
        setPixel(x - offset_y, y - offset_x, color);
        setPixel(x + offset_y, y - offset_x, color);
        setPixel(x + offset_x, y - offset_y, color);
        ++offset_y;
        if (decision <= 0) {
            decision += 2 * offset_y + 1;
        } else {
            --offset_x;
            decision += 2 * (offset_y - offset_x) + 1;
        }
    }
}

void RasterDisplayHost::drawText(int x, int y, std::string_view text, n203::display::Color color, int scale) {
    int cursor_x = x;
    for (char glyph : text) {
        drawGlyph(cursor_x, y, glyph, color, scale);
        cursor_x += n203::display::font3x5::kGlyphAdvance * scale;
    }
}

bool RasterDisplayHost::writePpm(const std::string& path, std::string& error) const {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        error = "Unable to create screenshot: " + path;
        return false;
    }
    output << "P6\n" << n203::display::DisplayLayout::kLogicalWidth << " "
           << n203::display::DisplayLayout::kLogicalHeight << "\n255\n";
    output.write(reinterpret_cast<const char*>(pixels_.data()), static_cast<std::streamsize>(pixels_.size()));
    if (!output) {
        error = "Unable to write screenshot: " + path;
        return false;
    }
    return true;
}

void RasterDisplayHost::setPixel(int x, int y, n203::display::Color color) {
    if (x < 0 || x >= n203::display::DisplayLayout::kLogicalWidth ||
        y < 0 || y >= n203::display::DisplayLayout::kLogicalHeight) {
        return;
    }
    const std::size_t index = static_cast<std::size_t>(y * n203::display::DisplayLayout::kLogicalWidth + x) * 3U;
    pixels_[index] = color.red;
    pixels_[index + 1U] = color.green;
    pixels_[index + 2U] = color.blue;
}

void RasterDisplayHost::drawGlyph(int x, int y, char glyph, n203::display::Color color, int scale) {
    const char* rows = n203::display::font3x5::glyphRows(glyph);
    for (int row = 0; row < n203::display::font3x5::kGlyphHeight; ++row) {
        for (int column = 0; column < n203::display::font3x5::kGlyphWidth; ++column) {
            if (rows[row * n203::display::font3x5::kGlyphWidth + column] == '1') {
                fillRect(x + column * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

}  // namespace n203::emulation::display
