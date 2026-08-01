#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "n203/display/canvas.hpp"
#include "n203/display/layout.hpp"

namespace n203::emulation::display {

class RasterDisplayHost final : public n203::display::DisplayCanvas {
public:
    RasterDisplayHost();

    void clear(n203::display::Color color) override;
    void fillRect(int x, int y, int width, int height, n203::display::Color color) override;
    void strokeRect(int x, int y, int width, int height, n203::display::Color color) override;
    void drawLine(int x0, int y0, int x1, int y1, n203::display::Color color) override;
    void fillCircle(int x, int y, int radius, n203::display::Color color) override;
    void strokeCircle(int x, int y, int radius, n203::display::Color color) override;
    void drawText(int x, int y, std::string_view text, n203::display::Color color, int scale = 1) override;

    [[nodiscard]] bool writePpm(const std::string& path, std::string& error) const;
    [[nodiscard]] const std::vector<std::uint8_t>& pixels() const { return pixels_; }

private:
    void setPixel(int x, int y, n203::display::Color color);
    void drawGlyph(int x, int y, char glyph, n203::display::Color color, int scale);

    std::vector<std::uint8_t> pixels_;
};

}  // namespace n203::emulation::display
