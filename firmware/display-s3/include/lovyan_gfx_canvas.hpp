#pragma once

#include <cstdint>
#include <string_view>

#include <LovyanGFX.hpp>

#include "n203/display/canvas.hpp"

namespace n203::firmware::display_s3 {

class LovyanGfxCanvas final : public display::DisplayCanvas {
public:
    explicit LovyanGfxCanvas(lgfx::LovyanGFX& target);

    void setTarget(lgfx::LovyanGFX& target);
    void beginFrame();
    void endFrame();

    void clear(display::Color color) override;
    void fillRect(int x, int y, int width, int height, display::Color color) override;
    void strokeRect(int x, int y, int width, int height, display::Color color) override;
    void drawLine(int x0, int y0, int x1, int y1, display::Color color) override;
    void fillCircle(int x, int y, int radius, display::Color color) override;
    void strokeCircle(int x, int y, int radius, display::Color color) override;
    void drawText(int x, int y, std::string_view text, display::Color color, int scale = 1) override;

private:
    [[nodiscard]] static std::uint32_t deviceColor(display::Color color);
    void drawGlyph(int x, int y, char glyph, display::Color color, int scale);

    lgfx::LovyanGFX* target_;
    bool frame_active_ = false;
};

}  // namespace n203::firmware::display_s3
