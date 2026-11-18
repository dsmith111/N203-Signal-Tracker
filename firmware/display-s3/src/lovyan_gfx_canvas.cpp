#include "lovyan_gfx_canvas.hpp"

#include "n203/display/font3x5.hpp"

namespace n203::firmware::display_s3 {

LovyanGfxCanvas::LovyanGfxCanvas(lgfx::LovyanGFX& target)
    : target_(&target) {}

void LovyanGfxCanvas::setTarget(lgfx::LovyanGFX& target) {
    if (!frame_active_) {
        target_ = &target;
    }
}

void LovyanGfxCanvas::beginFrame() {
    if (!frame_active_) {
        target_->startWrite();
        frame_active_ = true;
    }
}

void LovyanGfxCanvas::endFrame() {
    if (frame_active_) {
        target_->endWrite();
        frame_active_ = false;
    }
}

void LovyanGfxCanvas::clear(display::Color color) {
    target_->fillScreen(deviceColor(color));
}

void LovyanGfxCanvas::fillRect(int x, int y, int width, int height, display::Color color) {
    target_->fillRect(x, y, width, height, deviceColor(color));
}

void LovyanGfxCanvas::strokeRect(int x, int y, int width, int height, display::Color color) {
    target_->drawRect(x, y, width, height, deviceColor(color));
}

void LovyanGfxCanvas::drawLine(int x0, int y0, int x1, int y1, display::Color color) {
    target_->drawLine(x0, y0, x1, y1, deviceColor(color));
}

void LovyanGfxCanvas::fillCircle(int x, int y, int radius, display::Color color) {
    target_->fillCircle(x, y, radius, deviceColor(color));
}

void LovyanGfxCanvas::strokeCircle(int x, int y, int radius, display::Color color) {
    target_->drawCircle(x, y, radius, deviceColor(color));
}

void LovyanGfxCanvas::drawText(
    int x,
    int y,
    std::string_view text,
    display::Color color,
    int scale) {
    if (scale <= 0) {
        return;
    }

    int cursor_x = x;
    for (char glyph : text) {
        drawGlyph(cursor_x, y, glyph, color, scale);
        cursor_x += display::font3x5::kGlyphAdvance * scale;
    }
}

std::uint32_t LovyanGfxCanvas::deviceColor(display::Color color) {
    return lgfx::LovyanGFX::color888(color.red, color.green, color.blue);
}

void LovyanGfxCanvas::drawGlyph(int x, int y, char glyph, display::Color color, int scale) {
    const char* rows = display::font3x5::glyphRows(glyph);
    for (int row = 0; row < display::font3x5::kGlyphHeight; ++row) {
        for (int column = 0; column < display::font3x5::kGlyphWidth; ++column) {
            if (rows[row * display::font3x5::kGlyphWidth + column] == '1') {
                fillRect(x + column * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

}  // namespace n203::firmware::display_s3
