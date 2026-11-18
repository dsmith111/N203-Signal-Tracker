#pragma once

#include <cstddef>
#include <cstdint>

#include <LovyanGFX.hpp>

#include "board_config.hpp"
#include "n203_lgfx_device.hpp"

namespace n203::firmware::display_s3 {

struct DirtyTilePresentResult {
    std::uint16_t dirty_tiles = 0;
    std::uint32_t dirty_pixels = 0;
    bool full_refresh = false;
};

class DirtyTileFramebuffer final {
public:
    explicit DirtyTileFramebuffer(N203LgfxDevice& display);

    [[nodiscard]] bool begin();
    [[nodiscard]] lgfx::LGFX_Sprite& renderTarget() { return *render_target_; }
    [[nodiscard]] DirtyTilePresentResult present();

private:
    static constexpr int kTileSize = 16;
    static constexpr int kTileColumns =
        (board::kLogicalWidth + kTileSize - 1) / kTileSize;
    static constexpr int kTileRows =
        (board::kLogicalHeight + kTileSize - 1) / kTileSize;
    static constexpr std::size_t kTileCount =
        static_cast<std::size_t>(kTileColumns * kTileRows);

    [[nodiscard]] bool tileChanged(int tile_x, int tile_y) const;
    void swapBuffers();

    N203LgfxDevice& display_;
    lgfx::LGFX_Sprite frame_a_;
    lgfx::LGFX_Sprite frame_b_;
    lgfx::LGFX_Sprite* render_target_;
    lgfx::LGFX_Sprite* previous_frame_;
    bool previous_valid_ = false;
};

}  // namespace n203::firmware::display_s3
