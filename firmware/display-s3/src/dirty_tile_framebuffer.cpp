#include "dirty_tile_framebuffer.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace n203::firmware::display_s3 {

DirtyTileFramebuffer::DirtyTileFramebuffer(N203LgfxDevice& display)
    : display_(display),
      frame_a_(&display),
      frame_b_(&display),
      render_target_(&frame_a_),
      previous_frame_(&frame_b_) {}

bool DirtyTileFramebuffer::begin() {
    frame_a_.setPsram(true);
    frame_b_.setPsram(true);
    frame_a_.setColorDepth(16);
    frame_b_.setColorDepth(16);

    if (frame_a_.createSprite(board::kLogicalWidth, board::kLogicalHeight) == nullptr ||
        frame_b_.createSprite(board::kLogicalWidth, board::kLogicalHeight) == nullptr) {
        frame_a_.deleteSprite();
        frame_b_.deleteSprite();
        return false;
    }

    frame_a_.fillScreen(0);
    frame_b_.fillScreen(0);
    previous_valid_ = false;
    return true;
}

DirtyTilePresentResult DirtyTileFramebuffer::present() {
    DirtyTilePresentResult result{};
    if (!previous_valid_) {
        display_.startWrite();
        render_target_->pushSprite(&display_, 0, 0);
        display_.endWrite();
        result.dirty_tiles = static_cast<std::uint16_t>(kTileCount);
        result.dirty_pixels =
            static_cast<std::uint32_t>(board::kLogicalWidth * board::kLogicalHeight);
        result.full_refresh = true;
        previous_valid_ = true;
        swapBuffers();
        return result;
    }

    std::array<bool, kTileCount> dirty{};
    for (int tile_y = 0; tile_y < kTileRows; ++tile_y) {
        for (int tile_x = 0; tile_x < kTileColumns; ++tile_x) {
            const std::size_t index = static_cast<std::size_t>(tile_y * kTileColumns + tile_x);
            dirty[index] = tileChanged(tile_x, tile_y);
            if (dirty[index]) {
                ++result.dirty_tiles;
                const int width = std::min(kTileSize, board::kLogicalWidth - tile_x * kTileSize);
                const int height = std::min(kTileSize, board::kLogicalHeight - tile_y * kTileSize);
                result.dirty_pixels += static_cast<std::uint32_t>(width * height);
            }
        }
    }

    if (result.dirty_tiles != 0U) {
        display_.startWrite();
        for (int tile_y = 0; tile_y < kTileRows; ++tile_y) {
            int tile_x = 0;
            while (tile_x < kTileColumns) {
                const std::size_t index = static_cast<std::size_t>(tile_y * kTileColumns + tile_x);
                if (!dirty[index]) {
                    ++tile_x;
                    continue;
                }

                const int run_start = tile_x;
                do {
                    ++tile_x;
                } while (tile_x < kTileColumns &&
                         dirty[static_cast<std::size_t>(tile_y * kTileColumns + tile_x)]);

                const int x = run_start * kTileSize;
                const int y = tile_y * kTileSize;
                const int width = std::min(
                    (tile_x - run_start) * kTileSize,
                    board::kLogicalWidth - x);
                const int height = std::min(kTileSize, board::kLogicalHeight - y);
                display_.setClipRect(x, y, width, height);
                render_target_->pushSprite(&display_, 0, 0);
            }
        }
        display_.clearClipRect();
        display_.endWrite();
    }

    swapBuffers();
    return result;
}

bool DirtyTileFramebuffer::tileChanged(int tile_x, int tile_y) const {
    const auto* current = static_cast<const std::uint16_t*>(render_target_->getBuffer());
    const auto* previous = static_cast<const std::uint16_t*>(previous_frame_->getBuffer());
    const int x = tile_x * kTileSize;
    const int y = tile_y * kTileSize;
    const int width = std::min(kTileSize, board::kLogicalWidth - x);
    const int height = std::min(kTileSize, board::kLogicalHeight - y);

    for (int row = 0; row < height; ++row) {
        const std::size_t offset =
            static_cast<std::size_t>((y + row) * board::kLogicalWidth + x);
        if (std::memcmp(current + offset, previous + offset,
                        static_cast<std::size_t>(width) * sizeof(std::uint16_t)) != 0) {
            return true;
        }
    }
    return false;
}

void DirtyTileFramebuffer::swapBuffers() {
    std::swap(render_target_, previous_frame_);
}

}  // namespace n203::firmware::display_s3
