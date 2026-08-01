#pragma once

#include <cstdint>

#include "n203/display/canvas.hpp"
#include "n203/display/display_state.hpp"

namespace n203::display {

class RadarRenderer {
public:
    void render(DisplayCanvas& graphics, const DisplayState& state, std::uint32_t now_ms) const;
};

}  // namespace n203::display
