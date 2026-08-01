#pragma once

#include <optional>

#include "n203/display/display_input.hpp"

namespace n203::emulation::display {

class MouseTouchInput {
public:
    // The headless native host uses this to turn a command-line click into the
    // same logical touch event an SDL mouse adapter will emit.
    void tap(n203::display::Point point);
    [[nodiscard]] n203::display::DisplayInput poll();

private:
    std::optional<n203::display::Point> pending_tap_;
};

}  // namespace n203::emulation::display
