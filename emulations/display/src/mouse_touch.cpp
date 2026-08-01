#include "n203/emulation/display/mouse_touch.hpp"

namespace n203::emulation::display {

void MouseTouchInput::tap(n203::display::Point point) {
    pending_tap_ = point;
}

n203::display::DisplayInput MouseTouchInput::poll() {
    if (!pending_tap_.has_value()) {
        return {};
    }
    const n203::display::Point point = *pending_tap_;
    pending_tap_.reset();
    return n203::display::DisplayInput::touchReleased(point);
}

}  // namespace n203::emulation::display
