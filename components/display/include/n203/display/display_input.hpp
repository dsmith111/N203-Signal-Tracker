#pragma once

#include "n203/display/layout.hpp"

namespace n203::display {

enum class InputType {
    kNone,
    kTouchReleased,
};

struct DisplayInput {
    InputType type = InputType::kNone;
    Point point{};

    [[nodiscard]] static DisplayInput touchReleased(Point point) {
        return DisplayInput{InputType::kTouchReleased, point};
    }
};

}  // namespace n203::display
