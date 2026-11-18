#pragma once

#include <cstddef>
#include <cstdint>

namespace n203::display {

class RangeControl {
public:
    [[nodiscard]] static std::uint16_t decrease(std::uint16_t current_range_mm);
    [[nodiscard]] static std::uint16_t increase(std::uint16_t current_range_mm);
    [[nodiscard]] static constexpr std::size_t optionCount() { return 6; }
};

}  // namespace n203::display
