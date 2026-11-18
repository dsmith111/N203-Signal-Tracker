#include "n203/display/widgets/range_control.hpp"

#include <array>

namespace n203::display {
namespace {
constexpr std::array<std::uint16_t, RangeControl::optionCount()> kRangeOptions{{
    2000, 5000, 10000, 15000, 20000, 25000,
}};
}

std::uint16_t RangeControl::decrease(std::uint16_t current_range_mm) {
    for (std::size_t index = 0; index < kRangeOptions.size(); ++index) {
        if (kRangeOptions[index] >= current_range_mm) {
            return index == 0 ? kRangeOptions.front() : kRangeOptions[index - 1];
        }
    }
    return kRangeOptions.back();
}

std::uint16_t RangeControl::increase(std::uint16_t current_range_mm) {
    for (std::size_t index = 0; index < kRangeOptions.size(); ++index) {
        if (kRangeOptions[index] > current_range_mm) {
            return kRangeOptions[index];
        }
    }
    return kRangeOptions.back();
}

}  // namespace n203::display
