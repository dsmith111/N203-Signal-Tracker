#pragma once

#include <cstdint>

namespace n203::display {

struct Color {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
};

namespace theme {
constexpr Color kBackground{2, 16, 11};
constexpr Color kPanel{3, 24, 15};
constexpr Color kGridDim{11, 50, 36};
constexpr Color kGrid{20, 84, 60};
constexpr Color kPrimary{121, 229, 143};
constexpr Color kPulse{61, 143, 86};
constexpr Color kText{66, 169, 104};
constexpr Color kMutedText{40, 105, 68};
constexpr Color kTargetGhost{15, 65, 39};
constexpr Color kTargetHalo{25, 102, 57};
constexpr Color kTargetBody{105, 205, 116};
constexpr Color kTargetCore{215, 255, 208};
constexpr Color kSelection{184, 255, 173};
constexpr Color kScanline{1, 11, 7};
constexpr Color kNoise{7, 38, 22};
constexpr Color kBezelEdge{9, 45, 29};
constexpr Color kWarning{190, 130, 47};
constexpr Color kAlert{185, 69, 48};
}  // namespace theme

}  // namespace n203::display
