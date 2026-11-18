#pragma once

namespace n203::display::font3x5 {

inline constexpr int kGlyphWidth = 3;
inline constexpr int kGlyphHeight = 5;
inline constexpr int kGlyphAdvance = 4;

[[nodiscard]] inline constexpr const char* glyphRows(char glyph) {
    switch (glyph) {
        case 'A': return "010101111101101";
        case 'B': return "110101110101110";
        case 'C': return "111100100100111";
        case 'D': return "110101101101110";
        case 'E': return "111100110100111";
        case 'F': return "111100110100100";
        case 'G': return "111100101101111";
        case 'H': return "101101111101101";
        case 'I': return "111010010010111";
        case 'J': return "001001001101010";
        case 'K': return "101101110101101";
        case 'L': return "100100100100111";
        case 'M': return "101111111101101";
        case 'N': return "101111111111101";
        case 'O': return "111101101101111";
        case 'P': return "110101110100100";
        case 'Q': return "111101101111011";
        case 'R': return "110101110101101";
        case 'S': return "111100111001111";
        case 'T': return "111010010010010";
        case 'U': return "101101101101111";
        case 'V': return "101101101101010";
        case 'W': return "101101111111101";
        case 'X': return "101101010101101";
        case 'Y': return "101101010010010";
        case 'Z': return "111001010100111";
        case '0': return "111101101101111";
        case '1': return "010110010010111";
        case '2': return "110001111100111";
        case '3': return "110001111001110";
        case '4': return "101101111001001";
        case '5': return "111100110001110";
        case '6': return "011100111101111";
        case '7': return "111001010010010";
        case '8': return "111101111101111";
        case '9': return "111101111001110";
        case '+': return "000010111010000";
        case '-': return "000000111000000";
        case ':': return "000010000010000";
        case '.': return "000000000000010";
        case '/': return "001001010100100";
        default: return "000000000000000";
    }
}

}  // namespace n203::display::font3x5
