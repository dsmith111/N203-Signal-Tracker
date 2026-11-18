#pragma once

#include <array>
#include <cstdint>

namespace n203::firmware::display_s3::board {

// Shared LCD/XPT2046 SPI bus.
inline constexpr int kSpiSclk = 12;
inline constexpr int kSpiMosi = 11;
inline constexpr int kSpiMiso = 13;

// ILI9488. LCD SDO is intentionally not connected during write-only bring-up.
inline constexpr int kLcdCs = 10;
inline constexpr int kLcdDc = 9;
inline constexpr int kLcdReset = 14;

// XPT2046 resistive touch controller.
inline constexpr int kTouchCs = 8;
inline constexpr int kTouchIrq = 7;

// Active-high logic/PWM input to the module's onboard S8050 backlight stage.
inline constexpr int kBacklightControl = 6;

// Dedicated scanner link. Native USB CDC remains the diagnostics channel.
inline constexpr int kScannerUartRx = 41;  // C3 TX -> S3 RX.
inline constexpr int kScannerUartTx = 42;  // Reserved for display-to-scanner messages.
inline constexpr std::uint32_t kScannerUartBaud = 115'200U;

// Temporary direct-sensor bench uses the same UART RX pin but bypasses the C3.
inline constexpr std::uint32_t kC4001DirectBaud = 9'600U;

// Start at the ILI9488 datasheet write timing. Faster clocks require bench proof.
inline constexpr std::uint32_t kLcdWriteFrequencyHz = 20'000'000U;
inline constexpr std::uint32_t kLcdReadFrequencyHz = 6'000'000U;
inline constexpr std::uint32_t kTouchFrequencyHz = 1'000'000U;
inline constexpr std::uint32_t kBacklightPwmFrequencyHz = 12'000U;
inline constexpr int kBacklightPwmChannel = 7;

// Touch calibration for landscape rotation 1. These values were captured on
// one MSP3520/XPT2046 unit and resistive panels vary: if taps land off-target,
// run experiments/touch-bench on your panel and paste its
// "[calibration] cpp=" line here.
inline constexpr std::array<std::uint16_t, 8> kTouchCalibration{{
    408, 3843,
    246, 309,
    3664, 3806,
    3680, 349,
}};

inline constexpr int kLandscapeRotation = 1;
inline constexpr int kLogicalWidth = 480;
inline constexpr int kLogicalHeight = 320;

}  // namespace n203::firmware::display_s3::board
