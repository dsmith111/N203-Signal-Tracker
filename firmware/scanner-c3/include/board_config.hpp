#pragma once

#include <cstdint>

namespace n203::firmware::scanner_c3::board {

// Supplied 16-pin board layout, USB-C connector at the top:
// left  5, 6, 7, 8, 9, 10, 20, 21
// right 5V, GND, 3V3, 4, 3, 2, 1, 0
// This matches the common ESP32-C3 SuperMini layout.
inline constexpr const char* kBoardName = "ESP32-C3 SuperMini 16-pin";

struct C3BoardPins {
    int radar_rx_candidate_a;
    int radar_aux_input;
    int radar_tx;
    int link_rx;
    int link_tx;
    int imu_sda;
    int imu_scl;
};

inline constexpr C3BoardPins kPins{
    4,   // RD-03D TX -> C3 radar RX during read-only capture.
    5,   // C4001 OUT input; spare for RD-03D.
    6,   // C3 radar TX -> RD-03D RX only after the read-only gate passes.
    20,  // S3 GPIO42 TX -> C3 link RX.
    21,  // C3 link TX -> S3 GPIO41 RX.
    0,   // MPU-6050 SDA.
    1,   // MPU-6050 SCL.
};

// Use UART1 for the inter-board link so UART0/ROM-console behavior cannot
// interfere with the negotiated receive path. GPIO routing is independent of
// the controller's default pins on ESP32-C3.
inline constexpr int kLinkUartNumber = 1;
inline constexpr int kRadarUartNumber = 0;
inline constexpr std::uint32_t kUsbDiagnosticBaud = 115'200U;
inline constexpr std::uint32_t kLinkUartBaud = 115'200U;
inline constexpr std::uint32_t kRadarInitialBaud = 115'200U;
inline constexpr std::uint32_t kRd03dUartBaud = 256'000U;
inline constexpr std::uint32_t kC4001UartBaud = 9'600U;
inline constexpr std::uint32_t kI2cFrequencyHz = 100'000U;

// GPIO2, GPIO8, and GPIO9 are ESP32-C3 boot-strapping pins. GPIO9 is also the
// supplied board's BOOT button, and GPIO8 is commonly the onboard LED. None is
// assigned to the scanner buses.

}  // namespace n203::firmware::scanner_c3::board
