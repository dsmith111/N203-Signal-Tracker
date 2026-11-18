#include <Arduino.h>
#include <esp_system.h>

#include <array>
#include <cstdint>

#include "board_config.hpp"
#include "n203_lgfx_device.hpp"

namespace {

using n203::firmware::display_s3::N203LgfxDevice;
namespace board = n203::firmware::display_s3::board;

N203LgfxDevice display;

struct SolidPattern {
    const char* name;
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
};

constexpr std::array<SolidPattern, 5> kSolidPatterns{{
    {"RED", 255, 0, 0},
    {"GREEN", 0, 255, 0},
    {"BLUE", 0, 0, 255},
    {"WHITE", 255, 255, 255},
    {"BLACK", 0, 0, 0},
}};

constexpr std::uint32_t kPatternDurationMs = 1800U;
constexpr std::uint8_t kBenchBrightness = 192U;

std::size_t pattern_index = 0;
std::uint32_t pattern_started_ms = 0;
std::uint32_t last_heartbeat_ms = 0;
const char* current_pattern_name = "BOOT";

void holdSharedDevicesInactive() {
    pinMode(board::kTouchCs, OUTPUT);
    digitalWrite(board::kTouchCs, HIGH);

    pinMode(board::kBacklightControl, OUTPUT);
    digitalWrite(board::kBacklightControl, LOW);
}

void drawGeometryPattern() {
    current_pattern_name = "GEOMETRY";
    const std::uint32_t started_us = micros();
    const auto black = display.color888(0, 0, 0);
    const auto white = display.color888(255, 255, 255);
    const auto green = display.color888(80, 255, 120);
    const auto amber = display.color888(255, 180, 40);

    display.startWrite();
    display.fillScreen(black);
    display.drawRect(0, 0, display.width(), display.height(), white);
    display.drawRect(4, 4, display.width() - 8, display.height() - 8, green);
    display.drawLine(0, 0, display.width() - 1, display.height() - 1, green);
    display.drawLine(display.width() - 1, 0, 0, display.height() - 1, green);
    display.drawFastVLine(display.width() / 2, 0, display.height(), amber);
    display.drawFastHLine(0, display.height() / 2, display.width(), amber);

    for (int x = 40; x < display.width(); x += 40) {
        display.drawFastVLine(x, 0, (x % 80 == 0) ? 12 : 7, white);
        display.drawFastVLine(x, display.height() - ((x % 80 == 0) ? 12 : 7),
                              (x % 80 == 0) ? 12 : 7, white);
    }
    for (int y = 40; y < display.height(); y += 40) {
        display.drawFastHLine(0, y, (y % 80 == 0) ? 12 : 7, white);
        display.drawFastHLine(display.width() - ((y % 80 == 0) ? 12 : 7), y,
                              (y % 80 == 0) ? 12 : 7, white);
    }

    display.setTextColor(green, black);
    display.setTextSize(2);
    display.setCursor(18, 18);
    display.print("N203 DISPLAY BENCH");
    display.setTextSize(1);
    display.setCursor(18, 44);
    display.printf("%dx%d ROT=%d SPI=%luMHz", display.width(), display.height(),
                   board::kLandscapeRotation,
                   static_cast<unsigned long>(board::kLcdWriteFrequencyHz / 1'000'000U));
    display.setCursor(18, display.height() - 28);
    display.print("ILI9488 / ESP32-S3-N16R8");
    display.endWrite();

    Serial.printf("[pattern] name=%s draw_us=%lu\n", current_pattern_name,
                  static_cast<unsigned long>(micros() - started_us));
}

void drawSolidPattern(const SolidPattern& pattern) {
    current_pattern_name = pattern.name;
    const std::uint32_t started_us = micros();
    display.fillScreen(display.color888(pattern.red, pattern.green, pattern.blue));
    Serial.printf("[pattern] name=%s fill_us=%lu\n", current_pattern_name,
                  static_cast<unsigned long>(micros() - started_us));
}

void showPattern(std::size_t index) {
    if (index < kSolidPatterns.size()) {
        drawSolidPattern(kSolidPatterns[index]);
    } else {
        drawGeometryPattern();
    }
}

void printHeartbeat(std::uint32_t now_ms) {
    Serial.printf(
        "[heartbeat] uptime_ms=%lu pattern=%s heap_free=%lu psram_found=%s psram_size=%lu psram_free=%lu\n",
        static_cast<unsigned long>(now_ms),
        current_pattern_name,
        static_cast<unsigned long>(ESP.getFreeHeap()),
        psramFound() ? "yes" : "no",
        static_cast<unsigned long>(ESP.getPsramSize()),
        static_cast<unsigned long>(ESP.getFreePsram()));
}

}  // namespace

void setup() {
    holdSharedDevicesInactive();

    Serial.begin(115200);
    const std::uint32_t serial_wait_started = millis();
    while (!Serial && millis() - serial_wait_started < 1500U) {
        delay(10);
    }

    Serial.println();
    Serial.println("N203 Signal Tracker - ILI9488 display bench");
    Serial.printf("[boot] reset_reason=%d flash=%lu psram_expected=8388608\n",
                  static_cast<int>(esp_reset_reason()),
                  static_cast<unsigned long>(ESP.getFlashChipSize()));
    Serial.printf("[pins] sclk=%d mosi=%d miso=%d lcd_cs=%d dc=%d rst=%d touch_cs=%d touch_irq=%d led=%d\n",
                  board::kSpiSclk, board::kSpiMosi, board::kSpiMiso, board::kLcdCs,
                  board::kLcdDc, board::kLcdReset, board::kTouchCs, board::kTouchIrq,
                  board::kBacklightControl);

    display.init();
    display.setRotation(board::kLandscapeRotation);
    display.setColorDepth(16);
    display.setBrightness(0);

    Serial.printf("[display] width=%d height=%d expected=%dx%d\n", display.width(), display.height(),
                  board::kLogicalWidth, board::kLogicalHeight);
    if (display.width() != board::kLogicalWidth || display.height() != board::kLogicalHeight) {
        Serial.println("[fatal] unexpected logical display dimensions; backlight remains off");
        while (true) {
            delay(1000);
        }
    }

    display.fillScreen(display.color888(0, 0, 0));
    display.setBrightness(kBenchBrightness);

    pattern_index = 0;
    pattern_started_ms = millis();
    last_heartbeat_ms = pattern_started_ms;
    showPattern(pattern_index);
    printHeartbeat(pattern_started_ms);
}

void loop() {
    const std::uint32_t now_ms = millis();
    if (now_ms - pattern_started_ms >= kPatternDurationMs) {
        pattern_started_ms = now_ms;
        pattern_index = (pattern_index + 1U) % (kSolidPatterns.size() + 1U);
        showPattern(pattern_index);
    }

    if (now_ms - last_heartbeat_ms >= 1000U) {
        last_heartbeat_ms = now_ms;
        printHeartbeat(now_ms);
    }

    delay(5);
}
