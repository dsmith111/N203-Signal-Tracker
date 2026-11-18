#include <Arduino.h>
#include <esp_system.h>

#include <algorithm>
#include <array>
#include <cstdint>

#include "board_config.hpp"
#include "n203_lgfx_device.hpp"

namespace {

namespace board = n203::firmware::display_s3::board;
using n203::firmware::display_s3::N203LgfxDevice;

constexpr std::uint8_t kBenchBrightness = 192U;
constexpr std::uint32_t kSampleDrawIntervalMs = 80U;
constexpr std::uint32_t kHeldLogIntervalMs = 250U;

constexpr int kPlotX = 24;
constexpr int kPlotY = 174;
constexpr int kPlotWidth = 432;
constexpr int kPlotHeight = 122;

N203LgfxDevice display;
std::array<std::uint16_t, 8> calibration{};

bool calibrated = false;
bool touching = false;
std::uint32_t touch_started_ms = 0;
std::uint32_t last_draw_ms = 0;
std::uint32_t last_held_log_ms = 0;
std::uint32_t sample_count = 0;
lgfx::touch_point_t last_raw{};

std::uint32_t backgroundColor() {
    return display.color888(2, 16, 11);
}

std::uint32_t panelColor() {
    return display.color888(4, 31, 20);
}

std::uint32_t greenColor() {
    return display.color888(92, 255, 143);
}

std::uint32_t dimGreenColor() {
    return display.color888(35, 130, 74);
}

std::uint32_t amberColor() {
    return display.color888(255, 184, 52);
}

void drawDiagnosticPage() {
    const auto background = backgroundColor();
    const auto panel = panelColor();
    const auto green = greenColor();
    const auto dim_green = dimGreenColor();
    const auto amber = amberColor();

    display.fillScreen(background);
    display.drawRect(5, 5, display.width() - 10, display.height() - 10, dim_green);
    display.drawFastHLine(12, 42, display.width() - 24, dim_green);

    display.setTextColor(green, background);
    display.setTextSize(2);
    display.setCursor(18, 17);
    display.print("N203 XPT2046 TOUCH BENCH");

    display.setTextSize(1);
    display.setTextColor(amber, background);
    display.setCursor(18, 55);
    display.print(calibrated ? "MODE: CALIBRATED" : "MODE: RAW / UNCALIBRATED");
    display.setTextColor(green, background);
    display.setCursor(18, 71);
    display.print("Touch the panel to inspect samples.");
    display.setCursor(18, 87);
    display.print("Type c in the serial monitor for guided calibration.");

    display.fillRect(18, 108, display.width() - 36, 48, panel);
    display.drawRect(18, 108, display.width() - 36, 48, dim_green);
    display.fillRect(kPlotX, kPlotY, kPlotWidth, kPlotHeight, panel);
    display.drawRect(kPlotX, kPlotY, kPlotWidth, kPlotHeight, dim_green);
    display.setTextColor(dim_green, panel);
    display.setCursor(kPlotX + 8, kPlotY + 8);
    display.print(calibrated ? "MAPPED POSITION PREVIEW" : "MAPPED PREVIEW AVAILABLE AFTER CALIBRATION");
}

void drawSample(const lgfx::touch_point_t* raw) {
    const auto panel = panelColor();
    const auto green = greenColor();
    const auto dim_green = dimGreenColor();
    const auto amber = amberColor();

    display.fillRect(26, 116, display.width() - 52, 31, panel);
    display.setTextSize(1);
    display.setTextColor(raw ? green : dim_green, panel);
    display.setCursor(28, 119);
    if (raw) {
        display.printf("RAW X:%4d  Y:%4d  PRESSURE:%3u  IRQ:%s",
                       raw->x, raw->y, raw->size,
                       digitalRead(board::kTouchIrq) == LOW ? "LOW" : "HIGH");
    } else {
        display.printf("NO TOUCH                         IRQ:%s",
                       digitalRead(board::kTouchIrq) == LOW ? "LOW" : "HIGH");
    }

    display.fillRect(kPlotX + 1, kPlotY + 23, kPlotWidth - 2, kPlotHeight - 24, panel);
    if (!raw || !calibrated) {
        return;
    }

    auto mapped = *raw;
    display.convertRawXY(&mapped, 1);
    const int mapped_x = std::clamp<int>(mapped.x, 0, display.width() - 1);
    const int mapped_y = std::clamp<int>(mapped.y, 0, display.height() - 1);
    const int plot_x = kPlotX + 1 + mapped_x * (kPlotWidth - 3) / (display.width() - 1);
    const int plot_y = kPlotY + 24 + mapped_y * (kPlotHeight - 26) / (display.height() - 1);

    display.drawFastHLine(plot_x - 7, plot_y, 15, amber);
    display.drawFastVLine(plot_x, plot_y - 7, 15, amber);
    display.setTextColor(green, panel);
    display.setCursor(kPlotX + 8, kPlotY + kPlotHeight - 16);
    display.printf("MAPPED X:%3d  Y:%3d", mapped.x, mapped.y);
}

void printCalibration() {
    Serial.print("[calibration] values={");
    for (std::size_t i = 0; i < calibration.size(); ++i) {
        if (i != 0) {
            Serial.print(", ");
        }
        Serial.print(calibration[i]);
    }
    Serial.println("}");

    Serial.print("[calibration] cpp=constexpr std::array<std::uint16_t, 8> kTouchCalibration{{");
    for (std::size_t i = 0; i < calibration.size(); ++i) {
        if (i != 0) {
            Serial.print(", ");
        }
        Serial.print(calibration[i]);
    }
    Serial.println("}};");
}

void runCalibration() {
    const auto background = display.color888(0, 0, 0);
    const auto foreground = greenColor();

    touching = false;
    Serial.println("[calibration] starting; tap each corner marker and lift between targets");
    display.fillScreen(background);
    display.setTextColor(foreground, background);
    display.setTextSize(2);
    display.setCursor(84, 126);
    display.print("TOUCH EACH CORNER TARGET");
    display.setTextSize(1);
    display.setCursor(134, 154);
    display.print("Press the marker tip, then release");
    delay(1000);

    display.calibrateTouch(calibration.data(), foreground, background, 22);
    calibrated = true;
    printCalibration();
    drawDiagnosticPage();
    drawSample(nullptr);
    Serial.println("[calibration] complete; tap corners and center to verify mapped coordinates");
}

void processSerialCommands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'c' || command == 'C') {
            runCalibration();
        }
    }
}

void pollTouch(std::uint32_t now_ms) {
    lgfx::touch_point_t raw{};
    const bool sample_available = display.getTouchRaw(&raw, 1) != 0;
    if (sample_available) {
        last_raw = raw;
        ++sample_count;
        if (!touching) {
            touching = true;
            touch_started_ms = now_ms;
            last_held_log_ms = now_ms;
            sample_count = 1;
            auto mapped = raw;
            if (calibrated) {
                display.convertRawXY(&mapped, 1);
                Serial.printf(
                    "[touch] event=down raw_x=%d raw_y=%d mapped_x=%d mapped_y=%d pressure=%u irq=%d\n",
                    raw.x, raw.y, mapped.x, mapped.y, raw.size, digitalRead(board::kTouchIrq));
            } else {
                Serial.printf("[touch] event=down raw_x=%d raw_y=%d pressure=%u irq=%d\n",
                              raw.x, raw.y, raw.size, digitalRead(board::kTouchIrq));
            }
        } else if (now_ms - last_held_log_ms >= kHeldLogIntervalMs) {
            auto mapped = raw;
            if (calibrated) {
                display.convertRawXY(&mapped, 1);
                Serial.printf(
                    "[touch] event=held raw_x=%d raw_y=%d mapped_x=%d mapped_y=%d pressure=%u samples=%lu\n",
                    raw.x, raw.y, mapped.x, mapped.y, raw.size,
                    static_cast<unsigned long>(sample_count));
            } else {
                Serial.printf("[touch] event=held raw_x=%d raw_y=%d pressure=%u samples=%lu\n",
                              raw.x, raw.y, raw.size, static_cast<unsigned long>(sample_count));
            }
            last_held_log_ms = now_ms;
        }
    } else if (touching) {
        touching = false;
        auto mapped = last_raw;
        if (calibrated) {
            display.convertRawXY(&mapped, 1);
            Serial.printf(
                "[touch] event=up raw_x=%d raw_y=%d mapped_x=%d mapped_y=%d pressure=%u duration_ms=%lu samples=%lu\n",
                last_raw.x, last_raw.y, mapped.x, mapped.y, last_raw.size,
                static_cast<unsigned long>(now_ms - touch_started_ms),
                static_cast<unsigned long>(sample_count));
        } else {
            Serial.printf(
                "[touch] event=up raw_x=%d raw_y=%d pressure=%u duration_ms=%lu samples=%lu\n",
                last_raw.x, last_raw.y, last_raw.size,
                static_cast<unsigned long>(now_ms - touch_started_ms),
                static_cast<unsigned long>(sample_count));
        }
    }

    if (now_ms - last_draw_ms >= kSampleDrawIntervalMs) {
        last_draw_ms = now_ms;
        drawSample(sample_available ? &raw : nullptr);
    }
}

}  // namespace

void setup() {
    pinMode(board::kTouchCs, OUTPUT);
    digitalWrite(board::kTouchCs, HIGH);
    pinMode(board::kBacklightControl, OUTPUT);
    digitalWrite(board::kBacklightControl, LOW);

    Serial.begin(115200);
    const std::uint32_t serial_wait_started = millis();
    while (!Serial && millis() - serial_wait_started < 1500U) {
        delay(10);
    }

    Serial.println();
    Serial.println("N203 Signal Tracker - XPT2046 touch bench");
    Serial.printf("[boot] reset_reason=%d sclk=%d mosi=%d miso=%d touch_cs=%d touch_irq=%d\n",
                  static_cast<int>(esp_reset_reason()), board::kSpiSclk, board::kSpiMosi,
                  board::kSpiMiso, board::kTouchCs, board::kTouchIrq);

    display.init();
    display.setRotation(board::kLandscapeRotation);
    display.setColorDepth(16);
    display.setBrightness(0);
    display.fillScreen(backgroundColor());
    display.setBrightness(kBenchBrightness);

    drawDiagnosticPage();
    drawSample(nullptr);
    Serial.printf("[touch] controller=%s irq_idle=%d frequency=%luHz\n",
                  display.touch() ? "configured" : "missing",
                  digitalRead(board::kTouchIrq),
                  static_cast<unsigned long>(board::kTouchFrequencyHz));
    Serial.println("[touch] tap anywhere for raw samples; type c to calibrate");
}

void loop() {
    processSerialCommands();
    pollTouch(millis());
    delay(5);
}
