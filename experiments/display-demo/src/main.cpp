#include <Arduino.h>
#include <esp_system.h>

#include <cstddef>
#include <cstdint>

#include "board_config.hpp"
#include "demo_radar_source.hpp"
#include "dirty_tile_framebuffer.hpp"
#include "lovyan_gfx_canvas.hpp"
#include "n203/display/display_app.hpp"
#include "n203/display/radar_renderer.hpp"
#include "n203_lgfx_device.hpp"
#include "xpt2046_touch_input.hpp"

namespace {

namespace board = n203::firmware::display_s3::board;
using n203::display::DisplayApp;
using n203::display::InputType;
using n203::display::RadarRenderer;
using n203::experiments::display_demo::DemoRadarSource;
using n203::firmware::display_s3::DirtyTileFramebuffer;
using n203::firmware::display_s3::LovyanGfxCanvas;
using n203::firmware::display_s3::N203LgfxDevice;
using n203::firmware::display_s3::Xpt2046TouchInput;

constexpr std::uint8_t kDemoBrightness = 192U;
constexpr std::uint32_t kRenderIntervalMs = 50U;
constexpr std::uint32_t kFramePixelCount =
    static_cast<std::uint32_t>(board::kLogicalWidth * board::kLogicalHeight);

N203LgfxDevice display;
Xpt2046TouchInput touch(display);
DirtyTileFramebuffer framebuffer(display);
LovyanGfxCanvas canvas(framebuffer.renderTarget());
DisplayApp app;
RadarRenderer renderer;
DemoRadarSource radar;

std::uint32_t last_render_started_ms = 0;
std::uint32_t stats_started_ms = 0;
std::uint64_t compose_total_us = 0;
std::uint64_t transfer_total_us = 0;
std::uint64_t dirty_pixel_total = 0;
std::uint32_t total_worst_us = 0;
std::uint32_t render_count = 0;

void holdSharedDevicesInactive() {
    pinMode(board::kTouchCs, OUTPUT);
    digitalWrite(board::kTouchCs, HIGH);
    pinMode(board::kBacklightControl, OUTPUT);
    digitalWrite(board::kBacklightControl, LOW);
}

void renderFrame(std::uint32_t now_ms) {
    canvas.setTarget(framebuffer.renderTarget());
    const std::uint32_t compose_started_us = micros();
    canvas.beginFrame();
    renderer.render(canvas, app.state(), now_ms);
    canvas.endFrame();
    const std::uint32_t compose_us = micros() - compose_started_us;

    const std::uint32_t transfer_started_us = micros();
    const auto present = framebuffer.present();
    const std::uint32_t transfer_us = micros() - transfer_started_us;
    const std::uint32_t total_us = compose_us + transfer_us;

    if (present.full_refresh) {
        Serial.printf("[display] initial_full_refresh_ms=%lu.%03lu psram_free=%lu\n",
                      static_cast<unsigned long>(transfer_us / 1000U),
                      static_cast<unsigned long>(transfer_us % 1000U),
                      static_cast<unsigned long>(ESP.getFreePsram()));
        stats_started_ms = millis();
        return;
    }

    compose_total_us += compose_us;
    transfer_total_us += transfer_us;
    dirty_pixel_total += present.dirty_pixels;
    if (total_us > total_worst_us) {
        total_worst_us = total_us;
    }
    ++render_count;
}

void printStats(std::uint32_t now_ms) {
    const std::uint32_t elapsed_ms = now_ms - stats_started_ms;
    if (elapsed_ms < 1000U || render_count == 0U) {
        return;
    }

    const std::uint32_t compose_average_us =
        static_cast<std::uint32_t>(compose_total_us / render_count);
    const std::uint32_t transfer_average_us =
        static_cast<std::uint32_t>(transfer_total_us / render_count);
    const std::uint32_t fps_x10 = render_count * 10000U / elapsed_ms;
    const std::uint32_t dirty_percent_x10 = static_cast<std::uint32_t>(
        dirty_pixel_total * 1000U /
        (static_cast<std::uint64_t>(render_count) * kFramePixelCount));
    Serial.printf(
        "[ui] state=%s fps=%lu.%lu compose_ms=%lu.%03lu transfer_ms=%lu.%03lu "
        "worst_ms=%lu.%03lu dirty=%lu.%lu%% heap_free=%lu psram_free=%lu\n",
        radar.phaseName(),
        static_cast<unsigned long>(fps_x10 / 10U),
        static_cast<unsigned long>(fps_x10 % 10U),
        static_cast<unsigned long>(compose_average_us / 1000U),
        static_cast<unsigned long>(compose_average_us % 1000U),
        static_cast<unsigned long>(transfer_average_us / 1000U),
        static_cast<unsigned long>(transfer_average_us % 1000U),
        static_cast<unsigned long>(total_worst_us / 1000U),
        static_cast<unsigned long>(total_worst_us % 1000U),
        static_cast<unsigned long>(dirty_percent_x10 / 10U),
        static_cast<unsigned long>(dirty_percent_x10 % 10U),
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(ESP.getFreePsram()));

    stats_started_ms = now_ms;
    compose_total_us = 0;
    transfer_total_us = 0;
    dirty_pixel_total = 0;
    total_worst_us = 0;
    render_count = 0;
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
    Serial.println("N203 Signal Tracker - shared UI display demo");
    Serial.printf("[boot] reset_reason=%d flash=%lu psram=%s psram_size=%lu\n",
                  static_cast<int>(esp_reset_reason()),
                  static_cast<unsigned long>(ESP.getFlashChipSize()),
                  psramFound() ? "yes" : "no",
                  static_cast<unsigned long>(ESP.getPsramSize()));

    display.init();
    display.setRotation(board::kLandscapeRotation);
    display.setColorDepth(16);
    touch.begin(millis());
    display.setBrightness(0);

    if (display.width() != board::kLogicalWidth || display.height() != board::kLogicalHeight) {
        Serial.printf("[fatal] display=%dx%d expected=%dx%d; backlight remains off\n",
                      display.width(), display.height(), board::kLogicalWidth, board::kLogicalHeight);
        while (true) {
            delay(1000);
        }
    }

    if (!psramFound() || !framebuffer.begin()) {
        Serial.println("[fatal] unable to allocate two RGB565 framebuffers in PSRAM; backlight remains off");
        while (true) {
            delay(1000);
        }
    }

    display.fillScreen(N203LgfxDevice::color888(2, 16, 11));
    display.setBrightness(kDemoBrightness);

    const std::uint32_t now_ms = millis();
    radar.begin(now_ms);
    stats_started_ms = now_ms;
    last_render_started_ms = now_ms - kRenderIntervalMs;
    Serial.println("[display] shared renderer active; calibrated touch enabled; canned states cycle every 4.5s");
}

void loop() {
    const std::uint32_t now_ms = millis();
    radar.update(now_ms);

    n203::radar::RadarFrame frame{};
    if (radar.poll(frame)) {
        app.acceptRadarFrame(frame, now_ms);
    }

    const auto input = touch.poll(now_ms);
    if (input.type == InputType::kTouchReleased) {
        app.acceptInput(input);
        const auto& state = app.state();
        Serial.printf(
            "[input] release_x=%d release_y=%d range_mm=%u selected=%s contact_id=%u locked=%s\n",
            input.point.x, input.point.y, state.display_range_mm,
            state.has_selected_contact ? "yes" : "no", state.selected_contact_id,
            state.target_locked ? "yes" : "no");
    }
    app.update(now_ms);

    if (now_ms - last_render_started_ms >= kRenderIntervalMs) {
        last_render_started_ms = now_ms;
        renderFrame(now_ms);
    }

    printStats(millis());
    delay(1);
}
