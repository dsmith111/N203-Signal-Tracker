#include <Arduino.h>
#include <esp_system.h>

#include <algorithm>
#include <cstdint>

#include "board_config.hpp"
#include "dirty_tile_framebuffer.hpp"
#include "lovyan_gfx_canvas.hpp"
#include "n203/display/display_app.hpp"
#include "n203/display/radar_renderer.hpp"
#include "n203_lgfx_device.hpp"
#if defined(N203_S3_DIRECT_C4001)
#include "c4001_direct_radar_source.hpp"
#else
#include "uart_radar_source.hpp"
#endif
#include "xpt2046_touch_input.hpp"

namespace {

namespace board = n203::firmware::display_s3::board;
using n203::display::DisplayApp;
using n203::display::InputType;
using n203::display::RadarRenderer;
using n203::display::ScannerPresentation;
using n203::firmware::display_s3::DirtyTileFramebuffer;
using n203::firmware::display_s3::LovyanGfxCanvas;
using n203::firmware::display_s3::N203LgfxDevice;
#if defined(N203_S3_DIRECT_C4001)
using RadarSource = n203::firmware::display_s3::C4001DirectRadarSource;
#else
using RadarSource = n203::firmware::display_s3::UartRadarSource;
#endif
using n203::firmware::display_s3::Xpt2046TouchInput;

constexpr std::uint8_t kDisplayBrightness = 192U;
constexpr std::uint32_t kRenderIntervalMs = 50U;
constexpr std::uint32_t kFramePixelCount =
    static_cast<std::uint32_t>(board::kLogicalWidth * board::kLogicalHeight);

N203LgfxDevice display;
Xpt2046TouchInput touch(display);
DirtyTileFramebuffer framebuffer(display);
LovyanGfxCanvas canvas(framebuffer.renderTarget());
DisplayApp app;
RadarRenderer renderer;
HardwareSerial scanner_uart(1);
RadarSource radar(scanner_uart);

std::uint32_t last_render_started_ms = 0;
std::uint32_t stats_started_ms = 0;
std::uint64_t compose_total_us = 0;
std::uint64_t transfer_total_us = 0;
std::uint64_t dirty_pixel_total = 0;
std::uint32_t total_worst_us = 0;
std::uint32_t render_count = 0;
#if !defined(N203_S3_DIRECT_C4001)
std::uint32_t printed_descriptor_updates = 0U;
#endif

const char* stateName(ScannerPresentation state) {
    switch (state) {
        case ScannerPresentation::kStarting: return "starting";
        case ScannerPresentation::kScanning: return "scanning";
        case ScannerPresentation::kRangeOnly: return "range-only";
        case ScannerPresentation::kPresenceOnly: return "presence-only";
        case ScannerPresentation::kStale: return "stale";
        case ScannerPresentation::kOffline: return "offline";
        case ScannerPresentation::kInvalidFrame: return "invalid-frame";
    }
    return "unknown";
}

const char* presenceName(n203::radar::PresenceState state) {
    switch (state) {
        case n203::radar::PresenceState::kUnknown: return "unknown";
        case n203::radar::PresenceState::kClear: return "clear";
        case n203::radar::PresenceState::kDetected: return "detected";
    }
    return "invalid";
}

const char* motionName(n203::radar::MotionState state) {
    switch (state) {
        case n203::radar::MotionState::kUnknown: return "unknown";
        case n203::radar::MotionState::kTracking: return "tracking";
        case n203::radar::MotionState::kDopplerWarning: return "doppler-warning";
        case n203::radar::MotionState::kStabilizing: return "stabilizing";
    }
    return "invalid";
}

void holdSharedDevicesInactive() {
    pinMode(board::kTouchCs, OUTPUT);
    digitalWrite(board::kTouchCs, HIGH);
    pinMode(board::kBacklightControl, OUTPUT);
    digitalWrite(board::kBacklightControl, LOW);
}

void renderFrame() {
    canvas.setTarget(framebuffer.renderTarget());
    const std::uint32_t compose_started_us = micros();
    canvas.beginFrame();
    renderer.render(canvas, app.state(), millis());
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
    total_worst_us = std::max(total_worst_us, total_us);
    ++render_count;
}

void printStats(std::uint32_t now_ms) {
    const std::uint32_t elapsed_ms = now_ms - stats_started_ms;
    if (elapsed_ms < 1000U || render_count == 0U) {
        return;
    }

    const auto& source_stats = radar.stats();
#if !defined(N203_S3_DIRECT_C4001)
    const auto& link = source_stats;
    if (link.descriptor_updates != printed_descriptor_updates && radar.hasDescriptor()) {
        const auto& scanner = radar.descriptor();
        Serial.printf(
            "[scanner] session=%08lX firmware=%.16s radar=%.16s caps=%08lX "
            "max_contacts=%u max_range_mm=%lu fov_cdeg=%u\n",
            static_cast<unsigned long>(scanner.session_id), scanner.scanner_firmware.data(),
            scanner.radar_type.data(), static_cast<unsigned long>(scanner.capabilities_flags),
            static_cast<unsigned int>(scanner.maximum_contacts),
            static_cast<unsigned long>(scanner.maximum_range_mm),
            static_cast<unsigned int>(scanner.horizontal_fov_cdeg));
        printed_descriptor_updates = link.descriptor_updates;
    }
#endif
    const std::uint32_t fps_x10 = render_count * 10000U / elapsed_ms;
    const std::uint32_t compose_average_us =
        static_cast<std::uint32_t>(compose_total_us / render_count);
    const std::uint32_t transfer_average_us =
        static_cast<std::uint32_t>(transfer_total_us / render_count);
    const std::uint32_t dirty_percent_x10 = static_cast<std::uint32_t>(
        dirty_pixel_total * 1000U /
        (static_cast<std::uint64_t>(render_count) * kFramePixelCount));

    Serial.printf(
        "[runtime] state=%s fps=%lu.%lu compose_ms=%lu.%03lu transfer_ms=%lu.%03lu "
        "worst_ms=%lu.%03lu dirty=%lu.%lu%% heap=%lu psram=%lu\n",
        stateName(app.state().scanner),
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
    const auto& radar_frame = app.state().frame;
#if defined(N203_S3_DIRECT_C4001)
    Serial.printf(
        "[sensor] mode=c4001-direct report_mode=%s config=%s connected=%s "
        "age_ms=%lu baud=%lu rx=%lu reports=%lu presence_valid=%lu "
        "range_valid=%lu frames=%lu superseded=%lu malformed=%lu "
        "unsupported=%lu out_of_range=%lu overflow=%lu interrupted=%lu "
        "commands=%lu presence=%s contacts=%u range_mm=%u velocity_mm_s=%d seq=%lu\n",
        radar.reportModeName(), radar.configurationStateName(),
        radar.connected(now_ms) ? "yes" : "no",
        static_cast<unsigned long>(radar.lastReportAgeMs(now_ms)),
        static_cast<unsigned long>(board::kC4001DirectBaud),
        static_cast<unsigned long>(source_stats.bytes_received),
        static_cast<unsigned long>(source_stats.reports_received),
        static_cast<unsigned long>(source_stats.valid_presence_reports),
        static_cast<unsigned long>(source_stats.valid_range_reports),
        static_cast<unsigned long>(source_stats.frames_emitted),
        static_cast<unsigned long>(source_stats.frames_superseded),
        static_cast<unsigned long>(source_stats.malformed_reports),
        static_cast<unsigned long>(source_stats.unsupported_reports),
        static_cast<unsigned long>(source_stats.out_of_range_reports),
        static_cast<unsigned long>(source_stats.overflow_rejections),
        static_cast<unsigned long>(source_stats.interrupted_reports),
        static_cast<unsigned long>(source_stats.configuration_commands),
        presenceName(radar_frame.presence),
        static_cast<unsigned int>(radar_frame.contact_count),
        static_cast<unsigned int>(radar_frame.contact_count == 0U
                                      ? 0U
                                      : radar_frame.contacts[0].range_mm),
        static_cast<int>(radar_frame.contact_count == 0U
                             ? 0
                             : radar_frame.contacts[0].radial_velocity_mm_s),
        static_cast<unsigned long>(radar_frame.sequence));
#else
    Serial.printf(
        "[link] connected=%s age_ms=%lu rx=%lu tx=%lu packets=%lu frames=%lu "
        "superseded=%lu pkt_gap=%lu frame_gap=%lu dup=%lu/%lu resets=%lu/%lu ack=%lu/%lu\n",
        radar.connected(now_ms) ? "yes" : "no",
        static_cast<unsigned long>(radar.lastPacketAgeMs(now_ms)),
        static_cast<unsigned long>(link.bytes_received),
        static_cast<unsigned long>(link.bytes_transmitted),
        static_cast<unsigned long>(link.packets_received),
        static_cast<unsigned long>(link.frames_received),
        static_cast<unsigned long>(link.frames_superseded),
        static_cast<unsigned long>(link.packet_gaps),
        static_cast<unsigned long>(link.sequence_gaps),
        static_cast<unsigned long>(link.packet_duplicates),
        static_cast<unsigned long>(link.sequence_duplicates),
        static_cast<unsigned long>(link.packet_resets),
        static_cast<unsigned long>(link.sequence_resets),
        static_cast<unsigned long>(link.hello_acks_sent),
        static_cast<unsigned long>(link.hello_acks_dropped));
    Serial.printf(
        "[link] reject_cobs=%lu crc=%lu format=%lu payload=%lu overflow=%lu "
        "session_changes=%lu presence=%s motion=%s quality=%u contacts=%u seq=%lu\n",
        static_cast<unsigned long>(link.cobs_rejections),
        static_cast<unsigned long>(link.crc_rejections),
        static_cast<unsigned long>(link.format_rejections),
        static_cast<unsigned long>(link.payload_rejections),
        static_cast<unsigned long>(link.overflow_rejections),
        static_cast<unsigned long>(link.session_changes),
        presenceName(radar_frame.presence), motionName(radar_frame.motion),
        static_cast<unsigned int>(radar_frame.motion_quality),
        static_cast<unsigned int>(radar_frame.contact_count),
        static_cast<unsigned long>(radar_frame.sequence));
#endif

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
#if defined(N203_S3_C4001_POSITION)
    Serial.println("N203 Signal Tracker - temporary direct C4001 range display");
    Serial.printf("[boot] reset_reason=%d sensor_rx=%d sensor_tx=%d baud=%lu mode=range-setup\n",
                  static_cast<int>(esp_reset_reason()), board::kScannerUartRx,
                  board::kScannerUartTx,
                  static_cast<unsigned long>(board::kC4001DirectBaud));
#elif defined(N203_S3_DIRECT_C4001)
    Serial.println("N203 Signal Tracker - temporary direct C4001 display");
    Serial.printf("[boot] reset_reason=%d sensor_rx=%d baud=%lu mode=read-only\n",
                  static_cast<int>(esp_reset_reason()), board::kScannerUartRx,
                  static_cast<unsigned long>(board::kC4001DirectBaud));
#else
    Serial.println("N203 Signal Tracker - scanner UART display");
    Serial.printf("[boot] reset_reason=%d scanner_rx=%d scanner_tx=%d baud=%lu protocol=v%u\n",
                  static_cast<int>(esp_reset_reason()), board::kScannerUartRx,
                  board::kScannerUartTx,
                  static_cast<unsigned long>(board::kScannerUartBaud),
                  n203::radar::protocol::kProtocolVersion);
#endif

    display.init();
    display.setRotation(board::kLandscapeRotation);
    display.setColorDepth(16);
    touch.begin(millis());
    display.setBrightness(0);

    if (display.width() != board::kLogicalWidth || display.height() != board::kLogicalHeight) {
        Serial.printf("[fatal] display=%dx%d expected=%dx%d; backlight remains off\n",
                      display.width(), display.height(), board::kLogicalWidth,
                      board::kLogicalHeight);
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

    radar.begin();
    display.fillScreen(N203LgfxDevice::color888(2, 16, 11));
    display.setBrightness(kDisplayBrightness);

    const std::uint32_t now_ms = millis();
    stats_started_ms = now_ms;
    last_render_started_ms = now_ms - kRenderIntervalMs;
#if defined(N203_S3_DIRECT_C4001)
#if defined(N203_S3_C4001_POSITION)
    Serial.println("[sensor] position setup active; type p for presence mode or r for range mode");
    Serial.println("[sensor] waiting for C4001 $DFHPD/$DFDMD reports on GPIO41");
#else
    Serial.println("[sensor] waiting for strict C4001 $DFHPD/$DFDMD reports on GPIO41");
#endif
#else
    Serial.println("[link] waiting for N203 v2 Hello and COBS/CRC radar frames");
#endif
}

void loop() {
    const std::uint32_t now_ms = millis();
#if defined(N203_S3_C4001_POSITION)
    while (Serial.available() > 0) {
        const int command = Serial.read();
        if (command == 'r' || command == 'R') {
            radar.requestRangeMode();
            Serial.println("[sensor.config] requested=range");
        } else if (command == 'p' || command == 'P') {
            radar.requestPresenceMode();
            Serial.println("[sensor.config] requested=presence");
        }
    }
#endif
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
        renderFrame();
    }
    printStats(millis());
    delay(1);
}
