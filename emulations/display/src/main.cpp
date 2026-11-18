#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "n203/display/display_app.hpp"
#include "n203/display/radar_renderer.hpp"
#include "n203/emulation/display/mouse_touch.hpp"
#include "n203/emulation/display/raster_display_host.hpp"
#include "n203/emulation/display/scenario_source.hpp"
#if defined(N203_HAS_WIN32_LIVE)
#include "n203/emulation/display/win32_live_window.hpp"
#endif

namespace {

void printUsage() {
    std::cerr << "Usage:\n"
              << "  n203-display-emulator <scenario.json> --live\n"
              << "  n203-display-emulator <scenario.json> <output.ppm> [--frames N] [--tap X Y]\n";
}

#if defined(N203_HAS_WIN32_LIVE)
int runLive(
    n203::emulation::display::ScenarioRadarSource& radar,
    n203::display::DisplayApp& app,
    n203::emulation::display::MouseTouchInput& scripted_touch,
    const std::string& screenshot_path) {
    n203::emulation::display::Win32LiveWindow window;
    std::string error;
    if (!window.open(error)) {
        std::cerr << error << '\n';
        return 1;
    }

    n203::emulation::display::RasterDisplayHost display;
    n203::display::RadarRenderer renderer;
    const auto started_at = std::chrono::steady_clock::now();
    auto next_frame_at = started_at;
    while (window.running()) {
        const n203::display::DisplayInput live_input = window.pollInput();
        if (!window.running()) {
            break;
        }

        const auto current_time = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - started_at);
        const std::uint32_t now_ms = static_cast<std::uint32_t>(elapsed.count());
        radar.update(now_ms);
        n203::radar::RadarFrame frame{};
        if (radar.poll(frame)) {
            app.acceptRadarFrame(frame, now_ms);
        }
        app.acceptInput(scripted_touch.poll());
        app.acceptInput(live_input);
        app.update(now_ms);
        renderer.render(display, app.state(), now_ms);
        window.present(display);

        next_frame_at += std::chrono::milliseconds(33);
        std::this_thread::sleep_until(next_frame_at);
    }

    if (!screenshot_path.empty() && !display.writePpm(screenshot_path, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    return 0;
}
#endif

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage();
        return 2;
    }

    std::uint32_t frames = 0;
    bool has_tap = false;
    bool live = false;
    std::string output_path;
    n203::display::Point tap{};
    try {
        for (int index = 2; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--live") {
                live = true;
            } else if (argument == "--frames" && index + 1 < argc) {
                frames = static_cast<std::uint32_t>(std::stoul(argv[++index]));
            } else if (argument == "--tap" && index + 2 < argc) {
                tap.x = std::stoi(argv[++index]);
                tap.y = std::stoi(argv[++index]);
                has_tap = true;
            } else if (argument.rfind("--", 0) != 0 && output_path.empty()) {
                output_path = argument;
            } else {
                printUsage();
                return 2;
            }
        }
    } catch (const std::exception&) {
        printUsage();
        return 2;
    }

    if (!live && output_path.empty()) {
        printUsage();
        return 2;
    }

    n203::emulation::display::ScenarioRadarSource radar;
    std::string error;
    if (!radar.loadFromFile(argv[1], error)) {
        std::cerr << error << '\n';
        return 1;
    }

    n203::display::DisplayApp app;
    n203::emulation::display::MouseTouchInput touch;
    if (has_tap) {
        touch.tap(tap);
    }

#if defined(N203_HAS_WIN32_LIVE)
    if (live) {
        return runLive(radar, app, touch, output_path);
    }
#else
    if (live) {
        std::cerr << "Live display windows are not available in this build.\n";
        return 1;
    }
#endif

    for (std::uint32_t frame_index = 0; frame_index <= frames; ++frame_index) {
        const std::uint32_t now_ms = frame_index * 33U;
        radar.update(now_ms);
        n203::radar::RadarFrame frame{};
        if (radar.poll(frame)) {
            app.acceptRadarFrame(frame, now_ms);
        }
        app.update(now_ms);
    }
    app.acceptInput(touch.poll());

    n203::emulation::display::RasterDisplayHost display;
    n203::display::RadarRenderer renderer;
    renderer.render(display, app.state(), frames * 33U);
    if (!display.writePpm(output_path, error)) {
        std::cerr << error << '\n';
        return 1;
    }

    std::cout << "Rendered " << argv[1] << " to " << output_path << " at 480x320.\n";
    return 0;
}
