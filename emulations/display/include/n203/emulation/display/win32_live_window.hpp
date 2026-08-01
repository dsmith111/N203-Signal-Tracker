#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <optional>
#include <string>
#include <vector>

#include "n203/display/display_input.hpp"
#include "n203/emulation/display/raster_display_host.hpp"

namespace n203::emulation::display {

class Win32LiveWindow {
public:
    [[nodiscard]] bool open(std::string& error);
    [[nodiscard]] bool running() const { return running_; }
    [[nodiscard]] n203::display::DisplayInput pollInput();
    void present(const RasterDisplayHost& frame);

private:
    static LRESULT CALLBACK windowProcedure(HWND window, UINT message, WPARAM w_param, LPARAM l_param);
    LRESULT handleMessage(UINT message, WPARAM w_param, LPARAM l_param);
    void paint();

    HWND window_ = nullptr;
    bool running_ = false;
    std::optional<n203::display::Point> pending_tap_;
    std::vector<std::uint8_t> bgra_pixels_;
};

}  // namespace n203::emulation::display
