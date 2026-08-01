#include "n203/emulation/display/win32_live_window.hpp"

#include <algorithm>

#include <windowsx.h>

#include "n203/display/layout.hpp"

namespace n203::emulation::display {
namespace {

constexpr wchar_t kWindowClassName[] = L"N203SignalTrackerLiveWindow";
constexpr wchar_t kWindowTitle[] = L"N203 Signal Tracker — Display Emulator";
constexpr int kInitialScale = 2;

}  // namespace

bool Win32LiveWindow::open(std::string& error) {
    const HINSTANCE instance = GetModuleHandleW(nullptr);
    WNDCLASSEXW window_class{};
    window_class.cbSize = sizeof(window_class);
    window_class.lpfnWndProc = &Win32LiveWindow::windowProcedure;
    window_class.hInstance = instance;
    window_class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    window_class.lpszClassName = kWindowClassName;

    if (RegisterClassExW(&window_class) == 0 && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
        error = "Unable to register the live display window.";
        return false;
    }

    constexpr DWORD style = WS_OVERLAPPEDWINDOW;
    RECT window_bounds{
        0,
        0,
        n203::display::DisplayLayout::kLogicalWidth * kInitialScale,
        n203::display::DisplayLayout::kLogicalHeight * kInitialScale,
    };
    if (!AdjustWindowRect(&window_bounds, style, FALSE)) {
        error = "Unable to calculate the live display window size.";
        return false;
    }

    window_ = CreateWindowExW(
        0,
        kWindowClassName,
        kWindowTitle,
        style,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        window_bounds.right - window_bounds.left,
        window_bounds.bottom - window_bounds.top,
        nullptr,
        nullptr,
        instance,
        this);
    if (window_ == nullptr) {
        error = "Unable to create the live display window.";
        return false;
    }

    running_ = true;
    ShowWindow(window_, SW_SHOW);
    UpdateWindow(window_);
    return true;
}

n203::display::DisplayInput Win32LiveWindow::pollInput() {
    MSG message{};
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
        if (message.message == WM_QUIT) {
            running_ = false;
            continue;
        }
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }

    if (!pending_tap_.has_value()) {
        return {};
    }
    const n203::display::Point point = *pending_tap_;
    pending_tap_.reset();
    return n203::display::DisplayInput::touchReleased(point);
}

void Win32LiveWindow::present(const RasterDisplayHost& frame) {
    if (window_ == nullptr) {
        return;
    }

    const std::vector<std::uint8_t>& rgb_pixels = frame.pixels();
    const std::size_t pixel_count = rgb_pixels.size() / 3U;
    bgra_pixels_.resize(pixel_count * 4U);
    for (std::size_t index = 0; index < pixel_count; ++index) {
        bgra_pixels_[index * 4U] = rgb_pixels[index * 3U + 2U];
        bgra_pixels_[index * 4U + 1U] = rgb_pixels[index * 3U + 1U];
        bgra_pixels_[index * 4U + 2U] = rgb_pixels[index * 3U];
        bgra_pixels_[index * 4U + 3U] = 0xFFU;
    }

    InvalidateRect(window_, nullptr, FALSE);
    UpdateWindow(window_);
}

LRESULT CALLBACK Win32LiveWindow::windowProcedure(HWND window, UINT message, WPARAM w_param, LPARAM l_param) {
    Win32LiveWindow* host = reinterpret_cast<Win32LiveWindow*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE) {
        const auto* create = reinterpret_cast<const CREATESTRUCTW*>(l_param);
        host = static_cast<Win32LiveWindow*>(create->lpCreateParams);
        host->window_ = window;
        SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(host));
    }
    return host != nullptr ? host->handleMessage(message, w_param, l_param)
                           : DefWindowProcW(window, message, w_param, l_param);
}

LRESULT Win32LiveWindow::handleMessage(UINT message, WPARAM w_param, LPARAM l_param) {
    switch (message) {
        case WM_CLOSE:
            DestroyWindow(window_);
            return 0;
        case WM_DESTROY:
            window_ = nullptr;
            running_ = false;
            PostQuitMessage(0);
            return 0;
        case WM_ERASEBKGND:
            return 1;
        case WM_KEYDOWN:
            if (w_param == VK_ESCAPE) {
                DestroyWindow(window_);
                return 0;
            }
            break;
        case WM_LBUTTONUP: {
            RECT client{};
            GetClientRect(window_, &client);
            const int client_width = std::max(1L, client.right - client.left);
            const int client_height = std::max(1L, client.bottom - client.top);
            const int logical_x = GET_X_LPARAM(l_param) * n203::display::DisplayLayout::kLogicalWidth / client_width;
            const int logical_y = GET_Y_LPARAM(l_param) * n203::display::DisplayLayout::kLogicalHeight / client_height;
            pending_tap_ = n203::display::Point{
                std::clamp(logical_x, 0, n203::display::DisplayLayout::kLogicalWidth - 1),
                std::clamp(logical_y, 0, n203::display::DisplayLayout::kLogicalHeight - 1),
            };
            return 0;
        }
        case WM_PAINT:
            paint();
            return 0;
        default:
            break;
    }
    return DefWindowProcW(window_, message, w_param, l_param);
}

void Win32LiveWindow::paint() {
    PAINTSTRUCT paint_state{};
    HDC device = BeginPaint(window_, &paint_state);
    RECT client{};
    GetClientRect(window_, &client);

    if (!bgra_pixels_.empty()) {
        BITMAPINFO bitmap{};
        bitmap.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bitmap.bmiHeader.biWidth = n203::display::DisplayLayout::kLogicalWidth;
        bitmap.bmiHeader.biHeight = -n203::display::DisplayLayout::kLogicalHeight;
        bitmap.bmiHeader.biPlanes = 1;
        bitmap.bmiHeader.biBitCount = 32;
        bitmap.bmiHeader.biCompression = BI_RGB;
        SetStretchBltMode(device, COLORONCOLOR);
        StretchDIBits(
            device,
            0,
            0,
            client.right - client.left,
            client.bottom - client.top,
            0,
            0,
            n203::display::DisplayLayout::kLogicalWidth,
            n203::display::DisplayLayout::kLogicalHeight,
            bgra_pixels_.data(),
            &bitmap,
            DIB_RGB_COLORS,
            SRCCOPY);
    } else {
        FillRect(device, &client, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));
    }
    EndPaint(window_, &paint_state);
}

}  // namespace n203::emulation::display
