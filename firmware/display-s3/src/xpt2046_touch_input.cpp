#include "xpt2046_touch_input.hpp"

#include <algorithm>

#include "board_config.hpp"

namespace n203::firmware::display_s3 {
namespace {

constexpr std::uint32_t kPollIntervalMs = 10U;
constexpr std::uint32_t kMinimumPressDurationMs = 20U;
constexpr std::uint16_t kMinimumPressSamples = 2U;
constexpr std::uint8_t kReleaseDebounceSamples = 2U;

}  // namespace

void Xpt2046TouchInput::begin(std::uint32_t now_ms) {
    auto calibration = board::kTouchCalibration;
    display_.setTouchCalibrate(calibration.data());
    resetPress();
    last_poll_ms_ = now_ms - kPollIntervalMs;
}

display::DisplayInput Xpt2046TouchInput::poll(std::uint32_t now_ms) {
    if (now_ms - last_poll_ms_ < kPollIntervalMs) {
        return {};
    }
    last_poll_ms_ = now_ms;

    lgfx::touch_point_t raw{};
    if (display_.getTouchRaw(&raw, 1) != 0U) {
        if (!pressed_) {
            resetPress();
            pressed_ = true;
            press_started_ms_ = now_ms;
        }
        release_miss_count_ = 0;
        ++press_sample_count_;
        recordMappedSample(raw);
        return {};
    }

    if (!pressed_ || ++release_miss_count_ < kReleaseDebounceSamples) {
        return {};
    }

    const bool deliberate_press =
        now_ms - press_started_ms_ >= kMinimumPressDurationMs &&
        press_sample_count_ >= kMinimumPressSamples && history_count_ != 0U;
    const display::Point release_point = deliberate_press ? medianReleasePoint() : display::Point{};
    resetPress();
    return deliberate_press ? display::DisplayInput::touchReleased(release_point)
                            : display::DisplayInput{};
}

void Xpt2046TouchInput::recordMappedSample(lgfx::touch_point_t raw) {
    display_.convertRawXY(&raw, 1);
    const auto x = std::clamp<int>(raw.x, 0, board::kLogicalWidth - 1);
    const auto y = std::clamp<int>(raw.y, 0, board::kLogicalHeight - 1);
    x_history_[history_next_] = static_cast<std::int16_t>(x);
    y_history_[history_next_] = static_cast<std::int16_t>(y);
    history_next_ = static_cast<std::uint8_t>((history_next_ + 1U) % kHistorySize);
    history_count_ = std::min<std::uint8_t>(history_count_ + 1U, kHistorySize);
}

display::Point Xpt2046TouchInput::medianReleasePoint() const {
    auto x_values = x_history_;
    auto y_values = y_history_;
    std::sort(x_values.begin(), x_values.begin() + history_count_);
    std::sort(y_values.begin(), y_values.begin() + history_count_);
    const std::size_t median_index = history_count_ / 2U;
    return {x_values[median_index], y_values[median_index]};
}

void Xpt2046TouchInput::resetPress() {
    press_started_ms_ = 0;
    press_sample_count_ = 0;
    history_count_ = 0;
    history_next_ = 0;
    release_miss_count_ = 0;
    pressed_ = false;
}

}  // namespace n203::firmware::display_s3
