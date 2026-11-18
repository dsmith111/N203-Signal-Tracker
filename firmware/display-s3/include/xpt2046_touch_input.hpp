#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/display/display_input.hpp"
#include "n203_lgfx_device.hpp"

namespace n203::firmware::display_s3 {

class Xpt2046TouchInput final {
public:
    explicit Xpt2046TouchInput(N203LgfxDevice& display) : display_(display) {}

    void begin(std::uint32_t now_ms);
    [[nodiscard]] display::DisplayInput poll(std::uint32_t now_ms);

private:
    static constexpr std::size_t kHistorySize = 5U;

    void recordMappedSample(lgfx::touch_point_t raw);
    [[nodiscard]] display::Point medianReleasePoint() const;
    void resetPress();

    N203LgfxDevice& display_;
    std::array<std::int16_t, kHistorySize> x_history_{};
    std::array<std::int16_t, kHistorySize> y_history_{};
    std::uint32_t last_poll_ms_ = 0;
    std::uint32_t press_started_ms_ = 0;
    std::uint16_t press_sample_count_ = 0;
    std::uint8_t history_count_ = 0;
    std::uint8_t history_next_ = 0;
    std::uint8_t release_miss_count_ = 0;
    bool pressed_ = false;
};

}  // namespace n203::firmware::display_s3
