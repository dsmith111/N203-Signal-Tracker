#pragma once

#include <cstdint>

#include "n203/display/display_input.hpp"
#include "n203/display/display_state.hpp"
#include "n203/radar/frame_validation.hpp"

namespace n203::display {

class DisplayApp {
public:
    void acceptRadarFrame(const radar::RadarFrame& frame, std::uint32_t received_ms);
    void acceptInput(const DisplayInput& input);
    void update(std::uint32_t now_ms);

    [[nodiscard]] const DisplayState& state() const { return state_; }

private:
    void selectContactAt(Point point);
    void updateScannerPresentation(std::uint32_t now_ms);

    DisplayState state_{};
    std::uint32_t last_frame_received_ms_ = 0;
    bool received_frame_ = false;
    bool invalid_frame_received_ = false;
};

}  // namespace n203::display
