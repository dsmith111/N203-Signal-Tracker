#pragma once

#include <cstdint>
#include <string>

#include "n203/radar/radar_source.hpp"

namespace n203::emulation::display {

class ScenarioRadarSource final : public n203::radar::RadarSource {
public:
    [[nodiscard]] bool loadFromFile(const std::string& path, std::string& error);
    void update(std::uint32_t now_ms);
    bool poll(n203::radar::RadarFrame& frame) override;

private:
    n203::radar::RadarFrame frame_{};
    bool loaded_ = false;
};

}  // namespace n203::emulation::display
