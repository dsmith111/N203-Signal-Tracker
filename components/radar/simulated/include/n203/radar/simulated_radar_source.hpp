#pragma once

#include <cstddef>
#include <cstdint>

#include "n203/radar/radar_source.hpp"

namespace n203::radar {

class SimulatedRadarSource final : public RadarSource {
public:
    static constexpr std::uint32_t kFrameIntervalMs = 100U;
    static constexpr std::uint32_t kPhaseDurationMs = 4000U;
    static constexpr std::size_t kPhaseCount = 11U;

    void begin(std::uint32_t now_ms);
    void update(std::uint32_t now_ms);
    bool poll(RadarFrame& frame) override;

    [[nodiscard]] std::size_t phaseIndex() const { return phase_index_; }
    [[nodiscard]] const char* phaseName() const;

private:
    void buildFrame(std::uint32_t now_ms);

    RadarFrame frame_{};
    std::uint32_t phase_started_ms_ = 0;
    std::uint32_t next_frame_ms_ = 0;
    std::size_t phase_index_ = 0;
    bool begun_ = false;
    bool frame_available_ = false;
};

}  // namespace n203::radar
