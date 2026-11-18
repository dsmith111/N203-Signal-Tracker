#include <cassert>
#include <cstdint>

#include "n203/radar/frame_validation.hpp"
#include "n203/radar/simulated_radar_source.hpp"

void runSimulatedRadarSourceTests() {
    n203::radar::SimulatedRadarSource source;
    n203::radar::RadarFrame frame{};
    source.begin(0U);

    source.update(0U);
    assert(source.poll(frame));
    assert(frame.sequence == 1U);
    assert(n203::radar::validateFrame(frame) == n203::radar::FrameValidationError::kNone);
    source.update(50U);
    assert(!source.poll(frame));
    source.update(100U);
    assert(source.poll(frame));
    assert(frame.sequence == 2U);

    for (std::size_t phase = 1U; phase < n203::radar::SimulatedRadarSource::kPhaseCount; ++phase) {
        source.update(static_cast<std::uint32_t>(
            phase * n203::radar::SimulatedRadarSource::kPhaseDurationMs));
        assert(source.phaseIndex() == phase);
        assert(source.poll(frame));
        assert(n203::radar::validateFrame(frame) == n203::radar::FrameValidationError::kNone);
    }

    source.begin(0U);
    source.update(6U * n203::radar::SimulatedRadarSource::kPhaseDurationMs);
    assert(source.phaseIndex() == 6U);
    assert(source.poll(frame));
    assert(frame.measurement_state == n203::radar::MeasurementState::kPresenceOnly);
    assert(frame.presence == n203::radar::PresenceState::kDetected);
    assert(frame.contact_count == 0U);
}
