#include <cassert>

#include "n203/radar/frame_validation.hpp"
#include "n203/radar/rd03d/position_tracker.hpp"

namespace {

namespace rd03d = n203::radar::rd03d;

rd03d::RangeReport oneTarget(
    std::uint8_t slot,
    std::int16_t native_x_mm,
    std::int16_t y_mm,
    std::int16_t radial_velocity_mm_s = 0) {
    rd03d::RangeReport report{};
    report.target_count = 1U;
    auto& target = report.targets[slot];
    target.present = true;
    target.slot = slot;
    target.x_mm = native_x_mm;
    target.y_mm = y_mm;
    target.radial_velocity_mm_s = radial_velocity_mm_s;
    return report;
}

}  // namespace

void runRd03dPositionTrackerTests() {
    rd03d::PositionTracker tracker;

    auto tracked = tracker.update(oneTarget(0U, -200, 2000, -100), 100U);
    assert(tracked.target_count == 1U);
    assert(tracked.observed_count == 1U);
    assert(tracked.targets[0].observed);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].source_slot == 0U);
    assert(tracked.targets[0].x_mm == 200);
    assert(tracked.targets[0].y_mm == 2000);
    assert(tracked.targets[0].radial_velocity_mm_s == -100);
    assert(tracked.targets[0].confidence == 60U);

    // Small changes remain associated and are gently filtered.
    tracked = tracker.update(oneTarget(0U, -300, 2100, 0), 200U);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].x_mm == 240);
    assert(tracked.targets[0].y_mm == 2040);
    assert(tracked.targets[0].radial_velocity_mm_s == -60);

    // A source-slot change near the prior position retains software identity.
    tracked = tracker.update(oneTarget(1U, -320, 2120, 0), 300U);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].source_slot == 1U);
    assert(tracked.targets[0].x_mm == 272);
    assert(tracked.targets[0].y_mm == 2072);

    // Isolated empty reports retain the contact briefly with decayed confidence.
    rd03d::RangeReport clear{};
    tracked = tracker.update(clear, 400U);
    assert(tracked.target_count == 1U);
    assert(tracked.observed_count == 0U);
    assert(!tracked.targets[0].observed);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].confidence < 80U);

    tracked = tracker.update(clear, 601U);
    assert(tracked.target_count == 0U);

    // The retained internal track can still reacquire its identity for 1.2 s.
    tracked = tracker.update(oneTarget(0U, -300, 2100), 1000U);
    assert(tracked.target_count == 1U);
    assert(tracked.targets[0].id == 1U);

    tracked = tracker.update(clear, 2201U);
    assert(tracked.target_count == 0U);
    tracked = tracker.update(oneTarget(0U, -300, 2100), 2202U);
    assert(tracked.targets[0].id == 2U);

    // Regression: prediction time is unsigned, but a negative X velocity must
    // remain signed during projection. The third report previously projected
    // this track millions of millimetres away and allocated a replacement ID.
    tracker.reset();
    tracked = tracker.update(oneTarget(0U, -1200, 2600), 0U);
    assert(tracked.targets[0].id == 1U);
    tracked = tracker.update(oneTarget(0U, -1188, 2600), 100U);
    assert(tracked.targets[0].id == 1U);
    tracked = tracker.update(oneTarget(0U, -1176, 2600), 200U);
    assert(tracked.targets[0].id == 1U);

    tracker.reset();
    rd03d::RangeReport pair{};
    pair.target_count = 2U;
    pair.targets[0] = oneTarget(0U, 1000, 2000).targets[0];
    pair.targets[1] = oneTarget(1U, -1000, 2000).targets[1];
    tracked = tracker.update(pair, 100U);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].x_mm == -1000);
    assert(tracked.targets[1].id == 2U);
    assert(tracked.targets[1].x_mm == 1000);

    // Source slots may swap; association follows nearest canonical position.
    pair.targets[0] = oneTarget(0U, -900, 2000).targets[0];
    pair.targets[1] = oneTarget(1U, 900, 2000).targets[1];
    tracked = tracker.update(pair, 200U);
    assert(tracked.target_count == 2U);
    assert(tracked.targets[0].id == 1U);
    assert(tracked.targets[0].source_slot == 1U);
    assert(tracked.targets[1].id == 2U);
    assert(tracked.targets[1].source_slot == 0U);

    const auto frame = rd03d::makeTrackedPositionFrame(tracked, 42U, 200U);
    assert(frame.capabilities.supports(n203::radar::kTrackIdentity));
    assert(frame.measurement_state == n203::radar::MeasurementState::kScanning);
    assert(frame.contact_count == 2U);
    assert(frame.contacts[0].has(n203::radar::kContactPosition));
    assert(frame.contacts[0].has(n203::radar::kContactConfidence));
    assert(frame.contacts[0].id == 1U);
    assert(frame.contacts[1].id == 2U);
    assert(n203::radar::validateFrame(frame) ==
           n203::radar::FrameValidationError::kNone);
}
