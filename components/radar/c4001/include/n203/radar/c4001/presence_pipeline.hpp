#pragma once

#include <cstdint>

#include "n203/radar/c4001/report_stream.hpp"
#include "n203/radar/radar_frame.hpp"

namespace n203::radar::c4001 {

enum class PresenceReportError : std::uint8_t {
    kNone,
    kUnsupportedReport,
    kMalformedReport,
};

// Accepts only the two exact active-report shapes observed from the SEN0609:
// $DFHPD,0, , , * and $DFHPD,1, , , *.
[[nodiscard]] PresenceReportError decodePresenceReport(
    const Report& report,
    PresenceState& presence);

// Requires two consecutive reports for the initial state and for each state
// transition. Once confirmed, every valid observation produces a refresh so
// the display can apply its normal arrival-time staleness policy.
class PresenceFilter final {
public:
    [[nodiscard]] bool observe(PresenceState observed, PresenceState& filtered);
    void reset();

private:
    PresenceState confirmed_ = PresenceState::kUnknown;
    PresenceState candidate_ = PresenceState::kUnknown;
    std::uint8_t candidate_count_ = 0U;
    bool has_confirmed_ = false;
};

[[nodiscard]] RadarFrame makePresenceFrame(
    PresenceState presence,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms);

}  // namespace n203::radar::c4001
