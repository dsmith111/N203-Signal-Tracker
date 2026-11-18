#pragma once

#include <Arduino.h>

#include <cstdint>

#include "n203/radar/c4001/presence_pipeline.hpp"
#include "n203/radar/c4001/range_pipeline.hpp"
#include "n203/radar/c4001/report_stream.hpp"
#include "n203/radar/radar_source.hpp"

namespace n203::firmware::display_s3 {

struct C4001DirectStats {
    std::uint32_t bytes_received = 0;
    std::uint32_t reports_received = 0;
    std::uint32_t valid_presence_reports = 0;
    std::uint32_t valid_range_reports = 0;
    std::uint32_t malformed_reports = 0;
    std::uint32_t unsupported_reports = 0;
    std::uint32_t out_of_range_reports = 0;
    std::uint32_t overflow_rejections = 0;
    std::uint32_t interrupted_reports = 0;
    std::uint32_t frames_emitted = 0;
    std::uint32_t frames_superseded = 0;
    std::uint32_t configuration_commands = 0;
};

enum class C4001ConfigurationState : std::uint8_t {
    kDisabled,
    kAwaitingSensor,
    kWaitingAfterStop,
    kWaitingAfterMode,
    kWaitingAfterSave,
    kAwaitingRequestedMode,
    kComplete,
    kFailed,
};

// Temporary single-controller bench adapter. It reads the C4001 directly on
// the S3 scanner UART and publishes the same canonical RadarFrame model used
// by the normal C3-to-S3 link. The base target is read-only; the explicitly
// selected position-setup target can switch the sensor's saved report mode.
class C4001DirectRadarSource final : public radar::RadarSource {
public:
    explicit C4001DirectRadarSource(HardwareSerial& serial) : serial_(serial) {}

    void begin();
    bool poll(radar::RadarFrame& frame) override;

    [[nodiscard]] const C4001DirectStats& stats() const { return stats_; }
    [[nodiscard]] bool connected(std::uint32_t now_ms) const;
    [[nodiscard]] std::uint32_t lastReportAgeMs(std::uint32_t now_ms) const;
    [[nodiscard]] C4001ConfigurationState configurationState() const {
        return configuration_state_;
    }
    [[nodiscard]] const char* configurationStateName() const;
    [[nodiscard]] const char* reportModeName() const;

#if defined(N203_S3_C4001_POSITION)
    void requestRangeMode();
    void requestPresenceMode();
#endif

private:
    enum class ReportMode : std::uint8_t {
        kUnknown,
        kPresence,
        kRange,
    };

    void recordFrame(const radar::RadarFrame& candidate,
                     radar::RadarFrame& frame,
                     bool& have_frame);
#if defined(N203_S3_C4001_POSITION)
    void requestMode(ReportMode mode);
    void startConfiguration(std::uint32_t now_ms);
    void advanceConfiguration(std::uint32_t now_ms);
    void sendCommand(const char* command);
#endif

    HardwareSerial& serial_;
    radar::c4001::ReportStream stream_{};
    radar::c4001::PresenceFilter filter_{};
    radar::c4001::RangeFilter range_filter_{};
    C4001DirectStats stats_{};
    std::uint32_t next_frame_sequence_ = 0;
    std::uint32_t last_valid_report_ms_ = 0;
    bool received_valid_report_ = false;
    ReportMode report_mode_ = ReportMode::kUnknown;
    C4001ConfigurationState configuration_state_ =
        C4001ConfigurationState::kDisabled;
#if defined(N203_S3_C4001_POSITION)
    ReportMode requested_mode_ = ReportMode::kRange;
    std::uint32_t configuration_deadline_ms_ = 0U;
    std::uint8_t configuration_retries_ = 0U;
#endif
};

}  // namespace n203::firmware::display_s3
