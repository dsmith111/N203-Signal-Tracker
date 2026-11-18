#include "c4001_direct_radar_source.hpp"

#include <cstring>
#include <limits>

#include "board_config.hpp"

namespace n203::firmware::display_s3 {
namespace {

constexpr std::uint32_t kConnectedReportAgeMs = 1500U;
constexpr std::size_t kUartRxBufferSize = 512U;
#if defined(N203_S3_C4001_POSITION)
constexpr std::uint32_t kStopDelayMs = 1200U;
constexpr std::uint32_t kModeDelayMs = 100U;
constexpr std::uint32_t kSaveDelayMs = 600U;
constexpr std::uint32_t kModeVerificationTimeoutMs = 4000U;
constexpr std::uint8_t kMaximumConfigurationRetries = 2U;

bool deadlineReached(std::uint32_t now_ms, std::uint32_t deadline_ms) {
    return static_cast<std::int32_t>(now_ms - deadline_ms) >= 0;
}
#endif

}  // namespace

void C4001DirectRadarSource::begin() {
    stream_ = {};
    filter_.reset();
    range_filter_.reset();
    stats_ = {};
    next_frame_sequence_ = 0U;
    last_valid_report_ms_ = 0U;
    received_valid_report_ = false;
    report_mode_ = ReportMode::kUnknown;

    serial_.setRxBufferSize(kUartRxBufferSize);
#if defined(N203_S3_C4001_POSITION)
    requested_mode_ = ReportMode::kRange;
    configuration_state_ = C4001ConfigurationState::kAwaitingSensor;
    configuration_deadline_ms_ = 0U;
    configuration_retries_ = 0U;
    serial_.begin(board::kC4001DirectBaud, SERIAL_8N1,
                  board::kScannerUartRx, board::kScannerUartTx);
#else
    configuration_state_ = C4001ConfigurationState::kDisabled;
    serial_.begin(board::kC4001DirectBaud, SERIAL_8N1,
                  board::kScannerUartRx, -1);
#endif
}

void C4001DirectRadarSource::recordFrame(
    const radar::RadarFrame& candidate,
    radar::RadarFrame& frame,
    bool& have_frame) {
    if (have_frame) {
        ++stats_.frames_superseded;
    }
    frame = candidate;
    ++stats_.frames_emitted;
    have_frame = true;
}

bool C4001DirectRadarSource::poll(radar::RadarFrame& frame) {
    bool have_frame = false;
    radar::c4001::Report report{};

    while (serial_.available() > 0) {
        const int value = serial_.read();
        if (value < 0) {
            break;
        }

        const auto event = stream_.push(static_cast<std::uint8_t>(value), report);
        if (event != radar::c4001::StreamEvent::kReportReady) {
            continue;
        }

        radar::c4001::RangeMeasurement range{};
        const auto range_error = radar::c4001::decodeRangeReport(report, range);
        if (range_error == radar::c4001::RangeReportError::kNone) {
            ++stats_.valid_range_reports;
            const std::uint32_t now_ms = millis();
            last_valid_report_ms_ = now_ms;
            received_valid_report_ = true;
            report_mode_ = ReportMode::kRange;
            radar::c4001::RangeMeasurement filtered{};
            if (!range_filter_.observe(range, filtered)) {
                continue;
            }
            recordFrame(
                radar::c4001::makeRangeFrame(
                    filtered, next_frame_sequence_++, now_ms),
                frame, have_frame);
            continue;
        }
        if (range_error == radar::c4001::RangeReportError::kOutOfRange) {
            ++stats_.out_of_range_reports;
            continue;
        }
        if (range_error == radar::c4001::RangeReportError::kMalformedReport) {
            ++stats_.malformed_reports;
            continue;
        }

        radar::PresenceState observed = radar::PresenceState::kUnknown;
        const auto presence_error =
            radar::c4001::decodePresenceReport(report, observed);
        if (presence_error == radar::c4001::PresenceReportError::kUnsupportedReport) {
            ++stats_.unsupported_reports;
            continue;
        }
        if (presence_error != radar::c4001::PresenceReportError::kNone) {
            ++stats_.malformed_reports;
            continue;
        }

        ++stats_.valid_presence_reports;
        const std::uint32_t now_ms = millis();
        last_valid_report_ms_ = now_ms;
        received_valid_report_ = true;
        report_mode_ = ReportMode::kPresence;

        radar::PresenceState filtered = radar::PresenceState::kUnknown;
        if (!filter_.observe(observed, filtered)) {
            continue;
        }

        recordFrame(
            radar::c4001::makePresenceFrame(
                filtered, next_frame_sequence_++, now_ms),
            frame, have_frame);
    }

    const auto& stream_stats = stream_.stats();
    stats_.bytes_received = stream_stats.bytes;
    stats_.reports_received = stream_stats.reports;
    stats_.overflow_rejections = stream_stats.overflow_rejections;
    stats_.interrupted_reports = stream_stats.interrupted_reports;
#if defined(N203_S3_C4001_POSITION)
    advanceConfiguration(millis());
#endif
    return have_frame;
}

bool C4001DirectRadarSource::connected(std::uint32_t now_ms) const {
    return received_valid_report_ && now_ms - last_valid_report_ms_ <= kConnectedReportAgeMs;
}

std::uint32_t C4001DirectRadarSource::lastReportAgeMs(std::uint32_t now_ms) const {
    if (!received_valid_report_) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    return now_ms - last_valid_report_ms_;
}

const char* C4001DirectRadarSource::configurationStateName() const {
    switch (configuration_state_) {
        case C4001ConfigurationState::kDisabled: return "disabled";
        case C4001ConfigurationState::kAwaitingSensor: return "awaiting-sensor";
        case C4001ConfigurationState::kWaitingAfterStop: return "stopping";
        case C4001ConfigurationState::kWaitingAfterMode: return "setting-mode";
        case C4001ConfigurationState::kWaitingAfterSave: return "saving";
        case C4001ConfigurationState::kAwaitingRequestedMode: return "verifying";
        case C4001ConfigurationState::kComplete: return "complete";
        case C4001ConfigurationState::kFailed: return "failed";
    }
    return "invalid";
}

const char* C4001DirectRadarSource::reportModeName() const {
    switch (report_mode_) {
        case ReportMode::kUnknown: return "unknown";
        case ReportMode::kPresence: return "presence";
        case ReportMode::kRange: return "range";
    }
    return "invalid";
}

#if defined(N203_S3_C4001_POSITION)
void C4001DirectRadarSource::requestRangeMode() {
    requestMode(ReportMode::kRange);
}

void C4001DirectRadarSource::requestPresenceMode() {
    requestMode(ReportMode::kPresence);
}

void C4001DirectRadarSource::requestMode(ReportMode mode) {
    requested_mode_ = mode;
    configuration_retries_ = 0U;
    configuration_state_ = C4001ConfigurationState::kAwaitingSensor;
}

void C4001DirectRadarSource::sendCommand(const char* command) {
    serial_.write(reinterpret_cast<const std::uint8_t*>(command), std::strlen(command));
    ++stats_.configuration_commands;
}

void C4001DirectRadarSource::startConfiguration(std::uint32_t now_ms) {
    sendCommand("sensorStop");
    configuration_deadline_ms_ = now_ms + kStopDelayMs;
    configuration_state_ = C4001ConfigurationState::kWaitingAfterStop;
}

void C4001DirectRadarSource::advanceConfiguration(std::uint32_t now_ms) {
    switch (configuration_state_) {
        case C4001ConfigurationState::kAwaitingSensor:
            if (report_mode_ == ReportMode::kUnknown) {
                return;
            }
            if (report_mode_ == requested_mode_) {
                configuration_state_ = C4001ConfigurationState::kComplete;
                return;
            }
            startConfiguration(now_ms);
            return;

        case C4001ConfigurationState::kWaitingAfterStop:
            if (!deadlineReached(now_ms, configuration_deadline_ms_)) {
                return;
            }
            sendCommand(requested_mode_ == ReportMode::kRange
                            ? "setRunApp 1"
                            : "setRunApp 0");
            configuration_deadline_ms_ = now_ms + kModeDelayMs;
            configuration_state_ = C4001ConfigurationState::kWaitingAfterMode;
            return;

        case C4001ConfigurationState::kWaitingAfterMode:
            if (!deadlineReached(now_ms, configuration_deadline_ms_)) {
                return;
            }
            sendCommand("saveConfig");
            configuration_deadline_ms_ = now_ms + kSaveDelayMs;
            configuration_state_ = C4001ConfigurationState::kWaitingAfterSave;
            return;

        case C4001ConfigurationState::kWaitingAfterSave:
            if (!deadlineReached(now_ms, configuration_deadline_ms_)) {
                return;
            }
            report_mode_ = ReportMode::kUnknown;
            sendCommand("sensorStart");
            configuration_deadline_ms_ = now_ms + kModeVerificationTimeoutMs;
            configuration_state_ = C4001ConfigurationState::kAwaitingRequestedMode;
            return;

        case C4001ConfigurationState::kAwaitingRequestedMode:
            if (report_mode_ == requested_mode_) {
                configuration_state_ = C4001ConfigurationState::kComplete;
                return;
            }
            if (!deadlineReached(now_ms, configuration_deadline_ms_)) {
                return;
            }
            if (configuration_retries_ >= kMaximumConfigurationRetries) {
                configuration_state_ = C4001ConfigurationState::kFailed;
                return;
            }
            ++configuration_retries_;
            startConfiguration(now_ms);
            return;

        case C4001ConfigurationState::kDisabled:
        case C4001ConfigurationState::kComplete:
        case C4001ConfigurationState::kFailed:
            return;
    }
}
#endif

}  // namespace n203::firmware::display_s3
