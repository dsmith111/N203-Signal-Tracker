#include "n203/emulation/display/scenario_source.hpp"

#include <fstream>
#include <iterator>
#include <regex>

#include "n203/radar/contact.hpp"

namespace n203::emulation::display {
namespace {

std::string readFile(const std::string& path, std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "Unable to open scenario: " + path;
        return {};
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string stringValue(const std::string& document, const std::string& key, const std::string& fallback = {}) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch match;
    return std::regex_search(document, match, pattern) ? match[1].str() : fallback;
}

int integerValue(const std::string& document, const std::string& key, int fallback = 0) {
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
    std::smatch match;
    return std::regex_search(document, match, pattern) ? std::stoi(match[1].str()) : fallback;
}

n203::radar::MeasurementState measurementState(const std::string& value) {
    if (value == "range-only") return n203::radar::MeasurementState::kRangeOnly;
    if (value == "presence-only") return n203::radar::MeasurementState::kPresenceOnly;
    if (value == "initializing") return n203::radar::MeasurementState::kInitializing;
    if (value == "stale") return n203::radar::MeasurementState::kStale;
    if (value == "offline") return n203::radar::MeasurementState::kOffline;
    return n203::radar::MeasurementState::kScanning;
}

n203::radar::RadarHealth radarHealth(const std::string& value) {
    if (value == "offline") return n203::radar::RadarHealth::kOffline;
    if (value == "degraded") return n203::radar::RadarHealth::kDegraded;
    if (value == "unknown") return n203::radar::RadarHealth::kUnknown;
    return n203::radar::RadarHealth::kHealthy;
}

n203::radar::PresenceState presenceState(const std::string& value, std::uint8_t contact_count) {
    if (value == "clear") return n203::radar::PresenceState::kClear;
    if (value == "detected") return n203::radar::PresenceState::kDetected;
    if (value == "unknown") return n203::radar::PresenceState::kUnknown;
    return contact_count == 0U ? n203::radar::PresenceState::kClear
                               : n203::radar::PresenceState::kDetected;
}

n203::radar::MotionState motionState(const std::string& value) {
    if (value == "tracking") return n203::radar::MotionState::kTracking;
    if (value == "doppler-warning") return n203::radar::MotionState::kDopplerWarning;
    if (value == "stabilizing") return n203::radar::MotionState::kStabilizing;
    return n203::radar::MotionState::kUnknown;
}

std::uint32_t capabilitiesFor(n203::radar::MeasurementState state) {
    using namespace n203::radar;
    switch (state) {
        case MeasurementState::kPresenceOnly:
            return kPresence;
        case MeasurementState::kRangeOnly:
            return kPresence | kRange;
        default:
            return kPresence | kRange | kPosition | kAzimuth | kVelocity | kTrackIdentity | kMultipleContacts;
    }
}

}  // namespace

bool ScenarioRadarSource::loadFromFile(const std::string& path, std::string& error) {
    const std::string document = readFile(path, error);
    if (!error.empty()) {
        return false;
    }

    frame_ = {};
    frame_.health = radarHealth(stringValue(document, "health", "healthy"));
    frame_.measurement_state = measurementState(stringValue(document, "scanner_state", "scanning"));
    frame_.capabilities.flags = capabilitiesFor(frame_.measurement_state);

    const std::regex contact_pattern(R"(\{[^{}]*"id"\s*:\s*([0-9]+)[^{}]*\})");
    for (std::sregex_iterator it(document.begin(), document.end(), contact_pattern), end;
         it != end && frame_.contact_count < n203::radar::kMaximumContacts; ++it) {
        const std::string contact_document = it->str();
        n203::radar::Contact& contact = frame_.contacts[frame_.contact_count++];
        contact.id = static_cast<std::uint8_t>(integerValue(contact_document, "id"));
        contact.x_mm = static_cast<std::int16_t>(integerValue(contact_document, "x_mm"));
        contact.y_mm = static_cast<std::int16_t>(integerValue(contact_document, "y_mm"));
        contact.range_mm = static_cast<std::uint16_t>(integerValue(contact_document, "range_mm"));
        contact.azimuth_cdeg = static_cast<std::int16_t>(integerValue(contact_document, "azimuth_cdeg"));
        contact.radial_velocity_mm_s = static_cast<std::int16_t>(integerValue(contact_document, "radial_velocity_mm_s"));
        contact.signal = static_cast<std::uint16_t>(integerValue(contact_document, "signal"));
        contact.confidence = static_cast<std::uint8_t>(integerValue(contact_document, "confidence"));
        contact.valid_fields = n203::radar::kContactConfidence | n203::radar::kContactSignal;
        if (contact_document.find("\"x_mm\"") != std::string::npos && contact_document.find("\"y_mm\"") != std::string::npos) {
            contact.valid_fields |= n203::radar::kContactPosition;
        }
        if (contact_document.find("\"range_mm\"") != std::string::npos) {
            contact.valid_fields |= n203::radar::kContactRange;
        }
        if (contact_document.find("\"azimuth_cdeg\"") != std::string::npos) {
            contact.valid_fields |= n203::radar::kContactAzimuth;
        }
        if (contact_document.find("\"radial_velocity_mm_s\"") != std::string::npos) {
            contact.valid_fields |= n203::radar::kContactRadialVelocity;
        }
    }

    frame_.presence = presenceState(stringValue(document, "presence"), frame_.contact_count);
    const std::string motion = stringValue(document, "motion");
    frame_.motion = motionState(motion);
    frame_.motion_quality = static_cast<std::uint8_t>(integerValue(
        document, "motion_quality", frame_.motion == n203::radar::MotionState::kUnknown ? 0 : 100));

    loaded_ = true;
    return true;
}

void ScenarioRadarSource::update(std::uint32_t now_ms) {
    if (!loaded_) {
        return;
    }
    frame_.sensor_timestamp_ms = now_ms;
    ++frame_.sequence;
}

bool ScenarioRadarSource::poll(n203::radar::RadarFrame& frame) {
    if (!loaded_) {
        return false;
    }
    frame = frame_;
    return true;
}

}  // namespace n203::emulation::display
