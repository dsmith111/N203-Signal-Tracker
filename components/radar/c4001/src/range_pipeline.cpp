#include "n203/radar/c4001/range_pipeline.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace n203::radar::c4001 {
namespace {

struct Field {
    const char* data = nullptr;
    std::size_t size = 0U;
};

bool startsWith(const Report& report, const char* prefix) {
    std::size_t index = 0U;
    while (prefix[index] != '\0') {
        if (index >= report.size || report.text[index] != prefix[index]) {
            return false;
        }
        ++index;
    }
    return true;
}

Field trim(Field field) {
    while (field.size != 0U && field.data[0] == ' ') {
        ++field.data;
        --field.size;
    }
    while (field.size != 0U && field.data[field.size - 1U] == ' ') {
        --field.size;
    }
    return field;
}

bool equals(Field field, const char* expected) {
    field = trim(field);
    std::size_t index = 0U;
    while (expected[index] != '\0') {
        if (index >= field.size || field.data[index] != expected[index]) {
            return false;
        }
        ++index;
    }
    return index == field.size;
}

bool split(const Report& report, std::array<Field, 8U>& fields) {
    std::size_t field_index = 0U;
    std::size_t field_start = 0U;
    for (std::size_t index = 0U; index <= report.size; ++index) {
        if (index != report.size && report.text[index] != ',') {
            continue;
        }
        if (field_index >= fields.size()) {
            return false;
        }
        fields[field_index++] = {report.text.data() + field_start, index - field_start};
        field_start = index + 1U;
    }
    return field_index == fields.size();
}

bool parseUnsigned(Field field, std::uint32_t maximum, std::uint32_t& value) {
    field = trim(field);
    if (field.size == 0U) {
        return false;
    }
    std::uint32_t parsed = 0U;
    for (std::size_t index = 0U; index < field.size; ++index) {
        const char character = field.data[index];
        if (character < '0' || character > '9') {
            return false;
        }
        const std::uint32_t digit = static_cast<std::uint32_t>(character - '0');
        if (digit > maximum || parsed > (maximum - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
    }
    value = parsed;
    return true;
}

bool parseMilli(Field field, bool allow_negative, std::int32_t& value) {
    field = trim(field);
    if (field.size == 0U) {
        return false;
    }

    std::size_t index = 0U;
    bool negative = false;
    if (field.data[index] == '-' || field.data[index] == '+') {
        negative = field.data[index] == '-';
        if (negative && !allow_negative) {
            return false;
        }
        if (++index == field.size) {
            return false;
        }
    }

    std::uint32_t whole = 0U;
    std::size_t whole_digits = 0U;
    while (index < field.size && field.data[index] >= '0' && field.data[index] <= '9') {
        const std::uint32_t digit = static_cast<std::uint32_t>(field.data[index] - '0');
        if (whole > (100'000U - digit) / 10U) {
            return false;
        }
        whole = whole * 10U + digit;
        ++whole_digits;
        ++index;
    }
    if (whole_digits == 0U) {
        return false;
    }

    std::uint32_t fraction = 0U;
    std::size_t fraction_digits = 0U;
    if (index < field.size && field.data[index] == '.') {
        ++index;
        while (index < field.size && field.data[index] >= '0' && field.data[index] <= '9') {
            if (fraction_digits >= 3U) {
                return false;
            }
            fraction = fraction * 10U +
                       static_cast<std::uint32_t>(field.data[index] - '0');
            ++fraction_digits;
            ++index;
        }
        if (fraction_digits == 0U) {
            return false;
        }
    }
    if (index != field.size) {
        return false;
    }
    while (fraction_digits < 3U) {
        fraction *= 10U;
        ++fraction_digits;
    }

    const std::uint32_t magnitude = whole * 1000U + fraction;
    if (magnitude > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
        return false;
    }
    value = negative ? -static_cast<std::int32_t>(magnitude)
                     : static_cast<std::int32_t>(magnitude);
    return true;
}

}  // namespace

RangeReportError decodeRangeReport(
    const Report& report,
    RangeMeasurement& measurement) {
    measurement = {};
    if (!startsWith(report, "$DFDMD,")) {
        return RangeReportError::kUnsupportedReport;
    }

    std::array<Field, 8U> fields{};
    if (!split(report, fields) || !equals(fields[0], "$DFDMD") ||
        !equals(fields[7], "*")) {
        return RangeReportError::kMalformedReport;
    }

    std::uint32_t target_count = 0U;
    if (!parseUnsigned(fields[1], 1U, target_count)) {
        return RangeReportError::kMalformedReport;
    }
    measurement.target_count = static_cast<std::uint8_t>(target_count);
    if (target_count == 0U) {
        return RangeReportError::kNone;
    }

    std::uint32_t status = 0U;
    std::int32_t range_mm = 0;
    std::int32_t radial_velocity_mm_s = 0;
    std::uint32_t signal = 0U;
    if (!parseUnsigned(fields[2], 255U, status) ||
        !parseMilli(fields[3], false, range_mm) ||
        !parseMilli(fields[4], true, radial_velocity_mm_s) ||
        !parseUnsigned(fields[5], std::numeric_limits<std::uint32_t>::max(), signal)) {
        return RangeReportError::kMalformedReport;
    }
    (void)status;

    if (range_mm <= 0 || range_mm > 25'000 ||
        radial_velocity_mm_s < -10'000 || radial_velocity_mm_s > 10'000) {
        return RangeReportError::kOutOfRange;
    }

    measurement.range_mm = static_cast<std::uint16_t>(range_mm);
    measurement.radial_velocity_mm_s =
        static_cast<std::int16_t>(radial_velocity_mm_s);
    measurement.signal = signal > std::numeric_limits<std::uint16_t>::max()
        ? std::numeric_limits<std::uint16_t>::max()
        : static_cast<std::uint16_t>(signal);
    return RangeReportError::kNone;
}

bool RangeFilter::observe(
    const RangeMeasurement& observed,
    RangeMeasurement& filtered) {
    if (observed.target_count > 1U) {
        return false;
    }

    if (!has_confirmed_) {
        if (candidate_count_ == 0U ||
            candidate_.target_count != observed.target_count) {
            candidate_ = observed;
            candidate_count_ = 1U;
            return false;
        }
        confirmed_ = observed;
        candidate_count_ = 0U;
        has_confirmed_ = true;
        filtered = confirmed_;
        return true;
    }

    if (observed.target_count == confirmed_.target_count) {
        candidate_count_ = 0U;
        confirmed_ = observed;
        filtered = confirmed_;
        return true;
    }

    if (candidate_count_ == 0U ||
        candidate_.target_count != observed.target_count) {
        candidate_ = observed;
        candidate_count_ = 1U;
        filtered = confirmed_;
        return true;
    }

    confirmed_ = observed;
    candidate_count_ = 0U;
    filtered = confirmed_;
    return true;
}

void RangeFilter::reset() {
    confirmed_ = {};
    candidate_ = {};
    candidate_count_ = 0U;
    has_confirmed_ = false;
}

RadarFrame makeRangeFrame(
    const RangeMeasurement& measurement,
    std::uint32_t sequence,
    std::uint32_t sensor_timestamp_ms) {
    RadarFrame frame{};
    frame.sequence = sequence;
    frame.sensor_timestamp_ms = sensor_timestamp_ms;
    frame.capabilities.flags = kRange | kVelocity;
    frame.health = RadarHealth::kHealthy;
    frame.measurement_state = MeasurementState::kRangeOnly;
    frame.presence = PresenceState::kUnknown;
    frame.motion = measurement.target_count == 0U
                       ? MotionState::kUnknown
                       : MotionState::kTracking;
    frame.motion_quality = 0U;

    if (measurement.target_count != 0U) {
        frame.contact_count = 1U;
        Contact& contact = frame.contacts[0];
        contact.id = 1U;
        contact.valid_fields = kContactRange | kContactRadialVelocity | kContactSignal;
        contact.range_mm = measurement.range_mm;
        contact.radial_velocity_mm_s = measurement.radial_velocity_mm_s;
        contact.signal = measurement.signal;
    }
    return frame;
}

}  // namespace n203::radar::c4001
