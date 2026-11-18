#include "mpu6050_motion_provider.hpp"

#include <array>

namespace n203::firmware::scanner_c3 {
namespace {

constexpr std::array<std::uint8_t, 2> kCandidateAddresses{{0x68U, 0x69U}};
constexpr std::uint8_t kWhoAmIRegister = 0x75U;
constexpr std::uint8_t kWhoAmIValue = 0x68U;
constexpr std::uint8_t kSampleRegister = 0x3BU;
constexpr std::uint32_t kProbeRetryMs = 1000U;
constexpr std::uint8_t kMaximumConsecutiveErrors = 5U;
constexpr float kAccelLsbPerG = 16384.0F;
constexpr float kGyroLsbPerDps = 131.0F;

constexpr std::array<std::array<std::uint8_t, 2>, 6> kConfiguration{{
        {{0x6BU, 0x01U}},  // Wake; use X gyro PLL clock.
        {{0x6CU, 0x00U}},  // Enable every accelerometer and gyro axis.
        {{0x1AU, 0x03U}},  // DLPF: approximately 42-44 Hz bandwidth.
        {{0x19U, 0x09U}},  // 1 kHz / (1 + 9) = 100 Hz sample rate.
        {{0x1BU, 0x00U}},  // Gyro full scale: +/-250 degrees/second.
        {{0x1CU, 0x00U}},  // Accelerometer full scale: +/-2 g.
    }};

std::int16_t decodeSignedBigEndian(std::uint8_t high, std::uint8_t low) {
    const std::uint16_t value =
        (static_cast<std::uint16_t>(high) << 8U) | low;
    return static_cast<std::int16_t>(value);
}

}  // namespace

const char* mpu6050StateName(Mpu6050State state) {
    switch (state) {
        case Mpu6050State::kDisabled: return "disabled";
        case Mpu6050State::kProbing: return "probing";
        case Mpu6050State::kConfiguring: return "configuring";
        case Mpu6050State::kCalibrating: return "calibrating";
        case Mpu6050State::kReady: return "ready";
        case Mpu6050State::kDisconnected: return "disconnected";
    }
    return "invalid";
}

void Mpu6050MotionProvider::begin(
    TwoWire& wire, bool bus_ready, std::uint32_t now_ms) {
    wire_ = &wire;
    stats_ = {};
    estimator_.reset();
    address_ = 0U;
    probe_index_ = 0U;
    configuration_index_ = 0U;
    consecutive_errors_ = 0U;
    next_action_ms_ = now_ms;
    device_seen_ = false;
    state_ = bus_ready ? Mpu6050State::kProbing : Mpu6050State::kDisabled;
}

void Mpu6050MotionProvider::service(std::uint32_t now_ms) {
    if (wire_ == nullptr || state_ == Mpu6050State::kDisabled ||
        static_cast<std::int32_t>(now_ms - next_action_ms_) < 0) {
        return;
    }

    if (state_ == Mpu6050State::kProbing ||
        state_ == Mpu6050State::kDisconnected) {
        const std::uint8_t candidate = kCandidateAddresses[probe_index_];
        if (probeAddress(candidate)) {
            address_ = candidate;
            configuration_index_ = 0U;
            consecutive_errors_ = 0U;
            estimator_.reset();
            if (device_seen_) {
                ++stats_.reconnects;
            }
            device_seen_ = true;
            state_ = Mpu6050State::kConfiguring;
            next_action_ms_ = now_ms;
            return;
        }
        probe_index_ = static_cast<std::uint8_t>(
            (probe_index_ + 1U) % kCandidateAddresses.size());
        next_action_ms_ = now_ms +
            (probe_index_ == 0U ? kProbeRetryMs : 1U);
        return;
    }

    if (state_ == Mpu6050State::kConfiguring) {
        const auto& setting = kConfiguration[configuration_index_];
        if (!writeRegister(setting[0], setting[1])) {
            ++stats_.write_errors;
            loseDevice(now_ms);
            return;
        }
        ++configuration_index_;
        if (configuration_index_ == kConfiguration.size()) {
            state_ = Mpu6050State::kCalibrating;
            next_action_ms_ = now_ms + kSampleIntervalMs;
        } else {
            next_action_ms_ = now_ms + 1U;
        }
        return;
    }

    if (!readSample(now_ms)) {
        ++stats_.read_errors;
        ++consecutive_errors_;
        if (consecutive_errors_ >= kMaximumConsecutiveErrors) {
            loseDevice(now_ms);
        } else {
            next_action_ms_ = now_ms + kSampleIntervalMs;
        }
        return;
    }

    consecutive_errors_ = 0U;
    if (estimator_.metrics().calibrated) {
        state_ = Mpu6050State::kReady;
    }
    next_action_ms_ = now_ms + kSampleIntervalMs;
}

bool Mpu6050MotionProvider::probeAddress(std::uint8_t address) {
    address_ = address;
    // On ESP32, endTransmission(false) defers the transaction into the
    // following requestFrom(). If no device is present, that combined
    // operation emits an ESP-IDF error even though absence is expected here.
    // A stopped, address-only transaction first gives us a quiet ACK/NACK gate
    // and keeps an optional/unplugged IMU from flooding the USB console.
    wire_->beginTransmission(address_);
    if (wire_->endTransmission(true) != 0U) {
        address_ = 0U;
        return false;
    }
    std::uint8_t identity = 0U;
    const bool recognized =
        readRegisters(kWhoAmIRegister, &identity, 1U) &&
        identity == kWhoAmIValue;
    if (!recognized) {
        address_ = 0U;
    }
    return recognized;
}

bool Mpu6050MotionProvider::writeRegister(
    std::uint8_t reg, std::uint8_t value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission(true) == 0U;
}

bool Mpu6050MotionProvider::readRegisters(
    std::uint8_t reg, std::uint8_t* destination, std::size_t size) {
    if (address_ == 0U || size == 0U || size > 255U) {
        return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0U) {
        return false;
    }
    const std::uint8_t requested = static_cast<std::uint8_t>(size);
    if (wire_->requestFrom(address_, requested, static_cast<std::uint8_t>(true)) !=
        requested) {
        while (wire_->available() > 0) {
            wire_->read();
        }
        return false;
    }
    for (std::size_t index = 0U; index < size; ++index) {
        const int value = wire_->read();
        if (value < 0) {
            return false;
        }
        destination[index] = static_cast<std::uint8_t>(value);
    }
    return true;
}

bool Mpu6050MotionProvider::readSample(std::uint32_t now_ms) {
    std::array<std::uint8_t, 14> bytes{};
    if (!readRegisters(kSampleRegister, bytes.data(), bytes.size())) {
        return false;
    }

    n203::motion::ImuSample sample{};
    sample.timestamp_ms = now_ms;
    sample.accel_x_g =
        static_cast<float>(decodeSignedBigEndian(bytes[0], bytes[1])) /
        kAccelLsbPerG;
    sample.accel_y_g =
        static_cast<float>(decodeSignedBigEndian(bytes[2], bytes[3])) /
        kAccelLsbPerG;
    sample.accel_z_g =
        static_cast<float>(decodeSignedBigEndian(bytes[4], bytes[5])) /
        kAccelLsbPerG;
    sample.gyro_x_dps =
        static_cast<float>(decodeSignedBigEndian(bytes[8], bytes[9])) /
        kGyroLsbPerDps;
    sample.gyro_y_dps =
        static_cast<float>(decodeSignedBigEndian(bytes[10], bytes[11])) /
        kGyroLsbPerDps;
    sample.gyro_z_dps =
        static_cast<float>(decodeSignedBigEndian(bytes[12], bytes[13])) /
        kGyroLsbPerDps;

    estimator_.observe(sample);
    ++stats_.samples;
    stats_.last_sample_ms = now_ms;
    return true;
}

void Mpu6050MotionProvider::loseDevice(std::uint32_t now_ms) {
    ++stats_.disconnects;
    address_ = 0U;
    probe_index_ = 0U;
    configuration_index_ = 0U;
    consecutive_errors_ = 0U;
    state_ = Mpu6050State::kDisconnected;
    next_action_ms_ = now_ms + kProbeRetryMs;
}

}  // namespace n203::firmware::scanner_c3
