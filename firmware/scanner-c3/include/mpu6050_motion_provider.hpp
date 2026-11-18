#pragma once

#include <Wire.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "n203/motion/motion_estimator.hpp"

namespace n203::firmware::scanner_c3 {

enum class Mpu6050State : std::uint8_t {
    kDisabled,
    kProbing,
    kConfiguring,
    kCalibrating,
    kReady,
    kDisconnected,
};

struct Mpu6050Stats {
    std::uint32_t samples = 0U;
    std::uint32_t read_errors = 0U;
    std::uint32_t write_errors = 0U;
    std::uint32_t disconnects = 0U;
    std::uint32_t reconnects = 0U;
    std::uint32_t last_sample_ms = 0U;
};

class Mpu6050MotionProvider {
public:
    static constexpr std::uint32_t kSampleIntervalMs = 10U;

    void begin(TwoWire& wire, bool bus_ready, std::uint32_t now_ms);
    void service(std::uint32_t now_ms);

    Mpu6050State state() const { return state_; }
    std::uint8_t address() const { return address_; }
    const Mpu6050Stats& stats() const { return stats_; }
    const n203::motion::MotionMetrics& metrics() const {
        return estimator_.metrics();
    }

private:
    bool probeAddress(std::uint8_t address);
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegisters(std::uint8_t reg, std::uint8_t* destination,
                       std::size_t size);
    bool readSample(std::uint32_t now_ms);
    void loseDevice(std::uint32_t now_ms);

    TwoWire* wire_ = nullptr;
    Mpu6050State state_ = Mpu6050State::kDisabled;
    Mpu6050Stats stats_{};
    n203::motion::MotionEstimator estimator_{};
    std::uint8_t address_ = 0U;
    std::uint8_t probe_index_ = 0U;
    std::size_t configuration_index_ = 0U;
    std::uint8_t consecutive_errors_ = 0U;
    std::uint32_t next_action_ms_ = 0U;
    bool device_seen_ = false;
};

const char* mpu6050StateName(Mpu6050State state);

}  // namespace n203::firmware::scanner_c3
