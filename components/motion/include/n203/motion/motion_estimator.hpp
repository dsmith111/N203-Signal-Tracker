#pragma once

#include <cstdint>

namespace n203::motion {

struct ImuSample {
    std::uint32_t timestamp_ms = 0U;
    float accel_x_g = 0.0F;
    float accel_y_g = 0.0F;
    float accel_z_g = 0.0F;
    float gyro_x_dps = 0.0F;
    float gyro_y_dps = 0.0F;
    float gyro_z_dps = 0.0F;
};

struct MotionMetrics {
    bool calibrated = false;
    std::uint32_t sample_count = 0U;
    std::uint16_t bias_samples_collected = 0U;
    float accel_x_g = 0.0F;
    float accel_y_g = 0.0F;
    float accel_z_g = 0.0F;
    float accel_magnitude_g = 0.0F;
    float gravity_x_g = 0.0F;
    float gravity_y_g = 0.0F;
    float gravity_z_g = 0.0F;
    float gyro_x_dps = 0.0F;
    float gyro_y_dps = 0.0F;
    float gyro_z_dps = 0.0F;
    float gyro_magnitude_dps = 0.0F;
    float dynamic_acceleration_g = 0.0F;
    float gyro_bias_x_dps = 0.0F;
    float gyro_bias_y_dps = 0.0F;
    float gyro_bias_z_dps = 0.0F;
};

class MotionEstimator {
public:
    static constexpr std::uint16_t kStartupBiasSamples = 200U;
    static constexpr float kGravityFilterTimeConstantMs = 500.0F;

    void reset();
    void observe(const ImuSample& sample);

    const MotionMetrics& metrics() const { return metrics_; }

private:
    MotionMetrics metrics_{};
    bool gravity_initialized_ = false;
    std::uint32_t previous_timestamp_ms_ = 0U;
    float gyro_bias_sum_x_ = 0.0F;
    float gyro_bias_sum_y_ = 0.0F;
    float gyro_bias_sum_z_ = 0.0F;
};

}  // namespace n203::motion
