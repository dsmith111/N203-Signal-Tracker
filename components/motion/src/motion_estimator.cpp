#include "n203/motion/motion_estimator.hpp"

#include <cmath>

namespace n203::motion {
namespace {

float magnitude(float x, float y, float z) {
    return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

void MotionEstimator::reset() {
    metrics_ = {};
    gravity_initialized_ = false;
    previous_timestamp_ms_ = 0U;
    gyro_bias_sum_x_ = 0.0F;
    gyro_bias_sum_y_ = 0.0F;
    gyro_bias_sum_z_ = 0.0F;
}

void MotionEstimator::observe(const ImuSample& sample) {
    ++metrics_.sample_count;
    metrics_.accel_x_g = sample.accel_x_g;
    metrics_.accel_y_g = sample.accel_y_g;
    metrics_.accel_z_g = sample.accel_z_g;
    metrics_.accel_magnitude_g =
        magnitude(sample.accel_x_g, sample.accel_y_g, sample.accel_z_g);

    if (!gravity_initialized_) {
        metrics_.gravity_x_g = sample.accel_x_g;
        metrics_.gravity_y_g = sample.accel_y_g;
        metrics_.gravity_z_g = sample.accel_z_g;
        gravity_initialized_ = true;
    } else {
        std::uint32_t elapsed_ms = sample.timestamp_ms - previous_timestamp_ms_;
        if (elapsed_ms == 0U || elapsed_ms > 1000U) {
            elapsed_ms = 10U;
        }
        const float alpha = kGravityFilterTimeConstantMs /
            (kGravityFilterTimeConstantMs + static_cast<float>(elapsed_ms));
        const float input_weight = 1.0F - alpha;
        metrics_.gravity_x_g =
            alpha * metrics_.gravity_x_g + input_weight * sample.accel_x_g;
        metrics_.gravity_y_g =
            alpha * metrics_.gravity_y_g + input_weight * sample.accel_y_g;
        metrics_.gravity_z_g =
            alpha * metrics_.gravity_z_g + input_weight * sample.accel_z_g;
    }
    previous_timestamp_ms_ = sample.timestamp_ms;

    const float dynamic_x = sample.accel_x_g - metrics_.gravity_x_g;
    const float dynamic_y = sample.accel_y_g - metrics_.gravity_y_g;
    const float dynamic_z = sample.accel_z_g - metrics_.gravity_z_g;
    metrics_.dynamic_acceleration_g = magnitude(dynamic_x, dynamic_y, dynamic_z);

    if (!metrics_.calibrated) {
        gyro_bias_sum_x_ += sample.gyro_x_dps;
        gyro_bias_sum_y_ += sample.gyro_y_dps;
        gyro_bias_sum_z_ += sample.gyro_z_dps;
        ++metrics_.bias_samples_collected;
        if (metrics_.bias_samples_collected >= kStartupBiasSamples) {
            const float count = static_cast<float>(metrics_.bias_samples_collected);
            metrics_.gyro_bias_x_dps = gyro_bias_sum_x_ / count;
            metrics_.gyro_bias_y_dps = gyro_bias_sum_y_ / count;
            metrics_.gyro_bias_z_dps = gyro_bias_sum_z_ / count;
            metrics_.calibrated = true;
        }
    }

    if (metrics_.calibrated) {
        metrics_.gyro_x_dps = sample.gyro_x_dps - metrics_.gyro_bias_x_dps;
        metrics_.gyro_y_dps = sample.gyro_y_dps - metrics_.gyro_bias_y_dps;
        metrics_.gyro_z_dps = sample.gyro_z_dps - metrics_.gyro_bias_z_dps;
    } else {
        metrics_.gyro_x_dps = sample.gyro_x_dps;
        metrics_.gyro_y_dps = sample.gyro_y_dps;
        metrics_.gyro_z_dps = sample.gyro_z_dps;
    }
    metrics_.gyro_magnitude_dps = magnitude(
        metrics_.gyro_x_dps, metrics_.gyro_y_dps, metrics_.gyro_z_dps);
}

}  // namespace n203::motion
