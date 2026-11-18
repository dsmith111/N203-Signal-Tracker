#include <cassert>
#include <cmath>
#include <cstdint>

#include "n203/motion/motion_estimator.hpp"

namespace {

bool near(float actual, float expected, float tolerance = 0.001F) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

void runMotionEstimatorTests() {
    n203::motion::MotionEstimator estimator;
    for (std::uint16_t index = 0U;
         index < n203::motion::MotionEstimator::kStartupBiasSamples; ++index) {
        n203::motion::ImuSample sample{};
        sample.timestamp_ms = static_cast<std::uint32_t>(index) * 10U;
        sample.accel_z_g = 1.0F;
        sample.gyro_x_dps = 1.0F;
        sample.gyro_y_dps = -2.0F;
        sample.gyro_z_dps = 0.5F;
        estimator.observe(sample);
    }

    const auto& calibrated = estimator.metrics();
    assert(calibrated.calibrated);
    assert(calibrated.sample_count ==
           n203::motion::MotionEstimator::kStartupBiasSamples);
    assert(near(calibrated.gyro_bias_x_dps, 1.0F));
    assert(near(calibrated.gyro_bias_y_dps, -2.0F));
    assert(near(calibrated.gyro_bias_z_dps, 0.5F));
    assert(near(calibrated.gyro_magnitude_dps, 0.0F));
    assert(near(calibrated.accel_magnitude_g, 1.0F));
    assert(near(calibrated.dynamic_acceleration_g, 0.0F));

    n203::motion::ImuSample moving{};
    moving.timestamp_ms = 2000U;
    moving.accel_x_g = 0.5F;
    moving.accel_z_g = 1.0F;
    moving.gyro_x_dps = 11.0F;
    moving.gyro_y_dps = -2.0F;
    moving.gyro_z_dps = 0.5F;
    estimator.observe(moving);

    const auto& moved = estimator.metrics();
    assert(near(moved.gyro_x_dps, 10.0F));
    assert(near(moved.gyro_y_dps, 0.0F));
    assert(near(moved.gyro_z_dps, 0.0F));
    assert(near(moved.gyro_magnitude_dps, 10.0F));
    assert(moved.dynamic_acceleration_g > 0.45F);
    assert(moved.dynamic_acceleration_g < 0.50F);

    estimator.reset();
    assert(!estimator.metrics().calibrated);
    assert(estimator.metrics().sample_count == 0U);
}
