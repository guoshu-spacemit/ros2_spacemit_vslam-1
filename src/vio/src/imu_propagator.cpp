/**
 * @file imu_propagator.cpp
 * @brief Propagate image-time poses with calibrated inertial measurements.
 */
#include "vio/imu_propagator.h"

#include <algorithm>
#include <cmath>

namespace vio
{

void ImuPropagator::reset(std::int64_t timestamp_ns, const Eigen::Quaternionf& world_from_rig,
    const Eigen::Vector3f& position, const Eigen::Vector3f& velocity,
    const Eigen::Vector3f& gravity_world, const Eigen::Vector3f& gyro_bias,
    const Eigen::Vector3f& accel_bias, const Eigen::Matrix3f& rig_from_imu)
{
    timestamp_ns_ = timestamp_ns;
    world_from_rig_ = world_from_rig.normalized();
    position_ = position;
    velocity_ = velocity;
    gravity_world_ = gravity_world;
    gyro_bias_ = gyro_bias;
    accel_bias_ = accel_bias;
    rig_from_imu_ = rig_from_imu;
    valid_ = true;
}

bool ImuPropagator::integrate(const ImuSample& sample)
{
    if (!valid_ || sample.timestamp_ns <= timestamp_ns_)
    {
        return false;
    }

    const float dt = static_cast<float>(sample.timestamp_ns - timestamp_ns_) * 1.0e-9F;
    if (!(dt > 0.0F) || dt > 0.1F)
    {
        timestamp_ns_ = sample.timestamp_ns;
        return false;
    }

    const Eigen::Vector3f gyro_rig = rig_from_imu_ * (sample.angular_velocity - gyro_bias_);
    const Eigen::Vector3f accel_rig = rig_from_imu_ * (sample.acceleration - accel_bias_);
    const float angle = gyro_rig.norm() * dt;
    Eigen::Quaternionf delta = Eigen::Quaternionf::Identity();
    if (angle > 1.0e-7F)
    {
        delta = Eigen::Quaternionf(Eigen::AngleAxisf(angle, gyro_rig.normalized()));
    }
    else
    {
        delta.w() = 1.0F;
        delta.vec() = 0.5F * gyro_rig * dt;
        delta.normalize();
    }

    const Eigen::Vector3f acceleration_world = world_from_rig_ * accel_rig + gravity_world_;
    position_ += velocity_ * dt + 0.5F * acceleration_world * dt * dt;
    velocity_ += acceleration_world * dt;
    world_from_rig_ = (world_from_rig_ * delta).normalized();
    timestamp_ns_ = sample.timestamp_ns;
    return true;
}

PropagatedState ImuPropagator::state() const
{
    PropagatedState result;
    result.valid = valid_;
    result.timestamp_ns = timestamp_ns_;
    result.world_from_rig = world_from_rig_;
    result.position = position_;
    result.velocity = velocity_;
    return result;
}

} // namespace vio
