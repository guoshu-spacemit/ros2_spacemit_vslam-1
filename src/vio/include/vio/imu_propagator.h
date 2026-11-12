#ifndef IMU_PROPAGATOR_H
#define IMU_PROPAGATOR_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace vio
{

inline constexpr float kGravityMps2 = 9.80665F;

struct ImuSample
{
    std::int64_t timestamp_ns = 0;
    Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
};

struct PropagatedState
{
    bool valid = false;
    std::int64_t timestamp_ns = 0;
    Eigen::Quaternionf world_from_rig = Eigen::Quaternionf::Identity();
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
};

class ImuPropagator
{
public:
    void reset(std::int64_t timestamp_ns, const Eigen::Quaternionf& world_from_rig,
        const Eigen::Vector3f& position, const Eigen::Vector3f& velocity,
        const Eigen::Vector3f& gravity_world, const Eigen::Vector3f& gyro_bias,
        const Eigen::Vector3f& accel_bias, const Eigen::Matrix3f& rig_from_imu);

    bool integrate(const ImuSample& sample);

    PropagatedState state() const;

private:
    bool valid_ = false;
    std::int64_t timestamp_ns_ = 0;
    Eigen::Quaternionf world_from_rig_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f gravity_world_ = Eigen::Vector3f(0.0F, kGravityMps2, 0.0F);
    Eigen::Vector3f gyro_bias_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_bias_ = Eigen::Vector3f::Zero();
    Eigen::Matrix3f rig_from_imu_ = Eigen::Matrix3f::Identity();
};

} // namespace vio

#endif // IMU_PROPAGATOR_H
