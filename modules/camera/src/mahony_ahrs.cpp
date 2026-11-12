// Estimate orientation from inertial samples with the Mahony AHRS filter.

#include "spacemit_camera/mahony_ahrs.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace camera_sdk
{

namespace
{

constexpr float kGravity = 9.80665F;
constexpr float kMinNormSquared = 1.0e-12F;

float norm_squared_scalar(const float* values, std::size_t count)
{
    float result = 0.0F;
    for (std::size_t i = 0; i < count; ++i)
    {
        result += values[i] * values[i];
    }
    return result;
}

} // namespace

MahonyAhrs::MahonyAhrs(AhrsConfig config) : config_(config)
{
    if (config_.kp < 0.0F || config_.ki < 0.0F || config_.accel_rejection_m_s2 < 0.0F ||
        config_.max_dt_seconds <= 0.0F)
    {
        throw std::invalid_argument("invalid Mahony AHRS configuration");
    }
}

bool MahonyAhrs::update(
    float gx, float gy, float gz, float ax, float ay, float az, std::uint64_t timestamp_ns)
{
    if (timestamp_ns == 0 || (initialized_ && timestamp_ns <= last_timestamp_ns_))
    {
        return false;
    }

    if (!initialized_)
    {
        initialize_from_acceleration(ax, ay, az);
        last_timestamp_ns_ = timestamp_ns;
        initialized_ = true;
        return true;
    }

    const float dt =
        static_cast<float>(static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1.0e-9);
    last_timestamp_ns_ = timestamp_ns;
    if (!std::isfinite(dt) || dt <= 0.0F || dt > config_.max_dt_seconds)
    {
        integral_x_ = 0.0F;
        integral_y_ = 0.0F;
        integral_z_ = 0.0F;
        return true;
    }

    const float acceleration[] = {ax, ay, az};
    const float accel_norm_squared = norm_squared(acceleration, 3);
    if (accel_norm_squared > kMinNormSquared)
    {
        const float accel_norm = std::sqrt(accel_norm_squared);
        const bool use_acceleration = config_.accel_rejection_m_s2 == 0.0F ||
            std::fabs(accel_norm - kGravity) <= config_.accel_rejection_m_s2;
        if (use_acceleration)
        {
            const float inv_accel_norm = 1.0F / accel_norm;
            const float measured_x = ax * inv_accel_norm;
            const float measured_y = ay * inv_accel_norm;
            const float measured_z = az * inv_accel_norm;

            const auto& q = orientation_;
            const float predicted_x = 2.0F * (q.x * q.z - q.w * q.y);
            const float predicted_y = 2.0F * (q.y * q.z + q.w * q.x);
            const float predicted_z = 1.0F - 2.0F * (q.x * q.x + q.y * q.y);

            const float error_x = measured_y * predicted_z - measured_z * predicted_y;
            const float error_y = measured_z * predicted_x - measured_x * predicted_z;
            const float error_z = measured_x * predicted_y - measured_y * predicted_x;

            integral_x_ += config_.ki * error_x * dt;
            integral_y_ += config_.ki * error_y * dt;
            integral_z_ += config_.ki * error_z * dt;
            gx += config_.kp * error_x + integral_x_;
            gy += config_.kp * error_y + integral_y_;
            gz += config_.kp * error_z + integral_z_;
        }
    }

    const Quaternion previous = orientation_;
    const float half_dt = 0.5F * dt;
    orientation_.w += (-previous.x * gx - previous.y * gy - previous.z * gz) * half_dt;
    orientation_.x += (previous.w * gx + previous.y * gz - previous.z * gy) * half_dt;
    orientation_.y += (previous.w * gy - previous.x * gz + previous.z * gx) * half_dt;
    orientation_.z += (previous.w * gz + previous.x * gy - previous.y * gx) * half_dt;
    normalize_orientation();
    return true;
}

float MahonyAhrs::norm_squared(const float* values, std::size_t count) const
{
    return norm_squared_scalar(values, count);
}

void MahonyAhrs::initialize_from_acceleration(float ax, float ay, float az)
{
    const float acceleration[] = {ax, ay, az};
    const float norm_squared_value = norm_squared(acceleration, 3);
    if (norm_squared_value <= kMinNormSquared)
    {
        orientation_ = {};
        return;
    }

    const float inverse_norm = 1.0F / std::sqrt(norm_squared_value);
    const float ux = ax * inverse_norm;
    const float uy = ay * inverse_norm;
    const float uz = az * inverse_norm;

    // Shortest rotation from measured body-frame up to world-frame +Z.
    if (uz < -0.9999F)
    {
        orientation_ = {0.0F, 1.0F, 0.0F, 0.0F};
    }
    else
    {
        orientation_ = {1.0F + uz, uy, -ux, 0.0F};
        normalize_orientation();
    }
}

void MahonyAhrs::normalize_orientation()
{
    const float values[] = {orientation_.w, orientation_.x, orientation_.y, orientation_.z};
    const float norm_squared_value = norm_squared(values, 4);
    if (norm_squared_value <= kMinNormSquared)
    {
        orientation_ = {};
        return;
    }
    const float inverse_norm = 1.0F / std::sqrt(norm_squared_value);
    orientation_.w *= inverse_norm;
    orientation_.x *= inverse_norm;
    orientation_.y *= inverse_norm;
    orientation_.z *= inverse_norm;
}

} // namespace camera_sdk
