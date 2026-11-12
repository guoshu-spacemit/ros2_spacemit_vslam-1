// Mahony inertial orientation-estimation interface.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_MAHONY_AHRS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_MAHONY_AHRS_H_

#include <cstddef>
#include <cstdint>
namespace camera_sdk
{

struct AhrsConfig
{
    float kp{1.0F};
    float ki{0.0F};
    float accel_rejection_m_s2{3.0F};
    float max_dt_seconds{0.05F};
};

struct Quaternion
{
    float w{1.0F};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

class MahonyAhrs
{
public:
    explicit MahonyAhrs(AhrsConfig config = {});

    // Returns false for a duplicate, backward, or otherwise invalid timestamp.
    bool update(
        float gx, float gy, float gz, float ax, float ay, float az, std::uint64_t timestamp_ns);

    const Quaternion& orientation() const { return orientation_; }
    bool initialized() const { return initialized_; }

private:
    float norm_squared(const float* values, std::size_t count) const;
    void initialize_from_acceleration(float ax, float ay, float az);
    void normalize_orientation();

    AhrsConfig config_;
    Quaternion orientation_;
    float integral_x_{0.0F};
    float integral_y_{0.0F};
    float integral_z_{0.0F};
    std::uint64_t last_timestamp_ns_{0};
    bool initialized_{false};
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_MAHONY_AHRS_H_
