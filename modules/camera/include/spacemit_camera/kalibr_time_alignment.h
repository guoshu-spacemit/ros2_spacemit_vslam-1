// Checked Kalibr camera-to-IMU timestamp conversion.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_KALIBR_TIME_ALIGNMENT_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_KALIBR_TIME_ALIGNMENT_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace camera_sdk
{
// Kalibr cam0 convention, in seconds: t_imu = t_cam + timeshift_cam_imu.
// Keep source image timestamps unchanged and align IMU only at the VIO input.
// The inverse transform is also needed by bounded recorded-session lookahead.
class KalibrTimeAlignment
{
public:
    explicit KalibrTimeAlignment(std::optional<double> shift_seconds = std::nullopt)
        : calibrated_(shift_seconds.has_value())
    {
        if (!shift_seconds)
            return;
        if (!std::isfinite(*shift_seconds))
            throw std::runtime_error("cam0.timeshift_cam_imu must be finite seconds");
        const long double ns = std::round(static_cast<long double>(*shift_seconds) * 1.0e9L);
        const auto limit = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
        // Exclude INT64_MIN too, so inversion never overflows.
        if (ns < -limit || ns > limit)
            throw std::runtime_error("cam0.timeshift_cam_imu cannot be represented in nanoseconds");
        shift_ns_ = static_cast<std::int64_t>(ns);
    }

    std::int64_t shift_ns() const { return shift_ns_; }
    bool calibrated() const { return calibrated_; }

    std::int64_t imu_to_camera(std::int64_t raw_imu_ns) const
    {
        return add(raw_imu_ns, -shift_ns_);
    }

    std::int64_t camera_to_imu(std::int64_t image_ns) const { return add(image_ns, shift_ns_); }

private:
    static std::int64_t add(std::int64_t stamp, std::int64_t offset)
    {
        if (stamp <= 0 ||
            (offset > 0 && stamp > std::numeric_limits<std::int64_t>::max() - offset) ||
            (offset < 0 && stamp <= -offset))
            throw std::runtime_error("invalid timestamp after Kalibr camera/IMU time alignment");
        return stamp + offset;
    }

    std::int64_t shift_ns_{0};
    bool calibrated_{false};
};
} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_KALIBR_TIME_ALIGNMENT_H_
