#ifndef DISPLAY_GRAVITY_SEED_H
#define DISPLAY_GRAVITY_SEED_H
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace esdf_drm
{

// A display-only down direction. No state or constraint enters the VO solver.
// Call with IMU timestamps already aligned to image time by the Kalibr shift.
class DisplayGravitySeed
{
public:
    struct Seed
    {
        std::array<double, 3> gravity;
        std::int64_t first_ns;
        std::int64_t last_ns;

        bool covers(std::int64_t image_ns) const
        {
            return image_ns >= first_ns && image_ns <= last_ns;
        }
    };

    std::optional<Seed> add(std::int64_t stamp, const std::array<double, 3>& acceleration,
        const std::array<double, 3>& gyro)
    {
        double norm2 = 0.0;
        double gyro2 = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            norm2 += acceleration[axis] * acceleration[axis];
            gyro2 += gyro[axis] * gyro[axis];
        }
        if (stamp <= 0 || !std::isfinite(norm2) || !std::isfinite(gyro2) || norm2 < 64.0 ||
            norm2 > 121.0 || gyro2 > 0.05 * 0.05 ||
            (count_ && (stamp <= last_ || stamp - last_ > 100000000)))
        {
            *this = {};
            return {};
        }
        if (!count_)
            first_ = stamp;
        last_ = stamp;
        ++count_;
        double variance_sum = 0;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const double delta = acceleration[axis] - mean_[axis];
            mean_[axis] += delta / static_cast<double>(count_);
            m2_[axis] += delta * (acceleration[axis] - mean_[axis]);
            variance_sum += m2_[axis] / static_cast<double>(count_);
        }
        if (count_ < 20 || stamp - first_ < 300000000)
            return {};
        if (variance_sum >= 0.04)
        {
            *this = {};
            return {};
        }
        // Keep extending this quiet interval until a covered image pose arrives.
        // Constant memory, no dependence on callback wall time or camera FPS.
        return Seed{{-mean_[0], -mean_[1], -mean_[2]}, first_, last_};
    }

private:
    std::int64_t first_{0}, last_{0};
    std::size_t count_{0};
    std::array<double, 3> mean_{}, m2_{};
};

inline bool valid_display_rotation(const std::array<double, 9>& rotation)
{
    for (double value : rotation)
        if (!std::isfinite(value))
            return false;
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
        {
            double dot = 0;
            for (std::size_t axis = 0; axis < 3; ++axis)
                dot += rotation[row * 3 + axis] * rotation[col * 3 + axis];
            if (std::abs(dot - (row == col ? 1.0 : 0.0)) > 1e-3)
                return false;
        }
    double determinant = 0;
    for (std::size_t i = 0; i < 3; ++i)
        determinant += rotation[i] *
            (rotation[3 + (i + 1) % 3] * rotation[6 + (i + 2) % 3] -
                rotation[3 + (i + 2) % 3] * rotation[6 + (i + 1) % 3]);
    return std::abs(determinant - 1.0) <= 1e-3;
}

} // namespace esdf_drm

#endif // DISPLAY_GRAVITY_SEED_H
