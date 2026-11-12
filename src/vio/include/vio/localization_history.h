#ifndef LOCALIZATION_HISTORY_H
#define LOCALIZATION_HISTORY_H
#include <Eigen/Geometry>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>

namespace vio
{
inline Eigen::Isometry3d localization_pose(
    double x, double y, double z, double qx, double qy, double qz, double qw)
{
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (!Eigen::Vector3d(x, y, z).allFinite() || !q.coeffs().allFinite() ||
        std::abs(q.squaredNorm() - 1.0) > 1e-4)
        throw std::invalid_argument("Localization pose must be finite with a unit quaternion");
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = q.normalized().toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, z);
    return pose;
}

struct LocalizationStereo
{
    std::int64_t timestamp_ns{};
    std::uint32_t source_frame_index{};
    std::uint32_t width{}, height{};
    std::vector<std::uint8_t> left, right;
    std::size_t bytes() const { return left.size() + right.size(); }
};

inline std::vector<std::uint8_t> localization_copy_y(const std::uint8_t* pixels, std::size_t width,
    std::size_t height, std::size_t stride, std::size_t extent)
{
    if (!pixels || !width || !height || stride < width ||
        height > std::numeric_limits<std::size_t>::max() / stride ||
        (height - 1) * stride + width > extent)
        throw std::invalid_argument("Invalid localization Y plane extent");
    std::vector<std::uint8_t> result(width * height);
    for (std::size_t row = 0; row < height; ++row)
        std::memcpy(result.data() + row * width, pixels + row * stride, width);
    return result;
}

// Access belongs to the mutually exclusive stereo callback group. Contains no DMA tokens.
class LocalizationHistory
{
public:
    LocalizationHistory(std::size_t capacity, std::size_t maximum_bytes)
        : capacity_(capacity), maximum_bytes_(maximum_bytes)
    {
        if (!capacity || !maximum_bytes)
            throw std::invalid_argument("Localization history limits must be positive");
    }
    bool push(LocalizationStereo frame)
    {
        if (frame.timestamp_ns <= 0 || frame.bytes() > maximum_bytes_ ||
            (!frames_.empty() && frame.timestamp_ns <= frames_.back().timestamp_ns))
            return false;
        while (!frames_.empty() &&
            (frames_.size() >= capacity_ || bytes_ > maximum_bytes_ - frame.bytes()))
        {
            bytes_ -= frames_.front().bytes();
            frames_.pop_front();
        }
        bytes_ += frame.bytes();
        frames_.push_back(std::move(frame));
        return true;
    }
    const LocalizationStereo* find(std::int64_t timestamp_ns) const
    {
        const auto found = std::find_if(frames_.begin(), frames_.end(),
            [timestamp_ns](const LocalizationStereo& frame)
            { return frame.timestamp_ns == timestamp_ns; });
        return found == frames_.end() ? nullptr : &*found;
    }
    void clear()
    {
        frames_.clear();
        bytes_ = 0;
    }
    std::size_t bytes() const { return bytes_; }
    std::size_t size() const { return frames_.size(); }
    std::size_t maximum_bytes() const { return maximum_bytes_; }

private:
    std::size_t capacity_, maximum_bytes_, bytes_{};
    std::deque<LocalizationStereo> frames_;
};
} // namespace vio

#endif // LOCALIZATION_HISTORY_H
