#ifndef TRAJECTORY_WINDOW_H
#define TRAJECTORY_WINDOW_H
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>

namespace esdf_drm
{

// Source time, not display FPS or wall time: pausing replay preserves the trail.
class TrajectoryWindow
{
public:
    TrajectoryWindow(double seconds, std::size_t capacity, float minimum_distance)
        : capacity_(capacity), minimum_distance_(minimum_distance)
    {
        if (!std::isfinite(seconds) || seconds <= 0 || seconds > 3600 || capacity < 2 ||
            !std::isfinite(minimum_distance) || minimum_distance < 0)
            throw std::invalid_argument("invalid trajectory window");
        duration_ns_ = static_cast<std::int64_t>(seconds * 1.0e9);
    }

    bool update(std::int64_t timestamp_ns, const std::array<float, 3>& position)
    {
        if (timestamp_ns < 0)
            return false;
        for (float coordinate : position)
            if (!std::isfinite(coordinate))
                return false;
        if (ready_ && timestamp_ns == latest_ns_)
            return false;
        if (ready_ && timestamp_ns < latest_ns_)
            samples_.clear(); // Source restarted; never join different runs.
        ready_ = true;
        latest_ns_ = timestamp_ns;
        while (!samples_.empty() && timestamp_ns - samples_.front().timestamp_ns > duration_ns_)
            samples_.pop_front();
        float squared_distance = 0;
        if (!samples_.empty())
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const float delta = position[axis] - samples_.back().position[axis];
                squared_distance += delta * delta;
            }
        if (samples_.empty() || squared_distance >= minimum_distance_ * minimum_distance_)
            samples_.push_back({timestamp_ns, position});
        while (samples_.size() > capacity_)
            samples_.pop_front();
        return true;
    }

    std::vector<std::array<float, 3>> positions() const
    {
        std::vector<std::array<float, 3>> result;
        result.reserve(samples_.size());
        for (const auto& sample : samples_)
            result.push_back(sample.position);
        return result;
    }

    double span_seconds() const
    {
        return samples_.empty() ? 0.0 : (latest_ns_ - samples_.front().timestamp_ns) * 1.0e-9;
    }

private:
    struct Sample
    {
        std::int64_t timestamp_ns;
        std::array<float, 3> position;
    };
    std::deque<Sample> samples_;
    std::size_t capacity_;
    float minimum_distance_;
    std::int64_t duration_ns_{};
    std::int64_t latest_ns_{};
    bool ready_{false};
};

} // namespace esdf_drm

#endif // TRAJECTORY_WINDOW_H
