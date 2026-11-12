#ifndef LOCAL_MAP_WINDOW_H
#define LOCAL_MAP_WINDOW_H
#include "esdf_drm/map_updates.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace esdf_drm
{

// Single owner: the mapping worker. Radius zero preserves the unbounded map.
class LocalMapWindow
{
public:
    using Clock = std::chrono::steady_clock;

    LocalMapWindow(const esdf_drm::mapping::MapOptions& params, double radius_m, double rate_hz)
        : mapper_(params), radius_m_(radius_m)
    {
        if (!std::isfinite(radius_m) || radius_m < 0.0 ||
            radius_m > std::numeric_limits<float>::max() || !std::isfinite(rate_hz) ||
            rate_hz <= 0.0 || rate_hz > 60.0)
            throw std::invalid_argument(
                "map clearing requires finite float radius >= 0 and rate in (0,60]");
        period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / rate_hz));
    }

    esdf_drm::mapping::Map& tsdf() { return mapper_; }

    std::vector<esdf_drm::mapping::ChunkKey> clear(
        const esdf_drm::mapping::Vec3& center, Clock::time_point now = Clock::now())
    {
        if (radius_m_ == 0.0 || (started_ && now < next_clear_))
            return {};
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z))
            throw std::invalid_argument("non-finite map clearing center");
        std::vector<mapping::ChunkKey> removed;
        for (const auto& change :
            mapper_.clear_outside(center, static_cast<float>(radius_m_)).changes())
            removed.push_back(change.key);
        started_ = true;
        next_clear_ = now + period_;
        return removed;
    }

private:
    esdf_drm::mapping::Map mapper_;
    double radius_m_;
    Clock::duration period_{};
    Clock::time_point next_clear_{};
    bool started_{false};
};

} // namespace esdf_drm

#endif // LOCAL_MAP_WINDOW_H
