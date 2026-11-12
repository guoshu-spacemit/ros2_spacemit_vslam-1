#ifndef LOCALIZATION_REQUEST_H
#define LOCALIZATION_REQUEST_H
#include <Eigen/Geometry>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace vio
{
struct LocalizationCompletion
{
    bool success{};
    std::string detail;
    std::int64_t timestamp_ns{};
    double elapsed_ms{};
    Eigen::Isometry3d map_from_tracking = Eigen::Isometry3d::Identity();
};

// A completion callback owns only this state. Cancel invalidates prior session results.
class LocalizationRequest
{
public:
    std::optional<std::uint64_t> begin(std::int64_t timestamp_ns)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (in_flight_ || pending_)
            return std::nullopt;
        in_flight_ = true;
        timestamp_ns_ = timestamp_ns;
        return ++generation_;
    }
    bool complete(std::uint64_t generation, LocalizationCompletion completion)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_ || !in_flight_)
            return false;
        completion.timestamp_ns = timestamp_ns_;
        in_flight_ = false;
        pending_ = std::move(completion);
        return true;
    }
    std::optional<LocalizationCompletion> take()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = std::move(pending_);
        pending_.reset();
        return result;
    }
    bool busy() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return in_flight_ || pending_.has_value();
    }
    void cancel()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_;
        in_flight_ = false;
        pending_.reset();
    }

private:
    mutable std::mutex mutex_;
    std::uint64_t generation_{};
    bool in_flight_{};
    std::int64_t timestamp_ns_{};
    std::optional<LocalizationCompletion> pending_;
};
} // namespace vio

#endif // LOCALIZATION_REQUEST_H
