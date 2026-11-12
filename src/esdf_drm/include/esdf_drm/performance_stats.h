#ifndef PERFORMANCE_STATS_H
#define PERFORMANCE_STATS_H
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string_view>

namespace esdf_drm
{
// Bounded, one-second wall-clock window. No image/source timestamps and no
// timer-driven fake events. An idle producer reports zero instead of stale FPS.
class EventWindow
{
public:
    using Clock = std::chrono::steady_clock;
    struct Sample
    {
        double fps = 0.0;
        double mean_ms = 0.0;
        bool valid = false;
    };

    void record(double ms = 0.0, Clock::time_point now = Clock::now())
    {
        if (!std::isfinite(ms) || ms < 0.0)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_)
        {
            start_ = now;
            started_ = true;
        }
        events_[next_] = {now, ms};
        next_ = (next_ + 1) % events_.size();
        count_ = std::min(count_ + 1, events_.size());
    }

    Sample sample(Clock::time_point now = Clock::now()) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Sample result;
        std::size_t active = 0;
        for (std::size_t i = 0; i < count_; ++i)
        {
            const double age = std::chrono::duration<double>(now - events_[i].time).count();
            if (age >= 0.0 && age < 1.0)
            {
                ++active;
                result.mean_ms += (events_[i].ms - result.mean_ms) / active;
            }
        }
        if (active == 0)
            return result;
        const double elapsed = std::chrono::duration<double>(now - start_).count();
        result.fps = elapsed >= 1.0 ? active : (elapsed > 0.0 ? (active - 1) / elapsed : 0.0);
        result.valid = true;
        return result;
    }

private:
    struct Event
    {
        Clock::time_point time{};
        double ms = 0.0;
    };
    mutable std::mutex mutex_;
    std::array<Event, 512> events_{};
    std::size_t next_ = 0;
    std::size_t count_ = 0;
    bool started_ = false;
    Clock::time_point start_{};
};

// Only bounded, known labels reach the HUD. A stopped publisher must not leave
// a cached VIO label on screen indefinitely. Updates come from completed Track
// calls, never from IMU receipt or a render timer.
class TrackingMode
{
public:
    using Clock = std::chrono::steady_clock;
    void record(std::string_view mode, Clock::time_point now = Clock::now())
    {
        const char* label = "UNKNOWN";
        for (const char* known : {"VO", "VO - IMU INIT", "VIO", "LOST", "WAIT", "STOPPED", "ERROR"})
            if (mode == known)
                label = known;
        std::lock_guard<std::mutex> lock(mutex_);
        label_ = label;
        received_ = now;
        valid_ = true;
    }

    const char* sample(Clock::time_point now = Clock::now()) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ || now < received_ || now - received_ >= std::chrono::seconds(2))
            return "NO DATA";
        return label_;
    }

private:
    mutable std::mutex mutex_;
    const char* label_ = "NO DATA";
    Clock::time_point received_{};
    bool valid_ = false;
};

struct PerformanceStats
{
    TrackingMode tracking_mode;
    EventWindow vio;
    EventWindow tsdf;
    EventWindow las2;
};
} // namespace esdf_drm

#endif // PERFORMANCE_STATS_H
