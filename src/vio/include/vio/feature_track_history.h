#ifndef FEATURE_TRACK_HISTORY_H
#define FEATURE_TRACK_HISTORY_H
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace vio
{

// Diagnostic only: continuity is between processed Track frames, not camera
// sequence numbers or DDS deliveries. Retain at most two frames of IDs.
class FeatureTrackHistory
{
public:
    struct Observation
    {
        float age_seconds;
        std::uint32_t count;
    };

    void reset()
    {
        previous_.clear();
        current_.clear();
        timestamp_ns_ = 0;
        initialized_ = false;
    }

    void begin_frame(std::int64_t timestamp_ns)
    {
        if (initialized_ && timestamp_ns <= timestamp_ns_)
            reset();
        previous_.swap(current_);
        current_.clear();
        timestamp_ns_ = timestamp_ns;
        initialized_ = true;
    }

    Observation observe(std::uint64_t id)
    {
        const auto inserted = current_.try_emplace(id, Entry{timestamp_ns_, 1});
        auto& entry = inserted.first->second;
        if (inserted.second)
        {
            const auto previous = previous_.find(id);
            if (previous != previous_.end())
            {
                entry = previous->second;
                if (entry.count < std::numeric_limits<std::uint32_t>::max())
                    ++entry.count;
            }
        }
        // Subtract before conversion, preserving nanosecond resolution even
        // for Unix timestamps. Unsigned subtraction avoids signed overflow.
        const auto elapsed = static_cast<std::uint64_t>(timestamp_ns_) -
            static_cast<std::uint64_t>(entry.first_timestamp_ns);
        return {static_cast<float>(static_cast<double>(elapsed) * 1e-9), entry.count};
    }

private:
    struct Entry
    {
        std::int64_t first_timestamp_ns;
        std::uint32_t count;
    };
    std::unordered_map<std::uint64_t, Entry> previous_;
    std::unordered_map<std::uint64_t, Entry> current_;
    std::int64_t timestamp_ns_ = 0;
    bool initialized_ = false;
};

} // namespace vio

#endif // FEATURE_TRACK_HISTORY_H
