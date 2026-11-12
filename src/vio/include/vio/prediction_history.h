#ifndef PREDICTION_HISTORY_H
#define PREDICTION_HISTORY_H
#include <cstdint>
#include <limits>

namespace vio
{

// Read-only producer-side accounting. Pauses and skipped display/DDS frames
// cannot alter the streak. Duration is source-time since the first predicted pose.
class PredictionHistory
{
public:
    void reset() { *this = {}; }

    void update(std::int64_t stamp, bool predicted)
    {
        if (initialized_ && stamp <= last_stamp_)
            reset();
        if (!predicted)
            count_ = 0;
        else
        {
            if (count_ == 0)
                first_stamp_ = stamp;
            if (count_ < std::numeric_limits<std::uint32_t>::max())
                ++count_;
        }
        last_stamp_ = stamp;
        initialized_ = true;
    }

    std::uint32_t count() const { return count_; }
    float span_ms() const
    {
        return count_ == 0 ? 0.0F
            : static_cast<float>((static_cast<std::uint64_t>(last_stamp_) -
                static_cast<std::uint64_t>(first_stamp_)) *
                1e-6);
    }

private:
    std::int64_t first_stamp_ = 0;
    std::int64_t last_stamp_ = 0;
    std::uint32_t count_ = 0;
    bool initialized_ = false;
};

} // namespace vio

#endif // PREDICTION_HISTORY_H
