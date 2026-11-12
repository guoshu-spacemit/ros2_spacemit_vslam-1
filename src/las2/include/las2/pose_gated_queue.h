#ifndef POSE_GATED_QUEUE_H
#define POSE_GATED_QUEUE_H
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>

namespace las2
{

// Caller owns synchronization. Item may own a DMA import, so eviction must
// destroy the item rather than retaining a descriptor referring to reused data.
template <class Item> class PoseGatedQueue
{
public:
    PoseGatedQueue(std::size_t image_capacity, std::size_t pose_capacity)
        : image_capacity_(image_capacity), pose_capacity_(pose_capacity)
    {
        if (image_capacity == 0 || pose_capacity == 0)
        {
            throw std::invalid_argument("pose gate capacities must be positive");
        }
    }

    void push_image(std::int64_t stamp, Item item)
    {
        if (stamp <= dispatched_)
        {
            ++dropped_images_;
            return;
        }

        const bool inserted = images_.insert_or_assign(stamp, std::move(item)).second;
        if (!inserted)
        {
            ++dropped_images_;
        }
        trim_images();
    }

    void push_pose(std::int64_t stamp)
    {
        if (stamp <= dispatched_)
        {
            return;
        }
        poses_.insert(stamp);
        while (poses_.size() > pose_capacity_)
        {
            poses_.erase(poses_.begin());
        }
    }

    bool ready() const
    {
        for (auto it = images_.rbegin(); it != images_.rend(); ++it)
        {
            if (poses_.count(it->first) != 0)
            {
                return true;
            }
        }
        return false;
    }

    std::optional<Item> take_latest()
    {
        auto selected = images_.end();
        for (auto it = images_.rbegin(); it != images_.rend(); ++it)
        {
            if (poses_.count(it->first) != 0)
            {
                selected = std::prev(it.base());
                break;
            }
        }
        if (selected == images_.end())
        {
            return std::nullopt;
        }

        dispatched_ = selected->first;
        Item result = std::move(selected->second);
        const auto discarded_end = images_.upper_bound(dispatched_);
        dropped_images_ +=
            static_cast<std::uint64_t>(std::distance(images_.begin(), discarded_end) - 1);
        images_.erase(images_.begin(), discarded_end);
        poses_.erase(poses_.begin(), poses_.upper_bound(dispatched_));
        return result;
    }

    void clear()
    {
        images_.clear();
        poses_.clear();
    }

    std::size_t size() const { return images_.size(); }
    std::uint64_t dropped_images() const { return dropped_images_; }

private:
    void trim_images()
    {
        while (images_.size() > image_capacity_)
        {
            images_.erase(images_.begin());
            ++dropped_images_;
        }
    }

    const std::size_t image_capacity_;
    const std::size_t pose_capacity_;
    std::map<std::int64_t, Item> images_;
    std::set<std::int64_t> poses_;
    std::int64_t dispatched_{std::numeric_limits<std::int64_t>::min()};
    std::uint64_t dropped_images_{0};
};

} // namespace las2

#endif // POSE_GATED_QUEUE_H
