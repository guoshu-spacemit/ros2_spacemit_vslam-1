#ifndef EXACT_PAIR_QUEUE_H
#define EXACT_PAIR_QUEUE_H
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

namespace esdf_drm
{

// Exact timestamp join; caller owns synchronization. Do not erase an
// unmatched timestamp merely because a later worker completed first. Once a
// newer matched job is dequeued, older late work is deliberately rejected to
// keep mapping time monotonic.
template <class First, class Second> class ExactPairQueue
{
public:
    struct Pair
    {
        First first;
        Second second;
    };

    ExactPairQueue(
        std::size_t first_capacity, std::size_t second_capacity, std::size_t ready_capacity)
        : first_capacity_(first_capacity), second_capacity_(second_capacity),
            ready_capacity_(ready_capacity)
    {
        if (first_capacity == 0 || second_capacity == 0 || ready_capacity == 0)
        {
            throw std::invalid_argument("pair capacities must be positive");
        }
    }

    void push_first(std::int64_t stamp, First value)
    {
        if (stamp <= consumed_ || ready_.count(stamp) != 0)
        {
            return;
        }
        first_.insert_or_assign(stamp, std::move(value));
        match(stamp);
        trim(first_, first_capacity_);
    }

    void push_second(std::int64_t stamp, Second value)
    {
        ++received_second_;
        if (stamp <= consumed_ || ready_.count(stamp) != 0)
        {
            ++dropped_late_second_;
            return;
        }
        second_.insert_or_assign(stamp, std::move(value));
        match(stamp);
        if (second_.size() > second_capacity_)
        {
            dropped_unmatched_second_ += second_.size() - second_capacity_;
        }
        trim(second_, second_capacity_);
    }

    bool ready() const { return !ready_.empty(); }

    std::optional<Pair> pop()
    {
        if (ready_.empty())
        {
            return std::nullopt;
        }
        auto it = ready_.begin();
        consumed_ = it->first;
        Pair result = std::move(it->second);
        ready_.erase(it);
        first_.erase(first_.begin(), first_.upper_bound(consumed_));
        second_.erase(second_.begin(), second_.upper_bound(consumed_));
        return result;
    }

    void clear()
    {
        first_.clear();
        second_.clear();
        ready_.clear();
    }

    std::uint64_t matched() const { return matched_; }
    std::uint64_t dropped_ready() const { return dropped_ready_; }
    std::uint64_t received_second() const { return received_second_; }
    std::uint64_t dropped_late_second() const { return dropped_late_second_; }
    std::uint64_t dropped_unmatched_second() const { return dropped_unmatched_second_; }

private:
    template <class Map> static void trim(Map& map, std::size_t capacity)
    {
        while (map.size() > capacity)
        {
            map.erase(map.begin());
        }
    }

    void match(std::int64_t stamp)
    {
        auto first = first_.find(stamp);
        auto second = second_.find(stamp);
        if (first == first_.end() || second == second_.end())
        {
            return;
        }
        ready_.emplace(stamp, Pair{std::move(first->second), std::move(second->second)});
        first_.erase(first);
        second_.erase(second);
        ++matched_;
        while (ready_.size() > ready_capacity_)
        {
            ready_.erase(ready_.begin());
            ++dropped_ready_;
        }
    }

    const std::size_t first_capacity_;
    const std::size_t second_capacity_;
    const std::size_t ready_capacity_;
    std::map<std::int64_t, First> first_;
    std::map<std::int64_t, Second> second_;
    std::map<std::int64_t, Pair> ready_;
    std::int64_t consumed_{std::numeric_limits<std::int64_t>::min()};
    std::uint64_t matched_{0};
    std::uint64_t dropped_ready_{0};
    std::uint64_t received_second_{0};
    std::uint64_t dropped_late_second_{0};
    std::uint64_t dropped_unmatched_second_{0};
};

} // namespace esdf_drm

#endif // EXACT_PAIR_QUEUE_H
