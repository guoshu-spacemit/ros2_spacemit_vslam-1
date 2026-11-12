#ifndef OFFLINE_FIFO_H
#define OFFLINE_FIFO_H
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace las2
{
struct OfflineKey
{
    std::uint32_t index;
    std::int64_t stamp;
    bool operator<(const OfflineKey& other) const
    {
        return std::tie(index, stamp) < std::tie(other.index, other.stamp);
    }
};

inline OfflineKey parse_offline_ack(const std::string& value)
{
    const auto comma = value.find(',');
    if (comma == std::string::npos)
        throw std::runtime_error("invalid offline ACK");
    auto number = [](const char* begin, const char* end)
    {
        std::uint64_t result{};
        const auto parsed = std::from_chars(begin, end, result);
        if (begin == end || parsed.ec != std::errc{} || parsed.ptr != end)
            throw std::runtime_error("invalid offline ACK integer");
        return result;
    };
    const auto index = number(value.data(), value.data() + comma);
    const auto stamp = number(value.data() + comma + 1, value.data() + value.size());
    if (index > std::numeric_limits<std::uint32_t>::max() || stamp == 0 ||
        stamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("offline ACK outside source identity range");
    return {static_cast<std::uint32_t>(index), static_cast<std::int64_t>(stamp)};
}

// Caller owns synchronization. The window covers pending inputs, workers and
// output exports awaiting the consumer's exact ACK, including out-of-order ACKs.
template <class Item, class Lease> class OfflineFifo
{
public:
    using Clock = std::chrono::steady_clock;
    struct Work
    {
        OfflineKey key;
        Item item;
    };
    struct Stats
    {
        std::uint64_t accepted{}, dispatched{}, published{}, acknowledged{};
        std::size_t pending{}, active{}, waiting_ack{}, outstanding{}, inputs_in_use{};
        std::size_t maximum_active{}, maximum_outstanding{};
    };

    explicit OfflineFifo(std::size_t window) : window_(window)
    {
        if (window == 0 || window > 256)
            throw std::invalid_argument("offline window must be in [1, 256]");
    }
    void validate_next(OfflineKey key) const
    {
        if (key.stamp <= 0 || (last_ && (key.index <= last_->index || key.stamp <= last_->stamp)))
            throw std::runtime_error("duplicate or non-monotonic offline image identity");
    }
    bool has_credit() const { return states_.size() < window_; }
    bool ready() const { return !pending_.empty(); }
    void admit(OfflineKey key, Item item, Clock::time_point now = Clock::now())
    {
        validate_next(key);
        if (!has_credit())
            throw std::runtime_error("offline window capacity exceeded");
        states_.emplace(key, State{Phase::Queued, now, std::nullopt});
        try
        {
            pending_.push_back(Work{key, std::move(item)});
        }
        catch (...)
        {
            states_.erase(key);
            throw;
        }
        last_ = key;
        ++stats_.accepted;
        stats_.maximum_outstanding = std::max(stats_.maximum_outstanding, states_.size());
    }
    std::optional<Work> take()
    {
        if (pending_.empty())
            return std::nullopt;
        Work result = std::move(pending_.front());
        pending_.pop_front();
        states_.at(result.key).phase = Phase::Processing;
        inputs_in_use_.insert(result.key);
        ++stats_.dispatched;
        ++active_;
        stats_.maximum_active = std::max(stats_.maximum_active, active_);
        return result;
    }
    void published(OfflineKey key, Lease lease)
    {
        auto found = states_.find(key);
        if (found == states_.end() || found->second.phase != Phase::Processing)
            throw std::runtime_error("unexpected or duplicate offline completion");
        found->second.lease = std::move(lease);
        found->second.phase = Phase::Published;
        --active_;
        ++stats_.published;
    }
    Lease acknowledge(OfflineKey key)
    {
        auto found = states_.find(key);
        if (found == states_.end() || found->second.phase != Phase::Published)
            throw std::runtime_error("unsolicited, duplicate or early offline ACK");
        Lease lease = std::move(*found->second.lease);
        states_.erase(found);
        ++stats_.acknowledged;
        return lease;
    }
    // A fusion ACK can arrive while the worker still owns its imported input.
    // Call only after releasing that input and all temporary local VB handles.
    void released_input(OfflineKey key)
    {
        if (inputs_in_use_.erase(key) != 1)
            throw std::runtime_error("unexpected or duplicate offline input release");
    }
    bool complete(std::uint64_t expected, std::size_t exports_releasing = 0) const
    {
        return expected > 0 && stats_.accepted == expected && stats_.dispatched == expected &&
            stats_.published == expected && stats_.acknowledged == expected && states_.empty() &&
            inputs_in_use_.empty() && exports_releasing == 0;
    }
    std::optional<Clock::time_point> oldest_admission() const
    {
        if (states_.empty())
            return std::nullopt;
        auto oldest = states_.begin()->second.admitted;
        for (const auto& entry : states_)
            oldest = std::min(oldest, entry.second.admitted);
        return oldest;
    }
    Stats stats() const
    {
        auto result = stats_;
        result.pending = pending_.size();
        result.active = active_;
        result.outstanding = states_.size();
        result.inputs_in_use = inputs_in_use_.size();
        result.waiting_ack = states_.size() - result.pending - result.active;
        return result;
    }
    // Use only after workers joined. Return every remaining export exactly once
    // for node-owned VB_Unexport; clearing pending work releases imported inputs.
    std::vector<Lease> clear()
    {
        std::vector<Lease> leases;
        for (auto& entry : states_)
            if (entry.second.lease)
                leases.push_back(std::move(*entry.second.lease));
        pending_.clear();
        states_.clear();
        inputs_in_use_.clear();
        active_ = 0;
        return leases;
    }

private:
    enum class Phase
    {
        Queued,
        Processing,
        Published
    };
    struct State
    {
        Phase phase;
        Clock::time_point admitted;
        std::optional<Lease> lease;
    };
    std::size_t window_, active_{0};
    std::optional<OfflineKey> last_;
    std::deque<Work> pending_;
    std::map<OfflineKey, State> states_;
    std::set<OfflineKey> inputs_in_use_;
    Stats stats_;
};
} // namespace las2

#endif // OFFLINE_FIFO_H
