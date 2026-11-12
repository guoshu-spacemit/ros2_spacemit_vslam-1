#ifndef SESSION_REPLAY_WINDOW_H
#define SESSION_REPLAY_WINDOW_H
#include "session_replay_index.h"

#include <chrono>
#include <deque>
#include <optional>

namespace camera_io::replay
{

// All methods run under the replay mutex. Completed holes still consume window
// credit: FIFO camera exports and decoder leases may only retire with the
// contiguous prefix, even when downstream workers finish out of order.
class AckWindow
{
public:
    using Clock = std::chrono::steady_clock;

    AckWindow(const std::vector<Target>& targets, std::size_t capacity, Clock::duration timeout)
        : targets_(targets), capacity_(capacity), timeout_(timeout)
    {
        if (targets.empty() || capacity == 0 || capacity > 16 || timeout <= Clock::duration::zero())
            throw std::runtime_error("invalid frozen replay window configuration");
    }

    bool can_issue() const { return issued_ < targets_.size() && issued_ - prefix_ < capacity_; }

    std::size_t issue(Target target, Clock::time_point now)
    {
        check_timeout(now);
        if (!can_issue() || target.id != targets_[issued_].id ||
            target.stamp != targets_[issued_].stamp)
            throw std::runtime_error("out-of-order or overflowing frozen target issue");
        slots_.push_back({target, now + timeout_, false});
        const auto index = issued_++;
        high_water_ = std::max(high_water_, issued_ - prefix_);
        return index;
    }

    void acknowledge(Target target, Clock::time_point now)
    {
        check_timeout(now);
        const auto found = std::find_if(slots_.begin(), slots_.end(), [target](const Slot& slot)
            { return slot.target.id == target.id && slot.target.stamp == target.stamp; });
        if (found == slots_.end() || found->complete)
            throw std::runtime_error("unexpected, duplicate or mismatched frozen ACK: " +
                std::to_string(target.id) + "," + std::to_string(target.stamp));
        found->complete = true;
        ++completed_;
        while (!slots_.empty() && slots_.front().complete)
        {
            slots_.pop_front();
            ++prefix_;
        }
    }

    void check_timeout(Clock::time_point now) const
    {
        for (const auto& slot : slots_)
        {
            if (!slot.complete && now >= slot.deadline)
                throw std::runtime_error("timeout: frozen target ACK " +
                    std::to_string(slot.target.id) + "," + std::to_string(slot.target.stamp));
        }
    }

    std::optional<Clock::time_point> next_deadline() const
    {
        std::optional<Clock::time_point> deadline;
        for (const auto& slot : slots_)
            if (!slot.complete && (!deadline || slot.deadline < *deadline))
                deadline = slot.deadline;
        return deadline;
    }

    std::size_t issued() const { return issued_; }
    std::size_t prefix() const { return prefix_; }
    std::size_t completed() const { return completed_; }
    std::size_t high_water() const { return high_water_; }

private:
    struct Slot
    {
        Target target;
        Clock::time_point deadline;
        bool complete;
    };
    const std::vector<Target>& targets_;
    const std::size_t capacity_;
    const Clock::duration timeout_;
    std::deque<Slot> slots_;
    std::size_t issued_{0}, prefix_{0}, completed_{0}, high_water_{0};
};

// A final event barrier is separate from per-target acknowledgements. The
// downstream owner may observe the final ACK after this producer does, and
// must finish releasing its own output leases before launch may stop it.
class CompletionGate
{
public:
    using Clock = std::chrono::steady_clock;
    CompletionGate(bool enabled, Clock::duration timeout) : enabled_(enabled), timeout_(timeout)
    {
        if (timeout <= Clock::duration::zero())
            throw std::runtime_error("invalid replay completion timeout");
    }

    void start(Clock::time_point now)
    {
        if (!started_)
        {
            deadline_ = now + timeout_;
            started_ = true;
        }
    }

    void receive(const std::string& message, Clock::time_point now)
    {
        if (!enabled_)
            return;
        if (!error_.empty())
            throw std::runtime_error(error_);
        if (started_ && !complete_ && now >= deadline_)
            error_ = "timeout: downstream replay completion";
        else if (message == "complete")
            complete_ = true;
        else
            error_ = "downstream replay completion failed: " + message;
        if (!error_.empty())
            throw std::runtime_error(error_);
    }

    bool ready(Clock::time_point now) const
    {
        if (!enabled_)
            return true;
        if (!error_.empty())
            throw std::runtime_error(error_);
        if (complete_)
            return true;
        if (!started_)
            throw std::runtime_error("replay completion wait has not started");
        if (now >= deadline_)
            throw std::runtime_error("timeout: downstream replay completion");
        return false;
    }

    bool completed() const { return complete_ && error_.empty(); }

private:
    const bool enabled_;
    const Clock::duration timeout_;
    bool started_{false}, complete_{false};
    Clock::time_point deadline_{};
    std::string error_;
};

} // namespace camera_io::replay

#endif // SESSION_REPLAY_WINDOW_H
