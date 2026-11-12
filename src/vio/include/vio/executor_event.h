#ifndef EXECUTOR_EVENT_H
#define EXECUTOR_EVENT_H
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/waitable.hpp>

namespace vio
{
// Coalescing event executed by ROS in the supplied callback group. No timer/thread.
class ExecutorEvent final : public rclcpp::Waitable
{
public:
    ExecutorEvent(rclcpp::Context::SharedPtr context, std::function<void()> callback)
        : guard_(context), callback_(std::move(callback))
    {
    }

    void trigger()
    {
        pending_.store(true);
        guard_.trigger();
    }
    size_t get_number_of_ready_guard_conditions() override { return 1; }
    void add_to_wait_set(rcl_wait_set_t* wait_set) override { guard_.add_to_wait_set(wait_set); }
    bool is_ready(rcl_wait_set_t*) override { return pending_.load(); }
    std::shared_ptr<void> take_data() override
    {
        return std::make_shared<bool>(pending_.exchange(false));
    }
    void execute(std::shared_ptr<void>& data) override
    {
        if (*std::static_pointer_cast<bool>(data))
        {
            callback_();
        }
    }

private:
    rclcpp::GuardCondition guard_;
    std::function<void()> callback_;
    std::atomic<bool> pending_{false};
};
} // namespace vio

#endif // EXECUTOR_EVENT_H
