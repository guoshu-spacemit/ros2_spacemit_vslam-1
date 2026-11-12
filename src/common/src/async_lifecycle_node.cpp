/**
 * @file async_lifecycle_node.cpp
 * @brief Dispatch lifecycle startup after executor registration and optional dependency readiness.
 */
#include "common/lifecycle/async_lifecycle_node.h"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace common::lifecycle
{
namespace
{
constexpr double kDefaultAutostartTimeoutSec = 120.0;
constexpr double kMaxAutostartTimeoutSec = 3600.0;
// Defer lifecycle callbacks until the executor can run the fully constructed node.
constexpr auto kAutostartDispatchDelay = std::chrono::milliseconds(1);
} // namespace

AsyncLifecycleNode::AsyncLifecycleNode(
    const std::string& node_name, const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    const bool autostart = declare_parameter<bool>("autostart", true);
    const auto ready_topic = declare_parameter<std::string>("autostart_ready_topic", "");
    const double timeout =
        declare_parameter<double>("autostart_ready_timeout_sec", kDefaultAutostartTimeoutSec);
    shutdown_on_failure_ = declare_parameter<bool>("autostart_shutdown_on_failure", false);
    if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > kMaxAutostartTimeoutSec)
    {
        throw std::invalid_argument("autostart_ready_timeout_sec must be in (0, 3600]");
    }
    if (autostart && !ready_topic.empty())
    {
        autostart_pending_ = true;
        ready_subscription_ = create_subscription<std_msgs::msg::Bool>(ready_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool& message)
            {
                if (message.data && autostart_pending_)
                {
                    RCLCPP_INFO(get_logger(), "startup dependency ready; configuring hardware");
                    RunAutostart();
                }
            });
        // A one-shot deadline, not a polling or startup-delay timer.
        const auto readiness_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(timeout));
        autostart_timer_ = create_wall_timer(readiness_timeout,
            [this]
            {
                cancel_autostart();
                if (get_current_state().id() ==
                    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
                {
                    OnAutostartFailure("startup dependency readiness timed out");
                }
            });
        RCLCPP_INFO(get_logger(), "waiting for startup dependency %s before configure",
            ready_topic.c_str());
    }
    else if (autostart)
    {
        autostart_pending_ = true;
        autostart_timer_ = create_wall_timer(kAutostartDispatchDelay, [this] { RunAutostart(); });
    }
}

void AsyncLifecycleNode::cancel_autostart()
{
    autostart_pending_ = false;
    ready_subscription_.reset();
    if (autostart_timer_)
    {
        autostart_timer_->cancel();
        autostart_timer_.reset();
    }
}

void AsyncLifecycleNode::RunAutostart()
{
    if (!autostart_pending_)
    {
        return;
    }
    cancel_autostart();
    // A lifecycle manager may have configured the node while it waited.
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
        return;
    }
    const auto configured =
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (configured.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(get_logger(), "automatic configure failed; lifecycle state=%s",
            configured.label().c_str());
        OnAutostartFailure("automatic configure failed");
        return;
    }

    const auto activated = trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (activated.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_ERROR(get_logger(), "automatic activate failed; lifecycle state=%s",
            activated.label().c_str());
        OnAutostartFailure("automatic activate failed");
    }
}

void AsyncLifecycleNode::OnAutostartFailure(const char* reason)
{
    RCLCPP_ERROR(get_logger(), "%s", reason);
    if (shutdown_on_failure_)
    {
        // Opt-in for supervised, standalone processes. Composed nodes keep
        // their existing manual-retry behavior unless explicitly enabled.
        rclcpp::shutdown(get_node_base_interface()->get_context(), reason);
    }
}

} // namespace common::lifecycle
