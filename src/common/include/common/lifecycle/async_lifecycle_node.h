#ifndef ASYNC_LIFECYCLE_NODE_H
#define ASYNC_LIFECYCLE_NODE_H

#include <memory>
#include <string>

#include "rclcpp/node_options.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rclcpp
{
class SubscriptionBase;
class TimerBase;
} // namespace rclcpp

namespace common::lifecycle
{

/// Configure and activate a lifecycle node from its executor after optional readiness.
class AsyncLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// Construct the named node with the supplied ROS parameters and context.
    /// With autostart enabled, a true message on autostart_ready_topic starts
    /// configuration. An empty topic starts configuration on the first timer callback.
    /// A readiness timeout must be finite and in (0, 3600] seconds or construction
    /// throws std::invalid_argument. An automatic startup failure shuts down the
    /// node context only when autostart_shutdown_on_failure is enabled.
    explicit AsyncLifecycleNode(
        const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
    /// Cancel pending automatic startup before spinning or from the default callback group.
    void cancel_autostart();

private:
    void RunAutostart();
    void OnAutostartFailure(const char* reason);

    bool autostart_pending_{false};
    bool shutdown_on_failure_{false};
    std::shared_ptr<rclcpp::TimerBase> autostart_timer_;
    std::shared_ptr<rclcpp::SubscriptionBase> ready_subscription_;
};

} // namespace common::lifecycle

#endif // ASYNC_LIFECYCLE_NODE_H
