/**
 * @file readiness_barrier.cpp
 * @brief Gate finite-file playback on independent LAS2 worker readiness.
 */
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("las2_readiness_barrier");
    const auto topics = node->declare_parameter<std::vector<std::string>>(
        "input_topics", std::vector<std::string>{});
    if (topics.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "input_topics must not be empty");
        rclcpp::shutdown();
        return 1;
    }
    auto output = node->create_publisher<std_msgs::msg::Bool>(
        node->declare_parameter<std::string>("output_topic", "/las2/quad/ready"),
        rclcpp::QoS(1).reliable().transient_local());
    std::vector<bool> ready(topics.size(), false);
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> inputs;
    for (std::size_t i = 0; i < topics.size(); ++i)
    {
        inputs.push_back(node->create_subscription<std_msgs::msg::Bool>(topics[i],
            rclcpp::QoS(1).reliable().transient_local(),
            [&, i](std_msgs::msg::Bool::ConstSharedPtr value)
            {
                ready[i] = value->data;
                std_msgs::msg::Bool result;
                result.data = std::all_of(ready.begin(), ready.end(), [](bool v) { return v; });
                output->publish(result);
            }));
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
