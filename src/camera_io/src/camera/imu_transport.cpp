/**
 * @file imu_transport.cpp
 * @brief Convert camera IMU samples from the SDK DDS representation to ROS messages.
 */
#include "imu_transport.h"

#include "Imu.h"

namespace camera_io::detail
{
sensor_msgs::msg::Imu ToRosImu(const my_sensor_msgs::msg::Imu& source)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp.sec = source.header().stamp().sec();
    message.header.stamp.nanosec = source.header().stamp().nanosec();
    message.header.frame_id = source.header().frame_id();
    message.orientation.x = source.orientation().x();
    message.orientation.y = source.orientation().y();
    message.orientation.z = source.orientation().z();
    message.orientation.w = source.orientation().w();
    message.orientation_covariance = source.orientation_covariance();
    message.angular_velocity.x = source.angular_velocity().x();
    message.angular_velocity.y = source.angular_velocity().y();
    message.angular_velocity.z = source.angular_velocity().z();
    message.angular_velocity_covariance = source.angular_velocity_covariance();
    message.linear_acceleration.x = source.linear_acceleration().x();
    message.linear_acceleration.y = source.linear_acceleration().y();
    message.linear_acceleration.z = source.linear_acceleration().z();
    message.linear_acceleration_covariance = source.linear_acceleration_covariance();
    return message;
}
} // namespace camera_io::detail
