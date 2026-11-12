#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H
#include <sensor_msgs/msg/imu.hpp>

namespace my_sensor_msgs::msg
{
class Imu;
}

namespace camera_io::detail
{
// Preserve the source timestamp and covariance while selecting ROS RMW transport.
sensor_msgs::msg::Imu ToRosImu(const my_sensor_msgs::msg::Imu& source);
} // namespace camera_io::detail

#endif // IMU_TRANSPORT_H
