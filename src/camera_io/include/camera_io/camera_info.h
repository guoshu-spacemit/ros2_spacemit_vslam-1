#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdint>

namespace camera_sdk
{
struct KalibrCamera;
}

namespace camera_io
{
/// Describe a resized raw pinhole image using the supplied ROS stamp and frame ID.
/// Width and height are the positive output dimensions in pixels. The returned
/// message scales intrinsics from the calibration resolution, copies distortion,
/// and leaves stereo extrinsics to TF. Throws std::invalid_argument for invalid
/// geometry or intrinsics and for unsupported camera or distortion models.
sensor_msgs::msg::CameraInfo make_camera_info(const camera_sdk::KalibrCamera& camera,
    const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height);
} // namespace camera_io

#endif // CAMERA_INFO_H
