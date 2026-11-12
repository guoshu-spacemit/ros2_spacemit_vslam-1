/**
 * @file camera_info.cpp
 * @brief Publish calibrated ROS camera metadata for native camera frames.
 */
#include "camera_io/camera_info.h"
#include "spacemit_camera/kalibr_calibration.h"

#include <cmath>
#include <stdexcept>

namespace camera_io
{
sensor_msgs::msg::CameraInfo make_camera_info(const camera_sdk::KalibrCamera& camera,
    const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height)
{
    if (camera.camera_model != "pinhole" || camera.resolution[0] <= 0 ||
        camera.resolution[1] <= 0 || width == 0 || height == 0)
        throw std::invalid_argument("CameraInfo requires valid pinhole image geometry");
    for (const auto value : camera.intrinsics)
        if (!std::isfinite(value))
            throw std::invalid_argument("CameraInfo intrinsics must be finite");
    if (camera.intrinsics[0] <= 0 || camera.intrinsics[1] <= 0)
        throw std::invalid_argument("CameraInfo focal lengths must be positive");

    sensor_msgs::msg::CameraInfo result;
    result.header = header;
    result.width = width;
    result.height = height;
    if (camera.distortion_model == "radtan")
        result.distortion_model = "plumb_bob";
    else if (camera.distortion_model == "equidistant")
        result.distortion_model = "equidistant";
    else
        throw std::invalid_argument("Unsupported CameraInfo distortion model");
    result.d = camera.distortion_coeffs;
    const double sx = static_cast<double>(width) / camera.resolution[0];
    const double sy = static_cast<double>(height) / camera.resolution[1];
    const double fx = camera.intrinsics[0] * sx, fy = camera.intrinsics[1] * sy;
    const double cx = camera.intrinsics[2] * sx, cy = camera.intrinsics[3] * sy;
    result.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    result.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    result.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
    return result;
}
} // namespace camera_io
