/**
 * @file imu_frequency.cpp
 * @brief Resolve the IMU source sampling rate from typed inputs or camera JSON.
 */
#include "spacemit_camera/imu_frequency.h"

#include <cmath>
#include <stdexcept>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace camera_sdk
{
namespace
{
constexpr double kMaximumImuFrequencyHz = 5000.0;
}

double ResolveImuFrequencyHz(
    std::optional<double> nominal_frequency_hz, int samples_per_frame, int camera_fps)
{
    double frequency_hz;
    if (nominal_frequency_hz)
    {
        frequency_hz = *nominal_frequency_hz;
    }
    else
    {
        if (samples_per_frame <= 0 || camera_fps <= 0)
        {
            throw std::runtime_error("IMU samples_per_frame and camera FPS must be positive");
        }
        frequency_hz = static_cast<double>(samples_per_frame) * camera_fps;
    }
    if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0 ||
        frequency_hz > kMaximumImuFrequencyHz)
    {
        throw std::runtime_error("resolved IMU frequency must be in (0, 5000]");
    }
    return frequency_hz;
}

double LoadImuFrequencyHz(const std::string& path, int camera_fps_override)
{
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    if (!root.get<bool>("imu.enabled", false))
    {
        throw std::runtime_error("automatic IMU frequency requires imu.enabled=true");
    }
    if (root.get_child_optional("imu.nominal_frequency_hz"))
    {
        return ResolveImuFrequencyHz(root.get<double>("imu.nominal_frequency_hz"), 0, 0);
    }
    const int samples_per_frame = root.get<int>("imu.samples_per_frame");
    const int camera_fps =
        camera_fps_override > 0 ? camera_fps_override : root.get<int>("camera.fps");
    return ResolveImuFrequencyHz(std::nullopt, samples_per_frame, camera_fps);
}

} // namespace camera_sdk
