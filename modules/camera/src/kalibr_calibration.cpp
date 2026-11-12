// Load Kalibr camera and IMU calibration while retaining camera ordering.

#include "spacemit_camera/kalibr_calibration.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace camera_sdk
{
namespace
{

bool is_camera_key(const std::string& key)
{
    if (key.size() < 4 || key.compare(0, 3, "cam") != 0)
        return false;
    return std::all_of(
        key.begin() + 3, key.end(), [](unsigned char value) { return std::isdigit(value) != 0; });
}

int camera_index(const std::string& key) { return std::stoi(key.substr(3)); }

template <std::size_t Size, typename Value>
std::array<Value, Size> load_fixed_sequence(const YAML::Node& node, const std::string& field)
{
    if (!node || !node.IsSequence() || node.size() != Size)
    {
        throw std::runtime_error(field + " must contain " + std::to_string(Size) + " values");
    }
    std::array<Value, Size> result{};
    for (std::size_t i = 0; i < Size; ++i)
    {
        result[i] = node[i].as<Value>();
    }
    return result;
}

Transform4d load_transform(const YAML::Node& node, const std::string& field)
{
    if (!node || !node.IsSequence() || node.size() != 4)
    {
        throw std::runtime_error(field + " must be a 4x4 matrix");
    }
    Transform4d result{};
    for (std::size_t row = 0; row < 4; ++row)
    {
        if (!node[row].IsSequence() || node[row].size() != 4)
        {
            throw std::runtime_error(field + " must be a 4x4 matrix");
        }
        for (std::size_t column = 0; column < 4; ++column)
        {
            result[row * 4 + column] = node[row][column].as<double>();
        }
    }
    return result;
}

KalibrCamera load_camera(const std::string& name, const YAML::Node& node)
{
    KalibrCamera camera;
    camera.name = name;
    camera.camera_model = node["camera_model"].as<std::string>();
    camera.distortion_model = node["distortion_model"].as<std::string>();
    camera.rostopic = node["rostopic"].as<std::string>("");
    camera.intrinsics = load_fixed_sequence<4, double>(node["intrinsics"], name + ".intrinsics");
    camera.resolution = load_fixed_sequence<2, int>(node["resolution"], name + ".resolution");
    if (camera.resolution[0] <= 0 || camera.resolution[1] <= 0)
    {
        throw std::runtime_error(name + ".resolution must be positive");
    }
    const YAML::Node distortion = node["distortion_coeffs"];
    if (!distortion || !distortion.IsSequence())
    {
        throw std::runtime_error(name + ".distortion_coeffs must be a sequence");
    }
    for (const auto& coefficient : distortion)
    {
        camera.distortion_coeffs.push_back(coefficient.as<double>());
    }
    if (camera.camera_model != "pinhole")
    {
        throw std::runtime_error(name + ".camera_model must be pinhole");
    }
    if (camera.distortion_model != "radtan" && camera.distortion_model != "equidistant")
    {
        throw std::runtime_error(name + ".distortion_model must be radtan or equidistant");
    }
    if (camera.distortion_coeffs.size() != 4)
    {
        throw std::runtime_error(
            name + ".distortion_coeffs must contain 4 values for " + camera.distortion_model);
    }
    if (node["T_cam_imu"])
    {
        camera.t_cam_imu = load_transform(node["T_cam_imu"], name + ".T_cam_imu");
    }
    if (node["T_cn_cnm1"])
    {
        camera.t_cn_cnm1 = load_transform(node["T_cn_cnm1"], name + ".T_cn_cnm1");
    }
    if (node["timeshift_cam_imu"])
    {
        camera.timeshift_cam_imu = node["timeshift_cam_imu"].as<double>();
    }
    return camera;
}

} // namespace

bool KalibrCalibration::has_imu_extrinsics() const
{
    return std::any_of(cameras.begin(), cameras.end(),
        [](const KalibrCamera& camera) { return camera.t_cam_imu.has_value(); });
}

KalibrCalibration load_kalibr_calibration(const std::string& path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& error)
    {
        throw std::runtime_error("failed to load Kalibr calibration " + path + ": " + error.what());
    }
    if (!root.IsMap())
    {
        throw std::runtime_error("Kalibr calibration root must be a map");
    }

    std::vector<std::pair<int, KalibrCamera>> indexed;
    for (const auto& entry : root)
    {
        const std::string key = entry.first.as<std::string>();
        if (!is_camera_key(key))
            continue;
        indexed.emplace_back(camera_index(key), load_camera(key, entry.second));
    }
    if (indexed.empty())
    {
        throw std::runtime_error("Kalibr calibration does not contain any camN entries");
    }
    std::sort(indexed.begin(), indexed.end(),
        [](const auto& left, const auto& right) { return left.first < right.first; });
    KalibrCalibration calibration;
    for (std::size_t i = 0; i < indexed.size(); ++i)
    {
        if (indexed[i].first != static_cast<int>(i))
        {
            throw std::runtime_error("Kalibr camera entries must be contiguous from cam0");
        }
        calibration.cameras.push_back(std::move(indexed[i].second));
    }
    return calibration;
}

} // namespace camera_sdk
