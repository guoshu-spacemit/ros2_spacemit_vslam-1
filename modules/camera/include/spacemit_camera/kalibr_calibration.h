// Kalibr stereo-camera calibration data and loading interface.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_KALIBR_CALIBRATION_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_KALIBR_CALIBRATION_H_

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace camera_sdk
{

using Transform4d = std::array<double, 16>;

struct KalibrCamera
{
    std::string name;
    std::string camera_model;
    std::string distortion_model;
    std::string rostopic;
    std::array<double, 4> intrinsics{};
    std::vector<double> distortion_coeffs;
    std::array<int, 2> resolution{};
    std::optional<Transform4d> t_cam_imu;
    std::optional<Transform4d> t_cn_cnm1;
    std::optional<double> timeshift_cam_imu;
};

struct KalibrCalibration
{
    std::vector<KalibrCamera> cameras;

    bool has_imu_extrinsics() const;
};

KalibrCalibration load_kalibr_calibration(const std::string& path);

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_KALIBR_CALIBRATION_H_
