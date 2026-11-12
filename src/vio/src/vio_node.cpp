/**
 * @file vio_node.cpp
 * @brief Adapt synchronized camera and IMU descriptors to the native VSLAM SDK.
 */
#include <spacemit_vslam.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rmw/qos_profiles.h>
#include <sys/sys_api.h>
#include <sys/vb_api.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <fstream>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <iomanip>
#include <limits>
#include <string_view>
#include <condition_variable>
#include <cerrno>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <set>
#include <sstream>
#include <shared_mutex>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <common/msg/feature_point_array.hpp>
#include <vio/executor_event.h>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <spacemit_camera/imu_frequency.h>
#include <spacemit_camera/kalibr_time_alignment.h>
#include "vio/frame_feature_diagnostics.h"
#include "vio/dma_import_audit.h"
#include "vio/frame_recorder.h"
#include "vio/imu_propagator.h"
#include "vio/localization_history.h"
#include "vio/localization_request.h"
#include "vio/observation_settings.h"
#include "vio/replay_manifest.h"
#include "vio/tracking_mode.h"

namespace vio
{
namespace localization = spacemit_vslam::localization;

using Frame = mpp_msgs::msg::MppVideoFrameDesc;
using StereoSyncPolicy = message_filters::sync_policies::ExactTime<Frame, Frame>;

std::array<double, 16> LoadCalibrationMatrix(const YAML::Node& node, const std::string& name)
{
    if (!node || !node.IsSequence() || node.size() != 4)
    {
        throw std::runtime_error(name + " must be a 4x4 matrix");
    }
    std::array<double, 16> matrix{};
    for (std::size_t row = 0; row < 4; ++row)
    {
        if (!node[row].IsSequence() || node[row].size() != 4)
        {
            throw std::runtime_error(name + " must be a 4x4 matrix");
        }
        for (std::size_t column = 0; column < 4; ++column)
        {
            matrix[row * 4 + column] = node[row][column].as<double>();
        }
    }
    return matrix;
}

std::array<double, 16> InvertRigidTransform(const std::array<double, 16>& input)
{
    std::array<double, 16> output{input[0], input[4], input[8], 0.0, input[1], input[5], input[9],
        0.0, input[2], input[6], input[10], 0.0, 0.0, 0.0, 0.0, 1.0};
    output[3] = -(output[0] * input[3] + output[1] * input[7] + output[2] * input[11]);
    output[7] = -(output[4] * input[3] + output[5] * input[7] + output[6] * input[11]);
    output[11] = -(output[8] * input[3] + output[9] * input[7] + output[10] * input[11]);
    return output;
}

Eigen::Matrix3f rotation_from_matrix(const std::array<double, 16>& matrix)
{
    Eigen::Matrix3f rotation;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            rotation(row, column) = static_cast<float>(matrix[row * 4 + column]);
        }
    }
    return rotation;
}

localization::Pose pose_from_matrix(const std::array<double, 16>& matrix)
{
    const Eigen::Matrix3f rotation = rotation_from_matrix(matrix);
    const Eigen::Quaternionf quaternion(rotation);
    const Eigen::Quaternionf normalized = quaternion.normalized();
    localization::Pose pose;
    pose.orientation_xyzw = {normalized.x(), normalized.y(), normalized.z(), normalized.w()};
    pose.translation_m = {static_cast<float>(matrix[3]), static_cast<float>(matrix[7]),
        static_cast<float>(matrix[11])};
    return pose;
}

localization::CameraCalibration LoadCameraCalibration(
    const YAML::Node& node, const std::string& name, int output_width, int output_height)
{
    if (!node || !node["camera_model"] || !node["distortion_model"])
    {
        throw std::runtime_error(name + " is missing camera model fields");
    }
    const std::string model = node["camera_model"].as<std::string>();
    const std::string distortion_model = node["distortion_model"].as<std::string>();
    if (model != "pinhole")
    {
        throw std::runtime_error(name + ": only pinhole cameras are supported");
    }
    const auto intrinsics = node["intrinsics"].as<std::vector<float>>();
    const auto resolution = node["resolution"].as<std::vector<int>>();
    const auto distortion = node["distortion_coeffs"].as<std::vector<float>>();
    if (intrinsics.size() != 4 || resolution.size() != 2)
    {
        throw std::runtime_error("invalid " + name + " intrinsics or resolution");
    }

    if ((output_width == 0) != (output_height == 0) || output_width < 0 || output_height < 0)
    {
        throw std::runtime_error("image_width and image_height must both be zero or positive");
    }
    if (output_width == 0)
    {
        output_width = resolution[0];
        output_height = resolution[1];
    }
    const float scale_x = static_cast<float>(output_width) / static_cast<float>(resolution[0]);
    const float scale_y = static_cast<float>(output_height) / static_cast<float>(resolution[1]);

    localization::CameraCalibration camera;
    camera.intrinsics.image = {output_width, output_height};
    camera.intrinsics.focal_length_px = {intrinsics[0] * scale_x, intrinsics[1] * scale_y};
    camera.intrinsics.principal_point_px = {intrinsics[2] * scale_x, intrinsics[3] * scale_y};
    if (distortion_model == "radtan")
    {
        if (distortion.size() < 4)
        {
            throw std::runtime_error(name + ": radtan requires four coefficients");
        }
        camera.distortion.model = localization::LensDistortion::Model::Radtan;
        camera.distortion.coefficients = {distortion[0], distortion[1], distortion[2],
            distortion[3], distortion.size() > 4 ? distortion[4] : 0.0F};
    }
    else if (distortion_model == "equidistant")
    {
        if (distortion.size() != 4)
        {
            throw std::runtime_error(name + ": equidistant requires four coefficients");
        }
        camera.distortion.model = localization::LensDistortion::Model::Equidistant;
        camera.distortion.coefficients = distortion;
    }
    else
    {
        throw std::runtime_error(name + ": unsupported distortion model " + distortion_model);
    }
    return camera;
}

struct LoadedRig
{
    localization::RigCalibration rig;
    camera_sdk::KalibrTimeAlignment time_alignment;
    Eigen::Matrix3f rig_from_imu = Eigen::Matrix3f::Identity();
    std::string imu_calibration_source = "disabled";
    float imu_yaml_update_rate_hz = 0.0F;
};

struct ImuNoiseCalibration
{
    float gyroscope_noise_density = 1.6968e-4F;
    float gyroscope_random_walk = 1.9393e-5F;
    float accelerometer_noise_density = 2.0e-3F;
    float accelerometer_random_walk = 3.0e-3F;
    float update_rate_hz = 0.0F;
    std::string source = "built-in EuRoC defaults";
};

float load_positive_scalar(const YAML::Node& node, const std::string& name)
{
    if (!node)
    {
        throw std::runtime_error("IMU calibration is missing " + name);
    }
    const float value = node.as<float>();
    if (!std::isfinite(value) || !(value > 0.0F))
    {
        throw std::runtime_error("IMU calibration " + name + " must be finite and positive");
    }
    return value;
}

ImuNoiseCalibration load_imu_noise_calibration(const std::string& path)
{
    ImuNoiseCalibration calibration;
    if (path.empty())
    {
        return calibration;
    }

    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node imu = root["imu0"];
    if (!imu || !imu.IsMap())
    {
        throw std::runtime_error("IMU Kalibr file requires an imu0 mapping");
    }
    calibration.gyroscope_noise_density =
        load_positive_scalar(imu["gyroscope_noise_density"], "imu0.gyroscope_noise_density");
    calibration.gyroscope_random_walk =
        load_positive_scalar(imu["gyroscope_random_walk"], "imu0.gyroscope_random_walk");
    calibration.accelerometer_noise_density = load_positive_scalar(
        imu["accelerometer_noise_density"], "imu0.accelerometer_noise_density");
    calibration.accelerometer_random_walk =
        load_positive_scalar(imu["accelerometer_random_walk"], "imu0.accelerometer_random_walk");
    if (imu["update_rate"])
    {
        calibration.update_rate_hz = load_positive_scalar(imu["update_rate"], "imu0.update_rate");
    }
    calibration.source = path;
    return calibration;
}

LoadedRig LoadRigCalibration(const std::string& path, bool use_imu, int output_width,
    int output_height, float imu_frequency_hz, const std::string& imu_calibration_file)
{
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node cam0 = root["cam0"];
    const YAML::Node cam1 = root["cam1"];
    if (!cam0 || !cam1 || !cam1["T_cn_cnm1"])
    {
        throw std::runtime_error("stereo Kalibr file requires cam0, cam1 and T_cn_cnm1");
    }

    localization::CameraCalibration left =
        LoadCameraCalibration(cam0, "cam0", output_width, output_height);
    localization::CameraCalibration right =
        LoadCameraCalibration(cam1, "cam1", output_width, output_height);
    if (left.intrinsics.image.width_px != right.intrinsics.image.width_px ||
        left.intrinsics.image.height_px != right.intrinsics.image.height_px)
    {
        throw std::runtime_error("stereo camera resolutions differ");
    }
    left.body_from_camera = localization::Pose{};
    right.body_from_camera = pose_from_matrix(
        InvertRigidTransform(LoadCalibrationMatrix(cam1["T_cn_cnm1"], "cam1.T_cn_cnm1")));

    LoadedRig loaded;
    loaded.rig.cameras = {std::move(left), std::move(right)};
    if (use_imu)
    {
        if (!cam0["T_cam_imu"])
        {
            throw std::runtime_error("IMU mode requires cam0.T_cam_imu");
        }
        if (cam0["timeshift_cam_imu"])
            loaded.time_alignment =
                camera_sdk::KalibrTimeAlignment(cam0["timeshift_cam_imu"].as<double>());
        const auto cam_from_imu = LoadCalibrationMatrix(cam0["T_cam_imu"], "cam0.T_cam_imu");
        const ImuNoiseCalibration noise = load_imu_noise_calibration(imu_calibration_file);
        localization::InertialCalibration imu;
        imu.body_from_sensor = pose_from_matrix(cam_from_imu);
        loaded.rig_from_imu = rotation_from_matrix(cam_from_imu);
        imu.angular_noise.white_noise_density = noise.gyroscope_noise_density;
        imu.angular_noise.bias_random_walk = noise.gyroscope_random_walk;
        imu.linear_noise.white_noise_density = noise.accelerometer_noise_density;
        imu.linear_noise.bias_random_walk = noise.accelerometer_random_walk;
        imu.sample_rate_hz = imu_frequency_hz;
        loaded.rig.inertial_sensors.push_back(imu);
        loaded.imu_calibration_source = noise.source;
        loaded.imu_yaml_update_rate_hz = noise.update_rate_hz;
    }
    return loaded;
}

struct MppImportOps
{
    rclcpp::Logger logger;
    int import(std::uint64_t token, UL* handle) const { return VB_Import(token, handle); }
    int release(UL handle) const { return VB_ReleaseBuffer(handle); }
    void release_error(int result) const noexcept
    {
        RCLCPP_ERROR(logger, "VIO VB_ReleaseBuffer failed: result=%d", result);
    }
};

struct DmaAuditFileCloser
{
    void operator()(std::FILE* file) const noexcept
    {
        if (file)
            (void)std::fclose(file);
    }
};

class ImportedBuffer
{
public:
    ImportedBuffer(
        std::uint64_t token, std::shared_ptr<DmaImportAudit> audit, const rclcpp::Logger& logger)
        : audit_(std::move(audit)), reference_(token, audit_, MppImportOps{logger})
    {
        if (VB_GetVirAddr(reference_.handle(), &address_) != 0 || address_ == nullptr)
        {
            ++audit_->map_errors;
            throw std::runtime_error("VB_GetVirAddr failed");
        }
    }

    ImportedBuffer(const ImportedBuffer&) = delete;
    ImportedBuffer& operator=(const ImportedBuffer&) = delete;

    const std::uint8_t* address() const { return static_cast<const std::uint8_t*>(address_); }

    std::vector<std::uint8_t> copy_y(const Frame& frame) const
    {
        S32 fd = -1;
        U64 physical = 0;
        struct stat info{};
        const auto stride = frame.plane_stride[0];
        const auto extent =
            static_cast<std::uint64_t>(stride) * (frame.height ? frame.height - 1 : 0) +
            frame.width;
        if (VB_GetDmaBufFd(reference_.handle(), &fd) != 0 || fd < 0 || fstat(fd, &info) != 0 ||
            info.st_size <= 0 || frame.version != 1 || frame.plane_num != 2 ||
            frame.pixel_format != MPP_PIXEL_FORMAT_NV12 ||
            frame.compress_mode != COMPRESS_MODE_NONE || !frame.width || !frame.height ||
            (frame.width & 1U) || (frame.height & 1U) || stride < frame.width ||
            extent > frame.plane_size[0] || extent > frame.plane_size_valid[0] ||
            extent > frame.total_size || VB_GetPhyAddr(reference_.handle(), &physical) != 0 ||
            frame.plane_phy_addr[0] != physical)
        {
            ++audit_->map_errors;
            throw std::runtime_error("Cannot validate localization DMA Y plane");
        }
        const auto sync = [this, fd](std::uint64_t flags, bool start)
        {
            if (start)
                ++audit_->cache_start_calls;
            else
                ++audit_->cache_end_calls;
            dma_buf_sync request{};
            request.flags = flags;
            int result;
            do
            {
                result = ioctl(fd, DMA_BUF_IOCTL_SYNC, &request);
            } while (result < 0 && errno == EINTR);
            if (result != 0)
            {
                if (start)
                    ++audit_->cache_start_errors;
                else
                    ++audit_->cache_end_errors;
            }
            return result;
        };
        if (sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ, true) != 0)
            throw std::runtime_error("Localization DMA READ START failed");
        std::vector<std::uint8_t> pixels;
        try
        {
            pixels = localization_copy_y(address(), frame.width, frame.height, stride,
                std::min<std::size_t>(info.st_size, frame.total_size));
        }
        catch (...)
        {
            (void)sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, false);
            throw;
        }
        if (sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ, false) != 0)
            throw std::runtime_error("Localization DMA READ END failed");
        ++audit_->copies;
        return pixels;
    }

private:
    std::shared_ptr<DmaImportAudit> audit_;
    AuditedImportReference<UL, MppImportOps> reference_;
    void* address_{nullptr};
};

struct PendingStereo
{
    PendingStereo(Frame::ConstSharedPtr l, Frame::ConstSharedPtr r,
        const std::shared_ptr<DmaImportAudit>& audit, const rclcpp::Logger& logger)
        : left(std::move(l)), right(std::move(r)), left_buffer(left->token, audit, logger),
            right_buffer(right->token, audit, logger)
    {
    }
    Frame::ConstSharedPtr left;
    Frame::ConstSharedPtr right;
    ImportedBuffer left_buffer;
    ImportedBuffer right_buffer;
};

Eigen::Quaternionf quaternion_from_pose(const localization::Pose& pose)
{
    return Eigen::Quaternionf(pose.orientation_xyzw[3], pose.orientation_xyzw[0],
        pose.orientation_xyzw[1], pose.orientation_xyzw[2])
        .normalized();
}

Eigen::Vector3f vector_from_pose(const localization::Pose& pose)
{
    return Eigen::Vector3f(pose.translation_m[0], pose.translation_m[1], pose.translation_m[2]);
}

class RobotVioNode final : public rclcpp::Node
{
public:
    explicit RobotVioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("vio", options)
    {
        const auto manifest = declare_parameter<std::string>("replay_manifest", "");
        const auto ack_topic =
            declare_parameter<std::string>("replay_ack_topic", "/replay/vio/ack");
        const auto ready_topic =
            declare_parameter<std::string>("replay_ready_topic", "/replay/vio/ready");
        const auto imu_ack_topic =
            declare_parameter<std::string>("replay_imu_ack_topic", "/replay/vio/imu_ack");
        const auto pause_topic = declare_parameter<std::string>("replay_pause_topic", "");
        const double replay_timeout = declare_parameter<double>("replay_timeout_sec", 30.0);
        if (!manifest.empty())
        {
            if (!std::isfinite(replay_timeout) || replay_timeout <= 0.0)
                throw std::runtime_error("replay_timeout_sec must be finite and positive");
            replay_ack_pub_ = create_publisher<std_msgs::msg::String>(
                ack_topic, rclcpp::QoS(1).reliable().transient_local());
            replay_ready_pub_ = create_publisher<std_msgs::msg::String>(
                ready_topic, rclcpp::QoS(1).reliable().transient_local());
            replay_imu_ack_pub_ = create_publisher<std_msgs::msg::String>(
                imu_ack_topic, rclcpp::QoS(1).reliable().transient_local());
            try
            {
                std::ifstream input(manifest);
                replay_keys_ = read_replay_manifest(input);
            }
            catch (const std::exception& error)
            {
                publish_replay_error(error.what());
                throw;
            }
        }
        const auto record_file = declare_parameter<std::string>("frame_record_file", "");
        const auto dma_audit_file = declare_parameter<std::string>("dma_audit_file", "");
        if (!dma_audit_file.empty())
        {
            const int fd = open(dma_audit_file.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
            if (fd < 0)
                throw std::runtime_error("dma_audit_file must be fresh with an existing parent");
            auto* file = fdopen(fd, "w");
            if (!file)
            {
                close(fd);
                throw std::runtime_error("Cannot open reserved DMA audit file");
            }
            dma_audit_file_.reset(file);
        }
        const int record_capacity = declare_parameter<int>("frame_record_queue_depth", 4096);
        if (record_capacity < 1 || record_capacity > 1048576)
            throw std::runtime_error("invalid frame_record_queue_depth");
        const std::string left_topic =
            declare_parameter<std::string>("left_topic", "/camera/left/frame_desc");
        const std::string right_topic =
            declare_parameter<std::string>("right_topic", "/camera/right/frame_desc");
        const std::string imu_topic =
            declare_parameter<std::string>("imu_topic", "/camera/imu/data_raw");
        const std::string calibration_file = declare_parameter<std::string>("calibration_file", "");
        const std::string imu_calibration_file =
            declare_parameter<std::string>("imu_calibration_file", "");
        world_frame_ = declare_parameter<std::string>("world_frame", "vio_world");
        rig_frame_ = declare_parameter<std::string>("rig_frame", "camera_rig");
        image_pose_topic_ = declare_parameter<std::string>("image_pose_topic", "/vio/pose/image");
        imu_pose_topic_ = declare_parameter<std::string>("imu_pose_topic", "/vio/pose/imu");
        gravity_topic_ = declare_parameter<std::string>("gravity_topic", "/vio/gravity");
        publish_static_gravity_fallback_ =
            declare_parameter<bool>("publish_static_gravity_fallback", true);
        gravity_static_min_samples_ = declare_parameter<int>("gravity_static_min_samples", 120);
        gravity_accel_tolerance_m_s2_ =
            declare_parameter<double>("gravity_accel_tolerance_m_s2", 0.75);
        gravity_gyro_threshold_rad_s_ =
            declare_parameter<double>("gravity_gyro_threshold_rad_s", 0.08);
        trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "/vio/trajectory");
        feature_topic_ = declare_parameter<std::string>("feature_topic", "/vio/features");
        publish_features_ = declare_parameter<bool>("publish_features", true);
        publish_frame_diagnostics_ = declare_parameter<bool>("publish_frame_diagnostics", false);
        if (publish_frame_diagnostics_ && !publish_features_)
            throw std::invalid_argument("publish_frame_diagnostics requires publish_features");
        pose_input_topic_ = declare_parameter<std::string>("pose_input_topic", "/vio/pose_inputs");
        publish_pose_inputs_ = declare_parameter<bool>("publish_pose_inputs", false);
        trajectory_max_length_ = declare_parameter<int>("trajectory_max_length", 2000);
        trajectory_publish_period_ms_ = declare_parameter<int>("trajectory_publish_period_ms", 100);
        visualization_publish_period_ms_ =
            declare_parameter<int>("visualization_publish_period_ms", 50);
        trajectory_line_width_ = declare_parameter<double>("trajectory_line_width", 0.012);
        camera_axis_length_ = declare_parameter<double>("camera_axis_length", 0.15);
        camera_axis_shaft_diameter_ =
            declare_parameter<double>("camera_axis_shaft_diameter", 0.012);
        const int sync_queue_depth = declare_parameter<int>("sync_queue_depth", 2);
        const int image_queue_depth = declare_parameter<int>("image_queue_depth", 1);
        const int image_width = declare_parameter<int>("image_width", 0);
        const int image_height = declare_parameter<int>("image_height", 0);
        use_imu_ = declare_parameter<bool>("use_imu", true);
        const double minimum_observation_depth_m =
            declare_parameter<double>("minimum_observation_depth_m", 0.0);
        minimum_observation_depth_m_ =
            validate_minimum_observation_depth(minimum_observation_depth_m);
        if (minimum_observation_depth_m_ > 0.0F)
        {
            RCLCPP_INFO(
                get_logger(), "Minimum observation depth: %.3fm", minimum_observation_depth_m);
        }
        const auto camera_config_file = declare_parameter<std::string>("camera_config_file", "");
        const int camera_fps = declare_parameter<int>("camera_fps", 0);
        // Keep the legacy standalone default; a supplied camera profile selects
        // its source-mode frequency unless a positive rate is explicitly given.
        double imu_frequency_hz =
            declare_parameter<double>("imu_frequency_hz", camera_config_file.empty() ? 600.0 : 0.0);
        if (camera_fps < 0)
        {
            throw std::runtime_error("camera_fps must be zero (JSON) or positive");
        }
        if (use_imu_ && imu_frequency_hz == 0.0)
        {
            if (camera_config_file.empty())
                throw std::runtime_error("automatic IMU frequency requires camera_config_file");
            imu_frequency_hz = camera_sdk::LoadImuFrequencyHz(camera_config_file, camera_fps);
        }
        enable_external_propagation_ = declare_parameter<bool>("enable_external_propagation", true);
        slam_enabled_ = declare_parameter<bool>("slam_enabled", false);
        slam_pose_topic_ = declare_parameter<std::string>("slam_pose_topic", "/slam/pose/image");
        slam_trajectory_topic_ =
            declare_parameter<std::string>("slam_trajectory_topic", "/slam/trajectory");
        slam_map_save_directory_ = declare_parameter<std::string>("slam_map_save_directory", "");
        slam_map_load_directory_ = declare_parameter<std::string>("slam_map_load_directory", "");
        slam_local_frame_ = declare_parameter<std::string>("slam_local_frame", "slam_world");
        slam_map_frame_ = declare_parameter<std::string>("slam_map_frame", "map");
        publish_map_to_odom_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", false);
        slam_save_timeout_sec_ = declare_parameter<double>("slam_save_timeout_sec", 60.0);
        const auto slam_record_file = declare_parameter<std::string>("slam_frame_record_file", "");
        const auto localization_audit_file =
            declare_parameter<std::string>("slam_localization_audit_file", "");
        const auto global_hint_topic =
            declare_parameter<std::string>("global_hint_topic", "/localization/global_hint");
        const auto slam_status_topic =
            declare_parameter<std::string>("slam_status_topic", "/localization/slam_status");
        const auto slam_ready_topic =
            declare_parameter<std::string>("slam_ready_topic", "/localization/slam_ready");
        const int history_capacity = declare_parameter<int>("localization_history_capacity", 256);
        const auto history_bytes =
            declare_parameter<std::int64_t>("localization_history_max_bytes", 268435456);
        const int retention_ms = declare_parameter<int>("slam_retention_time_ms", 10000);
        localization_max_age_ns_ = static_cast<std::int64_t>(retention_ms) * 1000000LL;
        const int maximum_pending_keyframes =
            declare_parameter<int>("slam_maximum_pending_keyframes", 8);
        const int maximum_tracked_poses =
            declare_parameter<int>("slam_maximum_tracked_poses", 16384);
        const bool slam_sync_mode = declare_parameter<bool>("slam_sync_mode", false);
        const auto horizontal_radius = static_cast<float>(
            declare_parameter<double>("slam_localization_horizontal_radius", 0.5));
        const auto vertical_radius =
            static_cast<float>(declare_parameter<double>("slam_localization_vertical_radius", 0.5));
        localization_settings_.half_extent_m = {
            horizontal_radius, horizontal_radius, vertical_radius};
        if ((!slam_map_save_directory_.empty() || !slam_map_load_directory_.empty() ||
                !slam_record_file.empty() || publish_map_to_odom_tf_) &&
            !slam_enabled_)
            throw std::invalid_argument("SLAM map/recording/TF options require slam_enabled=true");
        if (!slam_map_save_directory_.empty() && !slam_map_load_directory_.empty())
            throw std::invalid_argument("Save and load use separate SLAM sessions");
        if (slam_enabled_ &&
            (slam_local_frame_.empty() || slam_map_frame_.empty() ||
                slam_map_frame_ == world_frame_ || slam_map_frame_ == rig_frame_ ||
                slam_map_frame_ == slam_local_frame_ || slam_local_frame_ == rig_frame_))
            throw std::invalid_argument("SLAM map/local/rig frames must be distinct");
        if (!std::isfinite(slam_save_timeout_sec_) || slam_save_timeout_sec_ <= 0.0 ||
            slam_save_timeout_sec_ > 3600.0 || history_capacity < 1 || history_capacity > 65536 ||
            history_bytes < 1 || history_bytes > 2147483648LL || retention_ms < 1 ||
            maximum_pending_keyframes < 1 || maximum_pending_keyframes > 1024 ||
            maximum_tracked_poses < 1 || maximum_tracked_poses > 1048576)
            throw std::invalid_argument("Invalid SLAM timeout/history/retention limits");
        for (float value : localization_settings_.half_extent_m)
            if (!std::isfinite(value) || value <= 0.0f)
                throw std::invalid_argument(
                    "SLAM localization search settings must be finite positive");

        const auto capabilities = localization::Session::capabilities();
        const auto supports = [capabilities](localization::Capability capability)
        { return (capabilities & static_cast<std::uint64_t>(capability)) != 0; };
        if (!supports(localization::Capability::Stereo) ||
            (use_imu_ && !supports(localization::Capability::Inertial)) ||
            (slam_enabled_ && !supports(localization::Capability::Mapping)))
            throw std::runtime_error(
                "The selected VSLAM library lacks a required sensor or mapping capability");
        if (!slam_map_save_directory_.empty() &&
            !supports(localization::Capability::MapPersistence))
            throw std::invalid_argument(
                "slam_map_save_directory requires a VSLAM backend with map persistence");
        if (!slam_map_load_directory_.empty() &&
            !supports(localization::Capability::Relocalization))
            throw std::invalid_argument(
                "slam_map_load_directory requires a VSLAM backend with saved-map relocalization");

        if (!slam_map_save_directory_.empty() &&
            (std::filesystem::exists(slam_map_save_directory_) ||
                !std::filesystem::is_directory(
                    std::filesystem::path(slam_map_save_directory_).parent_path())))
            throw std::invalid_argument(
                "SLAM save directory must be fresh with an existing parent");
        if (!slam_map_load_directory_.empty() &&
            !std::filesystem::is_directory(slam_map_load_directory_))
            throw std::invalid_argument("SLAM load directory does not exist");
        if (!slam_map_load_directory_.empty())
            localization_history_ =
                std::make_unique<LocalizationHistory>(static_cast<std::size_t>(history_capacity),
                    static_cast<std::size_t>(history_bytes));
        if (!localization_audit_file.empty())
        {
            if (std::filesystem::exists(localization_audit_file))
                throw std::invalid_argument("SLAM localization audit file must be fresh");
            localization_audit_.open(localization_audit_file);
            localization_audit_ << "event,timestamp_ns,age_ns,elapsed_ms,success,detail\n";
            if (!localization_audit_)
                throw std::runtime_error("Cannot open SLAM audit CSV");
        }
        const int spacemit_vslam_verbosity = declare_parameter<int>("spacemit_vslam_verbosity", 0);
        if (calibration_file.empty())
        {
            throw std::runtime_error("calibration_file parameter is required");
        }
        if (sync_queue_depth < 2 || sync_queue_depth > 64)
        {
            throw std::runtime_error("sync_queue_depth must be in [2, 64]");
        }
        if (image_queue_depth < 1 || image_queue_depth > 64)
        {
            throw std::runtime_error("image_queue_depth must be in [1, 64]");
        }
        if (use_imu_ && (!(imu_frequency_hz > 0.0) || imu_frequency_hz > 5000.0))
        {
            throw std::runtime_error("imu_frequency_hz must be in (0, 5000]");
        }
        if (gravity_static_min_samples_ < 1 || gravity_static_min_samples_ > 10000 ||
            !(gravity_accel_tolerance_m_s2_ > 0.0) || !(gravity_gyro_threshold_rad_s_ > 0.0))
        {
            throw std::runtime_error("invalid static gravity fallback parameters");
        }
        if ((image_width == 0) != (image_height == 0) || image_width < 0 || image_height < 0)
        {
            throw std::runtime_error("image_width and image_height must both be zero or positive");
        }
        if (trajectory_max_length_ < 2 || trajectory_max_length_ > 100000)
        {
            throw std::runtime_error("trajectory_max_length must be in [2, 100000]");
        }
        if (trajectory_publish_period_ms_ < 0 || trajectory_publish_period_ms_ > 10000)
        {
            throw std::runtime_error("trajectory_publish_period_ms must be in [0, 10000]");
        }
        if (visualization_publish_period_ms_ < 0 || visualization_publish_period_ms_ > 10000 ||
            !(trajectory_line_width_ > 0.0) || !(camera_axis_length_ > 0.0) ||
            !(camera_axis_shaft_diameter_ > 0.0))
        {
            throw std::runtime_error("invalid visualization parameters");
        }

        const LoadedRig loaded = LoadRigCalibration(calibration_file, use_imu_, image_width,
            image_height, static_cast<float>(imu_frequency_hz), imu_calibration_file);
        imu_time_alignment_ = loaded.time_alignment;
        if (use_imu_)
        {
            RCLCPP_INFO(get_logger(),
                "Kalibr timing: cam0.timeshift_cam_imu=%" PRId64
                "ns (%s); VIO IMU timestamp = raw IMU timestamp - shift; image unchanged",
                imu_time_alignment_.shift_ns(),
                imu_time_alignment_.calibrated() ? "calibrated" : "absent, zero");
            const localization::InertialCalibration& imu = loaded.rig.inertial_sensors.front();
            RCLCPP_INFO(get_logger(),
                "IMU calibration: source=%s gyro_noise=%.9g gyro_walk=%.9g "
                "accel_noise=%.9g accel_walk=%.9g runtime_rate=%.1fHz",
                loaded.imu_calibration_source.c_str(), imu.angular_noise.white_noise_density,
                imu.angular_noise.bias_random_walk, imu.linear_noise.white_noise_density,
                imu.linear_noise.bias_random_walk, imu_frequency_hz);
            if (loaded.imu_yaml_update_rate_hz > 0.0F &&
                std::abs(loaded.imu_yaml_update_rate_hz - imu_frequency_hz) > 1.0)
            {
                RCLCPP_WARN(get_logger(),
                    "IMU YAML update_rate=%.1fHz differs from runtime_rate=%.1fHz; "
                    "using runtime_rate because it describes the published stream",
                    loaded.imu_yaml_update_rate_hz, imu_frequency_hz);
            }
        }
        rig_from_imu_ = loaded.rig_from_imu;
        expected_image_width_ = loaded.rig.cameras[0].intrinsics.image.width_px;
        expected_image_height_ = loaded.rig.cameras[0].intrinsics.image.height_px;
        if (!supports(localization::Capability::Features))
            publish_features_ = publish_pose_inputs_ = publish_frame_diagnostics_ = false;
        else if (!supports(localization::Capability::Quality))
            publish_frame_diagnostics_ = false;
        if (!supports(localization::Capability::PoseInputs))
            publish_pose_inputs_ = false;
        localization::Options session_configuration;
        session_configuration.input_mode =
            use_imu_ ? localization::InputMode::StereoInertial : localization::InputMode::Stereo;
        session_configuration.verbosity = spacemit_vslam_verbosity;
        session_configuration.asynchronous_processing =
            declare_parameter<bool>("asynchronous_processing", true);
        session_configuration.allow_stationary_initialization =
            declare_parameter<bool>("enable_stationary_initialization", false);
        session_configuration.export_features = publish_features_;
        session_configuration.export_pose_inputs = publish_pose_inputs_;
        session_configuration.export_quality = publish_frame_diagnostics_;
        session_configuration.minimum_depth_m = minimum_observation_depth_m_;
        session_configuration.mapping.enabled = slam_enabled_;
        session_configuration.mapping.asynchronous = !slam_sync_mode;
        session_configuration.mapping.observation_retention_ms =
            static_cast<std::uint32_t>(retention_ms);
        session_configuration.mapping.pending_update_limit =
            static_cast<std::uint32_t>(maximum_pending_keyframes);
        session_configuration.mapping.trajectory_limit =
            static_cast<std::uint32_t>(maximum_tracked_poses);
        rig_config_ = loaded.rig;
        session_options_ = session_configuration;
        const bool start_enabled = declare_parameter<bool>("start_enabled", true);
        if (start_enabled)
            session_ = std::make_unique<localization::Session>(loaded.rig, session_configuration);

        if (!record_file.empty())
        {
            frame_recorder_ = std::make_unique<FrameRecorder>(
                record_file, static_cast<std::size_t>(record_capacity), world_frame_, rig_frame_);
            RCLCPP_INFO(get_logger(), "VIO frame recording: %s", record_file.c_str());
        }
        if (!slam_record_file.empty())
            slam_frame_recorder_ = std::make_unique<FrameRecorder>(slam_record_file,
                static_cast<std::size_t>(record_capacity), "dynamic-see-slam-localization-audit",
                rig_frame_);
        CreateOutputPublishers(slam_status_topic, slam_ready_topic);

        CreateInputSubscriptions(left_topic, right_topic, imu_topic, pause_topic, global_hint_topic,
            sync_queue_depth, image_queue_depth);

        InitializeMpp();

        enabled_.store(start_enabled);
        publish_tracking_mode(start_enabled ? "WAIT" : "STOPPED");
        if (slam_enabled_)
            publish_slam_status("WAITING", "native SLAM session ready");
        CreateEnableService();

        RCLCPP_INFO(get_logger(),
            "VIO ready: left=%s right=%s imu=%s image_pose=%s imu_pose=%s "
            "trajectory=%s features=%s pose_inputs=%s mode=%s image=%dx%d "
            "queues=input:%d sync:%d "
            "imu_rate=%.1fHz map=%s map_async=%s",
            left_topic.c_str(), right_topic.c_str(), imu_topic.c_str(), image_pose_topic_.c_str(),
            imu_pose_topic_.c_str(), trajectory_topic_.c_str(),
            publish_features_ ? feature_topic_.c_str() : "disabled",
            publish_pose_inputs_ ? pose_input_topic_.c_str() : "disabled",
            use_imu_ ? "stereo-inertial" : "stereo", expected_image_width_, expected_image_height_,
            image_queue_depth, sync_queue_depth, use_imu_ ? imu_frequency_hz : 0.0,
            slam_enabled_ ? "on" : "off",
            slam_enabled_ && session_configuration.mapping.asynchronous ? "on" : "off");
        StartReplayWatchdog(replay_timeout);

        // Last constructor operation: no later initialization failure can leave
        // a joinable discovery thread during construction unwinding.
        if (slam_ready_pub_)
            start_slam_readiness();
    }

    ~RobotVioNode() override
    {
        enabled_.store(false);
        try
        {
            publish_slam_ready(false);
        }
        catch (const std::exception& error)
        {
            RCLCPP_DEBUG(get_logger(), "Native SLAM readiness shutdown publish: %s", error.what());
        }
        slam_graph_stop_.store(true);
        if (slam_graph_)
        {
            try
            {
                // Sets registered events under the graph mutex before waking
                // the CV, so stop cannot race the wait's predicate-to-block gap.
                slam_graph_->notify_graph_change();
            }
            catch (const std::exception& error)
            {
                // Humble notifies the CV before triggering the node guard;
                // a stopped-context guard failure does not lose the wakeup.
                RCLCPP_DEBUG(get_logger(), "Native SLAM graph stop notification: %s", error.what());
            }
        }
        if (slam_graph_thread_.joinable())
            slam_graph_thread_.join();
        if (slam_readiness_event_)
            get_node_waitables_interface()->remove_waitable(
                slam_readiness_event_, stereo_callback_group_);
        slam_readiness_event_.reset();
        localization_request_->cancel();
        if (localization_event_)
            get_node_waitables_interface()->remove_waitable(
                localization_event_, stereo_callback_group_);
        localization_event_.reset();
        get_node_waitables_interface()->remove_waitable(tracking_event_, stereo_callback_group_);
        tracking_event_.reset();
        if (replay_ack_pub_ && !replay_failed_ && replay_position_ != replay_keys_.size())
            RCLCPP_ERROR(get_logger(), "Replay incomplete: consumed %zu of %zu targets",
                replay_position_, replay_keys_.size());
        pending_stereo_.reset(); // Release imported DMA buffers before VB_Exit.
        // Input is stopped. Persist the map and its owned trajectory before
        // destroying the session, which waits for accepted task callbacks.
        finalize_slam_map();
        frame_recorder_.reset(); // Drain records after tracking has stopped.
        slam_frame_recorder_.reset();
        session_.reset();
        int vb_exit_result = 0, sys_exit_result = 0;
        if (vb_initialized_)
        {
            vb_exit_result = VB_Exit();
            vb_initialized_ = false;
        }
        if (sys_initialized_)
        {
            sys_exit_result = SYS_Exit();
            sys_initialized_ = false;
        }
        write_dma_audit(vb_exit_result, sys_exit_result);
    }

private:
    void CreateOutputPublishers(
        const std::string& slam_status_topic, const std::string& slam_ready_topic)
    {
        image_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            image_pose_topic_, rclcpp::SensorDataQoS().keep_last(32));
        tracking_mode_pub_ = create_publisher<std_msgs::msg::String>(
            declare_parameter<std::string>("tracking_mode_topic", "/vio/tracking_mode"),
            rclcpp::SensorDataQoS().keep_last(1));
        imu_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            imu_pose_topic_, rclcpp::SensorDataQoS().keep_last(64));
        gravity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            gravity_topic_, rclcpp::SensorDataQoS().keep_last(8));
        trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
            trajectory_topic_, rclcpp::SensorDataQoS().keep_last(4));
        visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/vio/visualization", rclcpp::QoS(4));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        if (slam_enabled_)
        {
            slam_status_pub_ = create_publisher<std_msgs::msg::String>(
                slam_status_topic, rclcpp::QoS(1).reliable().transient_local());
            if (!slam_ready_topic.empty())
                slam_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
                    slam_ready_topic, rclcpp::QoS(1).reliable().transient_local());
            slam_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
                slam_pose_topic_, rclcpp::SensorDataQoS().keep_last(32));
            slam_trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
                slam_trajectory_topic_, rclcpp::SensorDataQoS().keep_last(4));
        }
        if (publish_features_)
        {
            feature_pub_ = create_publisher<common::msg::FeaturePointArray>(
                feature_topic_, rclcpp::SensorDataQoS().keep_last(4));
        }
        if (publish_pose_inputs_)
        {
            pose_input_pub_ = create_publisher<common::msg::FeaturePointArray>(
                pose_input_topic_, rclcpp::SensorDataQoS().keep_last(4));
        }
    }

    void CreateInputSubscriptions(const std::string& left_topic, const std::string& right_topic,
        const std::string& imu_topic, const std::string& pause_topic,
        const std::string& global_hint_topic, int sync_queue_depth, int image_queue_depth)
    {
        stereo_callback_group_ =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Preserve IMU callback order without serializing it with image tracking.
        imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        tracking_event_ = std::make_shared<ExecutorEvent>(
            get_node_base_interface()->get_context(), [this]() { process_ready_stereo(); });
        get_node_waitables_interface()->add_waitable(tracking_event_, stereo_callback_group_);
        if (!slam_map_load_directory_.empty())
        {
            localization_event_ =
                std::make_shared<ExecutorEvent>(get_node_base_interface()->get_context(),
                    [this]() { process_localization_completion(); });
            get_node_waitables_interface()->add_waitable(
                localization_event_, stereo_callback_group_);
        }
        rclcpp::SubscriptionOptions stereo_options;
        stereo_options.callback_group = stereo_callback_group_;
        if (localization_history_)
            global_hint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                global_hint_topic, rclcpp::QoS(1).reliable(),
                std::bind(&RobotVioNode::global_hint_callback, this, std::placeholders::_1),
                stereo_options);
        rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
        image_qos.depth = static_cast<std::size_t>(image_queue_depth);
        if (replay_ack_pub_)
        {
            const auto qos = rclcpp::QoS(1).reliable();
            replay_left_sub_ = create_subscription<Frame>(
                left_topic, qos, [this](Frame::ConstSharedPtr frame) { replay_image(frame, true); },
                stereo_options);
            replay_right_sub_ = create_subscription<Frame>(
                right_topic, qos, [this](Frame::ConstSharedPtr frame)
                { replay_image(frame, false); }, stereo_options);
            if (!pause_topic.empty())
            {
                replay_pause_sub_ = create_subscription<std_msgs::msg::Bool>(pause_topic,
                    rclcpp::QoS(1).reliable().transient_local(),
                    std::bind(&RobotVioNode::replay_pause_callback, this, std::placeholders::_1),
                    stereo_options);
            }
        }
        else
        {
            left_sub_.subscribe(this, left_topic, image_qos, stereo_options);
            right_sub_.subscribe(this, right_topic, image_qos, stereo_options);
            sync_ = std::make_unique<message_filters::Synchronizer<StereoSyncPolicy>>(
                StereoSyncPolicy(sync_queue_depth), left_sub_, right_sub_);
            sync_->registerCallback(std::bind(&RobotVioNode::stereo_callback, this,
                std::placeholders::_1, std::placeholders::_2));
        }
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic,
            replay_ack_pub_ ? rclcpp::QoS(2048).reliable()
                            : rclcpp::QoS(rclcpp::SensorDataQoS().keep_last(128)),
            std::bind(&RobotVioNode::imu_callback, this, std::placeholders::_1), imu_options);
    }

    void InitializeMpp()
    {
        // Allow up to five seconds per stage for MPP startup.
        constexpr int kMppInitAttempts = 50;
        constexpr auto kMppInitRetryDelay = std::chrono::milliseconds(100);
        const auto init_with_retry = [=](S32 (*initialize)())
        {
            S32 result = -1;
            for (int attempt = 0; attempt < kMppInitAttempts; ++attempt)
            {
                result = initialize();
                if (result == 0)
                {
                    break;
                }
                std::this_thread::sleep_for(kMppInitRetryDelay);
            }
            return result;
        };
        const S32 sys_result = init_with_retry(SYS_Init);
        if (sys_result != 0)
        {
            throw std::runtime_error("SYS_Init failed for VIO: " + std::to_string(sys_result));
        }
        sys_initialized_ = true;
        const S32 vb_result = init_with_retry(VB_Init);
        if (vb_result != 0)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
            throw std::runtime_error("VB_Init failed for VIO: " + std::to_string(vb_result));
        }
        vb_initialized_ = true;
    }

    void CreateEnableService()
    {
        enable_service_ = create_service<std_srvs::srv::SetBool>("~/set_enabled",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res)
            {
                if (req->data == enabled_.load())
                {
                    res->success = true;
                    res->message = req->data ? "already enabled" : "already stopped";
                    return;
                }
                if (req->data && slam_save_attempted_)
                {
                    res->success = false;
                    res->message = "SLAM save session cannot be restarted into the same directory";
                    return;
                }
                enabled_.store(false);
                // All image, tracking and IMU callbacks take shared ownership.
                // Wait for in-flight image and inertial submissions before destroying the session.
                std::unique_lock<std::shared_mutex> lock(session_mutex_);
                publish_slam_ready(false);
                pending_stereo_.reset();
                localization_request_->cancel();
                if (localization_history_)
                    localization_history_->clear();
                localized_ = false;
                finalize_slam_map();
                session_.reset();
                feature_diagnostics_.reset();
                imu_history_.clear();
                pending_imu_.clear();
                propagator_ = ImuPropagator{};
                trajectory_poses_.clear();
                slam_trajectory_poses_.clear();
                loop_closure_timestamps_.clear();
                static_acceleration_sum_.setZero();
                static_acceleration_.setZero();
                static_gravity_samples_ = 0;
                static_gravity_ready_ = false;
                last_imu_timestamp_ = last_registered_imu_timestamp_ = 0;
                last_image_timestamp_ns_ = 0;
                tracked_frames_ = imu_samples_ = propagated_samples_ = 0;
                last_image_frame_index_ = 0;
                trajectory_published_ = slam_trajectory_published_ = visualization_published_ =
                    false;
                try
                {
                    if (req->data)
                    {
                        session_ =
                            std::make_unique<localization::Session>(rig_config_, session_options_);
                        session_start_ns_ = now().nanoseconds();
                        enabled_.store(true);
                    }
                    res->success = true;
                    res->message = req->data ? "fresh VIO session enabled" : "VIO session released";
                }
                catch (const std::exception& e)
                {
                    session_.reset();
                    res->success = false;
                    res->message = e.what();
                }
                RCLCPP_INFO(get_logger(), "set_enabled=%s: %s", req->data ? "true" : "false",
                    res->message.c_str());
                publish_tracking_mode(res->success ? (req->data ? "WAIT" : "STOPPED") : "ERROR");
                if (slam_enabled_)
                    publish_slam_status("WAITING", req->data ? "fresh session" : "session stopped");
                if (slam_readiness_event_)
                    slam_readiness_event_->trigger();
            });
    }

    void StartReplayWatchdog(double replay_timeout_sec)
    {
        if (replay_ack_pub_)
        {
            replay_deadline_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(replay_timeout_sec));
            replay_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(replay_timeout_sec));
            replay_watchdog_ = create_wall_timer(
                std::chrono::seconds(1),
                [this]()
                {
                    if (!replay_failed_ && replay_imu_overflow_.load())
                    {
                        publish_replay_error(
                            "pending IMU capacity exceeded; pace replay IMU delivery");
                        return;
                    }
                    if (!replay_failed_ && !replay_paused_ &&
                        replay_position_ < replay_keys_.size() &&
                        std::chrono::steady_clock::now() >= replay_deadline_)
                        publish_replay_error("timeout waiting for target/IMU " +
                            replay_keys_[replay_position_].text());
                },
                stereo_callback_group_);
            std_msgs::msg::String ready;
            ready.data = "ready";
            replay_ready_pub_->publish(ready);
        }
    }

    void write_dma_audit(int vb_exit_result, int sys_exit_result) noexcept
    {
        const auto& audit = *dma_import_audit_;
        const bool clean = audit.imports.load() == audit.release_calls.load() &&
            audit.import_failures.load() == 0 && audit.release_failures.load() == 0 &&
            audit.map_errors.load() == 0 && audit.cache_start_errors.load() == 0 &&
            audit.cache_end_errors.load() == 0 &&
            audit.cache_start_calls.load() == audit.cache_end_calls.load() &&
            audit.copies.load() == audit.cache_end_calls.load() && vb_exit_result == 0 &&
            sys_exit_result == 0;
        RCLCPP_INFO(get_logger(),
            "VIO DMA summary imports=%" PRIu64 " import_failures=%" PRIu64 " release_calls=%" PRIu64
            " release_failures=%" PRIu64 " map_errors=%" PRIu64 " cache_start_calls=%" PRIu64
            " cache_start_errors=%" PRIu64 " cache_end_calls=%" PRIu64 " cache_end_errors=%" PRIu64
            " copies=%" PRIu64 " vb_exit_result=%d sys_exit_result=%d clean=%s",
            audit.imports.load(), audit.import_failures.load(), audit.release_calls.load(),
            audit.release_failures.load(), audit.map_errors.load(), audit.cache_start_calls.load(),
            audit.cache_start_errors.load(), audit.cache_end_calls.load(),
            audit.cache_end_errors.load(), audit.copies.load(), vb_exit_result, sys_exit_result,
            clean ? "true" : "false");
        if (!clean)
            RCLCPP_ERROR(get_logger(), "VIO DMA audit failed; inspect ownership/cache/SDK counts");
        if (!dma_audit_file_)
            return;
        try
        {
            std::ostringstream output;
            output << "{\"schema\":\"SPACEMIT_ROBOT_VIO_DMA_AUDIT_V1\"";
            const auto field = [&](const char* name, std::uint64_t value)
            { output << ",\"" << name << "\":" << value; };
            field("imports", audit.imports.load());
            field("import_failures", audit.import_failures.load());
            field("release_calls", audit.release_calls.load());
            field("release_failures", audit.release_failures.load());
            field("map_errors", audit.map_errors.load());
            field("cache_start_calls", audit.cache_start_calls.load());
            field("cache_start_errors", audit.cache_start_errors.load());
            field("cache_end_calls", audit.cache_end_calls.load());
            field("cache_end_errors", audit.cache_end_errors.load());
            field("copies", audit.copies.load());
            output << ",\"vb_exit_result\":" << vb_exit_result
                    << ",\"sys_exit_result\":" << sys_exit_result
                    << ",\"clean\":" << (clean ? "true" : "false") << "}\n";
            const auto bytes = output.str();
            if (std::fwrite(bytes.data(), 1, bytes.size(), dma_audit_file_.get()) != bytes.size() ||
                std::fflush(dma_audit_file_.get()) != 0)
                throw std::runtime_error("Cannot write VIO DMA audit JSON");
            if (std::fclose(dma_audit_file_.release()) != 0)
                throw std::runtime_error("Cannot close VIO DMA audit JSON");
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "%s", error.what());
        }
    }

    void start_slam_readiness()
    {
        publish_slam_ready(false);
        slam_graph_ = get_node_graph_interface();
        slam_graph_event_ = slam_graph_->get_graph_event();
        slam_readiness_event_ = std::make_shared<ExecutorEvent>(
            get_node_base_interface()->get_context(), [this]() { check_slam_readiness(); });
        get_node_waitables_interface()->add_waitable(slam_readiness_event_, stereo_callback_group_);
        slam_graph_thread_ = std::thread(
            [this]()
            {
                try
                {
                    slam_readiness_event_->trigger();
                    while (!slam_graph_stop_.load() && rclcpp::ok())
                    {
                        // Humble forwards this to condition_variable::wait_for.
                        // nanoseconds::max() overflows its now()+timeout deadline
                        // and can return immediately. A safe long wait re-arms
                        // on timeout; only a real graph event schedules readiness.
                        slam_graph_->wait_for_graph_change(
                            slam_graph_event_, std::chrono::hours(24));
                        if (slam_graph_stop_.load() || !rclcpp::ok())
                            break;
                        if (slam_graph_event_->check_and_clear())
                            slam_readiness_event_->trigger();
                    }
                }
                catch (const std::exception& error)
                {
                    if (!slam_graph_stop_.load() && rclcpp::ok())
                        RCLCPP_ERROR(get_logger(), "Native SLAM readiness graph wait failed: %s",
                            error.what());
                }
            });
    }

    void publish_slam_ready(bool value)
    {
        if (!slam_ready_pub_)
            return;
        slam_ready_value_ = value;
        std_msgs::msg::Bool ready;
        ready.data = value;
        slam_ready_pub_->publish(ready);
    }

    void check_slam_readiness()
    {
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        const auto left = replay_ack_pub_ ? replay_left_sub_ : left_sub_.getSubscriber();
        const auto right = replay_ack_pub_ ? replay_right_sub_ : right_sub_.getSubscriber();
        const bool ready = enabled_.load() && session_ && slam_enabled_ && sys_initialized_ &&
            vb_initialized_ && left && right && left->get_publisher_count() > 0 &&
            right->get_publisher_count() > 0;
        if (ready != slam_ready_value_)
        {
            publish_slam_ready(ready);
            RCLCPP_INFO(
                get_logger(), "Native SLAM matched-input readiness=%s", ready ? "true" : "false");
        }
    }
    static Eigen::Isometry3d localization_pose(const localization::Pose& pose)
    {
        return vio::localization_pose(pose.translation_m[0], pose.translation_m[1],
            pose.translation_m[2], pose.orientation_xyzw[0], pose.orientation_xyzw[1],
            pose.orientation_xyzw[2], pose.orientation_xyzw[3]);
    }

    static localization::Pose native_pose(const Eigen::Isometry3d& pose)
    {
        localization::Pose result;
        const Eigen::Quaterniond q(pose.linear());
        result.translation_m = {static_cast<float>(pose.translation().x()),
            static_cast<float>(pose.translation().y()), static_cast<float>(pose.translation().z())};
        result.orientation_xyzw = {static_cast<float>(q.x()), static_cast<float>(q.y()),
            static_cast<float>(q.z()), static_cast<float>(q.w())};
        for (float value : result.translation_m)
            if (!std::isfinite(value))
                throw std::invalid_argument("Global hint translation exceeds native float range");
        return result;
    }

    static std::string operation_detail(
        localization::OperationStatus status, const std::string& detail)
    {
        if (!detail.empty())
            return detail;
        switch (status)
        {
        case localization::OperationStatus::Succeeded:
            return "Map operation completed";
        case localization::OperationStatus::NotFound:
            return "Requested map data or pose was not found";
        case localization::OperationStatus::Unsupported:
            return "The selected library does not support this map request";
        case localization::OperationStatus::StaleObservation:
            return "Observation snapshot no longer has a usable tracking reference";
        case localization::OperationStatus::Failed:
            return "Map operation failed";
        }
        return "Map operation returned an unknown status";
    }

    static std::string json_quote(const std::string& value)
    {
        std::ostringstream result;
        result << '"';
        for (unsigned char c : value)
            if (c == '"' || c == '\\')
                result << '\\' << static_cast<char>(c);
            else if (c < 0x20)
                result << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<unsigned>(c) << std::dec;
            else
                result << static_cast<char>(c);
        result << '"';
        return result.str();
    }

    void publish_slam_status(const std::string& state, const std::string& detail,
        std::int64_t timestamp_ns = 0, double elapsed_ms = 0.0)
    {
        if (slam_status_pub_)
        {
            std_msgs::msg::String message;
            message.data = state + ": " + detail + " timestamp_ns=" + std::to_string(timestamp_ns);
            slam_status_pub_->publish(message);
        }
        RCLCPP_INFO(get_logger(),
            "slam_localization %s: %s timestamp_ns=%" PRId64 " elapsed_ms=%.3f", state.c_str(),
            detail.c_str(), timestamp_ns, elapsed_ms);
        if (localization_audit_.is_open())
        {
            std::string escaped = detail;
            for (std::size_t at = 0; (at = escaped.find('"', at)) != std::string::npos; at += 2)
                escaped.insert(at, 1, '"');
            const auto age = timestamp_ns > 0 ? last_image_timestamp_ns_ - timestamp_ns : 0;
            localization_audit_ << state << ',' << timestamp_ns << ',' << age << ','
                                << std::setprecision(17) << elapsed_ms << ','
                                << (state == "LOCALIZED") << ",\"" << escaped << "\"\n";
            localization_audit_.flush();
            if (!localization_audit_)
                RCLCPP_ERROR(get_logger(), "SLAM localization audit write failed");
        }
    }

    void global_hint_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr hint)
    {
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        if (!enabled_.load() || !session_ || !slam_enabled_ || localized_ ||
            localization_request_->busy())
            return;
        const auto timestamp_ns = static_cast<std::int64_t>(hint->header.stamp.sec) * 1000000000LL +
            hint->header.stamp.nanosec;
        try
        {
            if (hint->header.frame_id != slam_map_frame_)
                throw std::invalid_argument("Global hint frame does not match slam_map_frame");
            const auto& p = hint->pose.position;
            const auto& q = hint->pose.orientation;
            const auto guess = vio::localization_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
            const auto* frame = localization_history_->find(timestamp_ns);
            if (!frame)
                throw std::invalid_argument(
                    "Exact original hint timestamp is absent from bounded history");
            const auto age = last_image_timestamp_ns_ - timestamp_ns;
            if (age < 0 || age > localization_max_age_ns_)
                throw std::invalid_argument(
                    "Global hint timestamp exceeds the image history age limit");
            std::vector<localization::InputImage> images;
            for (std::uint32_t eye = 0; eye < 2; ++eye)
            {
                localization::InputImage image;
                image.pixels = eye ? frame->right.data() : frame->left.data();
                image.width = frame->width;
                image.height = frame->height;
                image.pitch = frame->width;
                image.timestamp_ns = timestamp_ns;
                image.camera_index = eye;
                images.push_back(image);
            }
            const auto begin = std::chrono::steady_clock::now();
            localization::RelocalizationRequest job;
            job.map.location = slam_map_load_directory_;
            job.observation = session_->capture_observation(images);
            if (job.observation.timestamp_ns() != timestamp_ns)
                throw std::runtime_error("Observation snapshot does not match the hint timestamp");
            job.prior = localization::PosePrior{native_pose(guess)};
            job.search = localization_settings_;
            const auto generation = localization_request_->begin(timestamp_ns);
            if (!generation)
                return;
            const auto request = localization_request_;
            const std::weak_ptr<ExecutorEvent> event = localization_event_;
            try
            {
                publish_slam_status("LOCALIZING", "saved-map localization queued", timestamp_ns);
                session_->relocalize(job,
                    [request, event, generation = *generation, begin, timestamp_ns](
                        localization::LocalizationOutcome result)
                    {
                        LocalizationCompletion completion;
                        completion.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - begin)
                                                    .count();
                        completion.detail = operation_detail(result.status, result.error);
                        try
                        {
                            if (result.status == localization::OperationStatus::Succeeded)
                            {
                                if (result.source_timestamp_ns != timestamp_ns)
                                    throw std::runtime_error(
                                        "Localization result does not match the "
                                        "observation timestamp");
                                if (!result.map_from_tracking)
                                    throw std::runtime_error(
                                        "Localization result has no tracking-to-map alignment");
                                completion.map_from_tracking =
                                    RobotVioNode::localization_pose(*result.map_from_tracking);
                                completion.success = true;
                                completion.detail = "saved-map localization succeeded";
                            }
                        }
                        catch (const std::exception& error)
                        {
                            completion.detail = error.what();
                        }
                        if (request->complete(generation, std::move(completion)))
                            if (auto ready = event.lock())
                                try
                                {
                                    ready->trigger();
                                }
                                catch (const std::exception&)
                                {
                                }
                    });
                const double preparation_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin)
                                                    .count();
                RCLCPP_INFO(get_logger(),
                    "localization_prepare timestamp_ns=%" PRId64 " elapsed=%.3fms",
                    static_cast<std::int64_t>(timestamp_ns), preparation_ms);
            }
            catch (...)
            {
                localization_request_->cancel();
                throw;
            }
        }
        catch (const std::exception& error)
        {
            publish_slam_status("FAILED", error.what(), timestamp_ns);
        }
    }

    void process_localization_completion()
    {
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        const auto completion = localization_request_->take();
        if (!completion || !enabled_.load() || !session_ || !slam_enabled_)
            return;
        if (completion->success)
        {
            map_from_odom_ = completion->map_from_tracking;
            localized_ = true;
            // Existing local-world history must never be retagged as map poses.
            slam_trajectory_poses_.clear();
            slam_trajectory_published_ = false;
            localization_history_->clear();
            if (publish_map_to_odom_tf_)
                publish_map_transform(last_image_timestamp_ns_);
        }
        publish_slam_status(completion->success ? "LOCALIZED" : "FAILED", completion->detail,
            completion->timestamp_ns, completion->elapsed_ms);
    }

    void publish_map_transform(std::int64_t timestamp_ns)
    {
        if (!localized_ || timestamp_ns <= 0)
            return;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = rclcpp::Time(timestamp_ns);
        transform.header.frame_id = slam_map_frame_;
        transform.child_frame_id = world_frame_;
        transform.transform.translation.x = map_from_odom_.translation().x();
        transform.transform.translation.y = map_from_odom_.translation().y();
        transform.transform.translation.z = map_from_odom_.translation().z();
        const Eigen::Quaterniond q(map_from_odom_.linear());
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform);
    }

    void finalize_slam_map() noexcept
    {
        if (slam_map_save_directory_.empty() || slam_save_attempted_ || !session_ || !slam_enabled_)
            return;
        slam_save_attempted_ = true;
        struct SaveState
        {
            std::mutex mutex;
            std::condition_variable ready;
            bool done{}, success{}, canceled{};
            std::string detail;
            std::vector<localization::TimedPose> poses;
        };
        const auto state = std::make_shared<SaveState>();
        bool owns_directory = false;
        try
        {
            if (!std::filesystem::create_directory(slam_map_save_directory_))
                throw std::runtime_error("SLAM save directory is no longer fresh");
            owns_directory = true;
            if (tracked_frames_ == 0)
                throw std::runtime_error("Cannot save an empty SLAM session");
            session_->save_map(localization::MapExportRequest{slam_map_save_directory_},
                [state](localization::SaveOutcome result)
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->canceled || state->done)
                        return;
                    state->success = result.status == localization::OperationStatus::Succeeded;
                    state->detail = operation_detail(result.status, result.error);
                    if (state->success)
                    {
                        state->poses = std::move(result.trajectory);
                        if (state->poses.empty())
                        {
                            state->success = false;
                            state->detail = "Persisted map trajectory snapshot is empty";
                        }
                        else
                            state->detail = "persisted map and trajectory snapshot";
                    }
                    state->done = true;
                    state->ready.notify_one();
                });
            std::unique_lock<std::mutex> lock(state->mutex);
            if (!state->ready.wait_for(lock, std::chrono::duration<double>(slam_save_timeout_sec_),
                    [&]() { return state->done; }))
            {
                state->canceled = true;
                throw std::runtime_error("Timed out waiting for map export completion");
            }
            if (!state->success)
                throw std::runtime_error(state->detail);
            std::ofstream output(
                std::filesystem::path(slam_map_save_directory_) / "slam_poses.csv");
            output << "timestamp_ns,tx,ty,tz,qx,qy,qz,qw\n" << std::setprecision(17);
            std::int64_t previous = 0;
            for (const auto& stamped : state->poses)
            {
                if (stamped.timestamp_ns <= previous)
                    throw std::runtime_error("SLAM snapshot timestamps must strictly increase");
                (void)localization_pose(stamped.pose);
                previous = stamped.timestamp_ns;
                output << stamped.timestamp_ns;
                for (auto value : stamped.pose.translation_m)
                    output << ',' << value;
                for (auto value : stamped.pose.orientation_xyzw)
                    output << ',' << value;
                output << '\n';
            }
            output.close();
            if (!output)
                throw std::runtime_error("Cannot write SLAM pose snapshot");
            RCLCPP_INFO(get_logger(), "SLAM map saved: %s poses=%zu",
                slam_map_save_directory_.c_str(), state->poses.size());
        }
        catch (const std::exception& error)
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->success = false;
            state->detail = error.what();
            RCLCPP_ERROR(get_logger(), "SLAM map save FAILED: %s", error.what());
        }
        try
        {
            if (!owns_directory)
                return; // Do not write into an unrelated newly created directory.
            // JSON is emitted explicitly; YAML::Dump is not JSON serialization.
            std::ofstream output(
                std::filesystem::path(slam_map_save_directory_) / "slam_save_summary.json");
            output << "{\"success\":" << (state->success ? "true" : "false")
                    << ",\"pose_count\":" << state->poses.size()
                    << ",\"tracked_frames\":" << tracked_frames_ << ",\"complete_pose_history\":"
                    << (state->poses.size() == tracked_frames_ ? "true" : "false")
                    << ",\"world_frame\":" << json_quote(slam_map_frame_)
                    << ",\"detail\":" << json_quote(state->detail)
                    << ",\"pose_snapshot\":\"owned trajectory from completed map export\"}\n";
            output.close();
            if (!output)
                RCLCPP_ERROR(get_logger(), "Cannot write SLAM save summary");
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "SLAM save summary failed: %s", error.what());
        }
    }

    void publish_replay_error(const std::string& reason)
    {
        replay_failed_ = true;
        {
            std::lock_guard<std::mutex> lock(stereo_pending_mutex_);
            pending_stereo_.reset();
        }
        replay_left_.reset();
        replay_right_.reset();
        RCLCPP_ERROR(get_logger(), "Replay stopped: %s", reason.c_str());
        std_msgs::msg::String message;
        message.data = "error:" + reason;
        replay_ack_pub_->publish(message);
    }

    void replay_pause_callback(const std_msgs::msg::Bool::ConstSharedPtr& message)
    {
        if (replay_failed_)
            return;
        if (message->data)
        {
            bool pending;
            {
                std::lock_guard<std::mutex> lock(stereo_pending_mutex_);
                pending = static_cast<bool>(pending_stereo_);
            }
            if (pending || replay_pair_pending_ || replay_left_ || replay_right_)
            {
                // A pause applies only between acknowledged targets, never to missing IMU/eyes.
                replay_paused_ = false;
                RCLCPP_WARN(get_logger(), "Ignoring replay pause while a target is pending");
                return;
            }
            replay_paused_ = true;
            return;
        }
        replay_paused_ = false;
        replay_deadline_ = std::chrono::steady_clock::now() + replay_timeout_;
    }

    void replay_image(const Frame::ConstSharedPtr& frame, bool left)
    {
        if (replay_failed_)
            return;
        if (replay_paused_)
        {
            // Resume and image subscriptions can be selected in either order by the executor.
            replay_paused_ = false;
            replay_deadline_ = std::chrono::steady_clock::now() + replay_timeout_;
        }
        try
        {
            if (frame->header.stamp.sec < 0 || frame->header.stamp.nanosec >= 1000000000U)
                throw std::runtime_error("invalid replay descriptor timestamp");
            const ReplayKey key{frame->frame_index,
                static_cast<std::int64_t>(frame->header.stamp.sec) * 1000000000LL +
                    frame->header.stamp.nanosec};
            if (replay_position_ >= replay_keys_.size())
                throw std::runtime_error("unexpected descriptor after manifest end: " + key.text());
            if (!(key == replay_keys_[replay_position_]))
                throw std::runtime_error("unexpected/missing target: expected " +
                    replay_keys_[replay_position_].text() + " received " + key.text());
            auto& slot = left ? replay_left_ : replay_right_;
            if (slot || replay_pair_pending_)
                throw std::runtime_error("duplicate or overlapping target: " + key.text());
            slot = frame;
            if (replay_left_ && replay_right_)
            {
                replay_pair_pending_ = true;
                stereo_callback(replay_left_, replay_right_);
                replay_left_.reset();
                replay_right_.reset();
            }
        }
        catch (const std::exception& error)
        {
            publish_replay_error(error.what());
        }
        catch (...)
        {
            publish_replay_error("unknown image enqueue exception");
        }
    }

    void record_frame(const FrameRecord& record)
    {
        if (frame_recorder_ && !frame_recorder_->enqueue(record))
        {
            replay_record_failed_ = true;
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "VIO recording incomplete: dropped=%lu write_failed=%s",
                static_cast<unsigned long>(frame_recorder_->dropped()),
                frame_recorder_->failed() ? "true" : "false");
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr message)
    {
        if (!enabled_.load())
            return;
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        if (!enabled_.load())
            return;
        if (!use_imu_)
        {
            return;
        }
        const std::int64_t raw_timestamp_ns =
            static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(message->header.stamp.nanosec);
        // The session gate and replay receipt watermark remain in source time.
        if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U ||
            raw_timestamp_ns <= session_start_ns_)
        {
            return;
        }
        std::int64_t timestamp_ns;
        try
        {
            timestamp_ns = imu_time_alignment_.imu_to_camera(raw_timestamp_ns);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
            return;
        }

        localization::InertialSample measurement;
        measurement.timestamp_ns = timestamp_ns;
        measurement.acceleration_m_s2 = {static_cast<float>(message->linear_acceleration.x),
            static_cast<float>(message->linear_acceleration.y),
            static_cast<float>(message->linear_acceleration.z)};
        measurement.angular_velocity_rad_s = {static_cast<float>(message->angular_velocity.x),
            static_cast<float>(message->angular_velocity.y),
            static_cast<float>(message->angular_velocity.z)};

        ImuSample sample{timestamp_ns,
            Eigen::Vector3f(measurement.acceleration_m_s2[0], measurement.acceleration_m_s2[1],
                measurement.acceleration_m_s2[2]),
            Eigen::Vector3f(measurement.angular_velocity_rad_s[0],
                measurement.angular_velocity_rad_s[1], measurement.angular_velocity_rad_s[2])};
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (timestamp_ns <= last_imu_timestamp_)
            {
                return;
            }
            last_imu_timestamp_ = timestamp_ns;
            imu_history_.push_back(sample);
            pending_imu_.push_back(sample);
            const float acceleration_error = std::abs(sample.acceleration.norm() - kGravityMps2);
            if (publish_static_gravity_fallback_ &&
                acceleration_error <= gravity_accel_tolerance_m_s2_ &&
                sample.angular_velocity.norm() <= gravity_gyro_threshold_rad_s_)
            {
                if (static_gravity_samples_ < gravity_static_min_samples_)
                {
                    static_acceleration_sum_ += sample.acceleration;
                    ++static_gravity_samples_;
                }
            }
            else if (!static_gravity_ready_)
            {
                static_acceleration_sum_.setZero();
                static_gravity_samples_ = 0;
            }
            if (!static_gravity_ready_ && static_gravity_samples_ >= gravity_static_min_samples_)
            {
                static_acceleration_ =
                    static_acceleration_sum_ / static_cast<float>(static_gravity_samples_);
                static_gravity_ready_ = true;
            }
            while (imu_history_.size() > 2048)
            {
                imu_history_.pop_front();
            }
            while (pending_imu_.size() > 2048)
            {
                if (replay_ack_pub_)
                    replay_imu_overflow_.store(true);
                pending_imu_.pop_front();
            }
            ++imu_samples_;
        }

        PropagatedState state;
        {
            std::lock_guard<std::mutex> lock(propagation_mutex_);
            propagator_.integrate(sample);
            state = propagator_.state();
        }
        if (state.valid)
        {
            ++propagated_samples_;
            publish_propagated_pose(state);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "imu samples=%" PRIu64 " propagated=%" PRIu64, imu_samples_, propagated_samples_);
        wake_ready_stereo();
        if (replay_imu_ack_pub_)
        {
            std_msgs::msg::String ack;
            ack.data = std::to_string(raw_timestamp_ns);
            replay_imu_ack_pub_->publish(ack);
        }
    }

    bool stereo_ready_locked() const
    {
        if (!pending_stereo_)
        {
            return false;
        }
        const auto& stamp = pending_stereo_->left->header.stamp;
        const auto timestamp = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
        return !use_imu_ || last_imu_timestamp_ >= timestamp;
    }

    void wake_ready_stereo()
    {
        bool ready;
        {
            std::scoped_lock lock(stereo_pending_mutex_, imu_mutex_);
            ready = stereo_ready_locked();
        }
        if (ready)
        {
            tracking_event_->trigger();
        }
    }

    void process_ready_stereo()
    {
        if (!enabled_.load())
            return;
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        if (!enabled_.load())
            return;
        if (replay_ack_pub_ && replay_imu_overflow_.load())
        {
            if (!replay_failed_)
                publish_replay_error("pending IMU capacity exceeded");
            return;
        }
        std::unique_ptr<PendingStereo> frame;
        {
            std::scoped_lock lock(stereo_pending_mutex_, imu_mutex_);
            if (!stereo_ready_locked())
            {
                return;
            }
            frame = std::move(pending_stereo_);
        }
        bool track_attempted = false;
        bool track_error_recorded = false;
        std::string failure;
        if (!replay_ack_pub_)
        {
            // Preserve realtime exception handling, including propagation of unknown errors.
            ProcessStereo(*frame, track_attempted, track_error_recorded, failure);
            return;
        }
        try
        {
            ProcessStereo(*frame, track_attempted, track_error_recorded, failure);
        }
        catch (const std::exception& error)
        {
            failure = error.what();
        }
        catch (...)
        {
            failure = "unknown processing exception";
        }
        // Only the exception from Track itself is an attempted, recorded outcome.
        // Enqueue/write failure below still takes precedence over acknowledgement.
        if (track_error_recorded)
            failure.clear();
        frame.reset(); // Release both imports before acknowledging the producer.
        if (replay_ack_pub_)
        {
            if (replay_imu_overflow_.load())
                failure = "pending IMU capacity exceeded";
            if (replay_record_failed_ || (frame_recorder_ && frame_recorder_->failed()))
                failure = "Track record enqueue/write failed";
            if (!failure.empty() || !track_attempted)
            {
                publish_replay_error(
                    failure.empty() ? "processing returned before Track" : failure);
                return;
            }
            std_msgs::msg::String ack;
            ack.data = replay_keys_.at(replay_position_).text();
            ++replay_position_;
            replay_pair_pending_ = false;
            replay_deadline_ = std::chrono::steady_clock::now() + replay_timeout_;
            replay_ack_pub_->publish(ack);
        }
    }

    void stereo_callback(const Frame::ConstSharedPtr& left, const Frame::ConstSharedPtr& right)
    {
        if (!enabled_.load())
            return;
        std::shared_lock<std::shared_mutex> session_lock(session_mutex_);
        if (!enabled_.load())
            return;
        const auto stamp = static_cast<std::int64_t>(left->header.stamp.sec) * 1000000000LL +
            left->header.stamp.nanosec;
        if (stamp <= session_start_ns_)
            return;
        if (left->frame_index != right->frame_index || left->header.stamp != right->header.stamp)
        {
            if (replay_ack_pub_)
            {
                publish_replay_error("mismatched stereo pair");
                return;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "dropping mismatched stereo pair left=%u right=%u", left->frame_index,
                right->frame_index);
            return;
        }
        try
        {
            // Retain real VB references, not just descriptors, while waiting for IMU.
            auto frame =
                std::make_unique<PendingStereo>(left, right, dma_import_audit_, get_logger());
            {
                std::lock_guard<std::mutex> lock(stereo_pending_mutex_);
                if (replay_ack_pub_ && pending_stereo_)
                    throw std::runtime_error("replay cannot replace a pending target");
                pending_stereo_.swap(frame); // At most one waiting stereo pair.
            }
            wake_ready_stereo();
        }
        catch (const std::exception& error)
        {
            if (replay_ack_pub_)
                publish_replay_error(error.what());
            else
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "VIO stereo enqueue failed: %s", error.what());
        }
    }

    // Tracking already published the local pose. Optional mapping failures
    // leave that result and the active relocalization state intact.
    void HandleSlamResult(const PendingStereo& frame, std::int64_t timestamp_ns,
        const localization::TrackingResult& frame_result)
    {
        const auto& left = frame.left;
        const auto& estimate = frame_result.image_pose;
        localization::Pose slam_pose;
        bool slam_pose_valid = false;
        try
        {
            if (slam_enabled_)
            {
                if (!frame_result.map_error.empty())
                    throw std::runtime_error(frame_result.map_error);
                if (!frame_result.map_pose)
                    throw std::runtime_error("Mapping did not return a pose");
                slam_pose = *frame_result.map_pose;
                slam_pose_valid = true;
                const double slam_ms = frame_result.map_submit_ms;
                update_map_metrics(slam_ms);
                if (localized_ && !localization_request_->busy())
                    map_from_odom_ = localization_pose(slam_pose) *
                        localization_pose(*estimate.tracking_from_body).inverse();
                if (slam_frame_recorder_ && !localization_request_->busy())
                {
                    FrameRecord record;
                    record.source_frame_index = left->frame_index;
                    record.image_timestamp_ns = record.estimate_timestamp_ns = timestamp_ns;
                    record.status = FrameRecord::Status::Ok;
                    for (std::size_t i = 0; i < 3; ++i)
                        record.translation[i] = slam_pose.translation_m[i];
                    for (std::size_t i = 0; i < 4; ++i)
                        record.quaternion_xyzw[i] = slam_pose.orientation_xyzw[i];
                    record.track_duration_ms = slam_ms;
                    if (!slam_frame_recorder_->enqueue(record))
                        RCLCPP_ERROR(get_logger(), "SLAM live frame recording failed/dropped");
                }
            }
            if (slam_pose_valid && !localization_request_->busy())
            {
                publish_slam_pose(timestamp_ns, slam_pose);
                if (publish_map_to_odom_tf_)
                    publish_map_transform(estimate.timestamp_ns);
            }
            if (localization_history_ && !localized_)
            {
                const auto copy_start = std::chrono::steady_clock::now();
                try
                {
                    const auto pixels = static_cast<std::uint64_t>(left->width) * left->height;
                    if (pixels > localization_history_->maximum_bytes() / 2)
                        throw std::runtime_error(
                            "Stereo pair exceeds localization history byte limit");
                    LocalizationStereo saved;
                    saved.timestamp_ns = timestamp_ns;
                    saved.source_frame_index = left->frame_index;
                    saved.width = left->width;
                    saved.height = left->height;
                    saved.left = frame.left_buffer.copy_y(*left);
                    saved.right = frame.right_buffer.copy_y(*frame.right);
                    if (!localization_history_->push(std::move(saved)))
                        throw std::runtime_error("Localization history rejected stereo pair");
                    const double copy_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - copy_start)
                                                .count();
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "localization_history_copy frame=%u elapsed=%.3fms", left->frame_index,
                        copy_ms);
                }
                catch (const std::exception& error)
                {
                    publish_slam_status("FAILED", error.what(), timestamp_ns);
                }
            }
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "Optional SLAM/localization work failed: %s", error.what());
            // This is not a native localization completion. Preserve the
            // request/map state instead of restarting global queries while
            // localized_ or an in-flight native request is still active.
        }
    }

    // The caller serializes stereo work and holds shared session ownership.
    // It owns both DMA imports through this call and releases them before replay ACK.
    // Replay uses track_attempted to distinguish early drops and track_error_recorded
    // to acknowledge recorded tracking failures. Other exceptions set failure.
    void ProcessStereo(const PendingStereo& frame, bool& track_attempted,
        bool& track_error_recorded, std::string& failure)
    {
        const auto& left = frame.left;
        const auto& right = frame.right;
        const auto& left_buffer = frame.left_buffer;
        const auto& right_buffer = frame.right_buffer;
        try
        {
            if (left->plane_num == 0 || right->plane_num == 0 || left->width == 0 ||
                left->height == 0 || right->width != left->width || right->height != left->height)
            {
                throw std::runtime_error("invalid stereo MPP descriptor");
            }
            if (static_cast<int>(left->width) != expected_image_width_ ||
                static_cast<int>(left->height) != expected_image_height_)
            {
                throw std::runtime_error("stereo descriptor size does not match the "
                                            "configured Kalibr scale");
            }
            const std::int64_t timestamp_ns =
                static_cast<std::int64_t>(left->header.stamp.sec) * 1000000000LL +
                static_cast<std::int64_t>(left->header.stamp.nanosec);
            std::vector<localization::InputImage> images;
            images.push_back(make_image(left_buffer.address(), *left, timestamp_ns, 0));
            images.push_back(make_image(right_buffer.address(), *right, timestamp_ns, 1));

            const auto start = std::chrono::steady_clock::now();
            localization::TrackingResult frame_result;
            localization::PoseEstimate estimate;
            std::vector<localization::FeatureObservation> observations;
            std::vector<localization::FeatureObservation> pose_input_observations;
            localization::FrameQuality frame_diagnostics;
            std::vector<ImuSample> registered_imu;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                while (!pending_imu_.empty() && pending_imu_.front().timestamp_ns <= timestamp_ns)
                {
                    const ImuSample sample = pending_imu_.front();
                    pending_imu_.pop_front();
                    registered_imu.push_back(sample);
                }
            }
            const auto registration_start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(odometry_mutex_);
                if (timestamp_ns <= last_image_timestamp_ns_)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "dropping non-monotonic stereo timestamp at frame=%u", left->frame_index);
                    return;
                }
                for (const auto& sample : registered_imu)
                {
                    if (sample.timestamp_ns <= last_registered_imu_timestamp_)
                    {
                        continue;
                    }
                    try
                    {
                        localization::InertialSample measurement;
                        measurement.timestamp_ns = sample.timestamp_ns;
                        measurement.acceleration_m_s2 = {sample.acceleration.x(),
                            sample.acceleration.y(), sample.acceleration.z()};
                        measurement.angular_velocity_rad_s = {sample.angular_velocity.x(),
                            sample.angular_velocity.y(), sample.angular_velocity.z()};
                        session_->submit_imu(measurement);
                        last_registered_imu_timestamp_ = sample.timestamp_ns;
                    }
                    catch (const std::exception& error)
                    {
                        if (replay_ack_pub_)
                            throw;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "IMU submission failed: %s", error.what());
                    }
                }
                const auto track_start = std::chrono::steady_clock::now();
                FrameRecord record;
                record.source_frame_index = left->frame_index;
                record.image_timestamp_ns = timestamp_ns;
                try
                {
                    track_attempted = true;
                    frame_result = session_->process(images);
                    estimate = frame_result.image_pose;
                    frame_diagnostics = frame_result.diagnostics;
                }
                catch (...)
                {
                    feature_diagnostics_.reset();
                    record.track_duration_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - track_start)
                                                    .count();
                    record_frame(record);
                    track_error_recorded = true;
                    publish_tracking_mode("ERROR", left->frame_index, timestamp_ns);
                    throw;
                }
                const auto track_end = std::chrono::steady_clock::now();
                publish_tracking_mode(
                    tracking_mode_label(estimate.tracking_from_body.has_value(), use_imu_,
                        use_imu_ && estimate.tracking_from_body && frame_result.imu_fusion_active),
                    left->frame_index, timestamp_ns);
                if (frame_recorder_)
                {
                    record.estimate_timestamp_ns = estimate.timestamp_ns;
                    record.status = estimate.tracking_from_body ? FrameRecord::Status::Ok
                                                                : FrameRecord::Status::InvalidPose;
                    if (estimate.tracking_from_body)
                    {
                        const auto& pose = *estimate.tracking_from_body;
                        for (std::size_t i = 0; i < 3; ++i)
                            record.translation[i] = pose.translation_m[i];
                        for (std::size_t i = 0; i < 4; ++i)
                            record.quaternion_xyzw[i] = pose.orientation_xyzw[i];
                    }
                    record.track_duration_ms =
                        std::chrono::duration<double, std::milli>(track_end - track_start).count();
                    record_frame(record);
                }
                if (publish_features_ &&
                    (estimate.tracking_from_body || publish_frame_diagnostics_))
                {
                    observations = session_->features(0);
                }
                if (publish_pose_inputs_)
                {
                    pose_input_observations = session_->pose_inputs(0);
                }
                last_image_timestamp_ns_ = timestamp_ns;
                const auto reset_start = std::chrono::steady_clock::now();
                if (use_imu_ && estimate.tracking_from_body)
                {
                    reset_propagator(timestamp_ns, *estimate.tracking_from_body, frame_result);
                }
                const auto reset_end = std::chrono::steady_clock::now();
                const double track_ms =
                    std::chrono::duration<double, std::milli>(track_end - track_start).count();
                const double reset_ms =
                    std::chrono::duration<double, std::milli>(reset_end - reset_start).count();
                const double registration_ms =
                    std::chrono::duration<double, std::milli>(track_start - registration_start)
                        .count();
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "vio_timing frame=%u registered_imu=%zu register=%.2fms "
                    "localization_process=%.2fms external_reset=%.2fms ext_prop=%s",
                    left->frame_index, registered_imu.size(), registration_ms, track_ms, reset_ms,
                    enable_external_propagation_ ? "on" : "off");
            }
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                    .count();
            if (publish_pose_inputs_)
            {
                publish_features(pose_input_pub_, timestamp_ns, left->frame_index, left->width,
                    left->height, pose_input_observations, nullptr,
                    estimate.tracking_from_body.has_value(), true);
            }
            if (publish_frame_diagnostics_)
                publish_features(feature_pub_, timestamp_ns, left->frame_index, left->width,
                    left->height, observations, &frame_diagnostics,
                    estimate.tracking_from_body.has_value());
            if (!estimate.tracking_from_body)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "spacemit_vslam returned invalid pose for frame=%u (%.2f ms)",
                    left->frame_index, elapsed_ms);
                return;
            }

            // Preserve a valid local pose even if the Session map update failed.
            // Saved-map failures and history copies must not skip this publication.
            publish_image_pose(estimate, left->frame_index);
            if (publish_features_ && !publish_frame_diagnostics_)
            {
                publish_features(feature_pub_, estimate.timestamp_ns, left->frame_index,
                    left->width, left->height, observations, nullptr, true);
            }
            ++tracked_frames_;

            HandleSlamResult(frame, timestamp_ns, frame_result);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "tracked=%" PRIu64 " frame=%u track=%.2fms", tracked_frames_, left->frame_index,
                elapsed_ms);
        }
        catch (const std::exception& error)
        {
            failure = error.what();
            publish_tracking_mode("ERROR");
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "VIO stereo callback failed: %s", error.what());
        }
    }

    void publish_tracking_mode(
        const char* mode, std::uint32_t frame = 0, std::int64_t timestamp_ns = 0)
    {
        // Called by serialized stereo work, or with exclusive session ownership
        // during enable/stop. No extra timer or tracker lock is required.
        std_msgs::msg::String message;
        message.data = mode;
        tracking_mode_pub_->publish(message);
        if (last_tracking_mode_ != mode)
        {
            RCLCPP_INFO(get_logger(), "tracking_mode=%s frame=%u timestamp_ns=%" PRId64, mode,
                frame, timestamp_ns);
            last_tracking_mode_ = mode;
        }
    }

    localization::InputImage make_image(const std::uint8_t* address, const Frame& message,
        std::int64_t timestamp_ns, std::uint32_t camera_index) const
    {
        localization::InputImage image;
        image.pixels = address;
        image.width = static_cast<std::int32_t>(message.width);
        image.height = static_cast<std::int32_t>(message.height);
        image.pitch = static_cast<std::int32_t>(
            message.plane_stride[0] == 0 ? message.width : message.plane_stride[0]);
        image.timestamp_ns = timestamp_ns;
        image.camera_index = camera_index;
        return image;
    }

    void publish_image_pose(const localization::PoseEstimate& estimate, std::uint32_t frame_index)
    {
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(estimate.timestamp_ns);
        message.header.frame_id = world_frame_;
        message.child_frame_id = rig_frame_;
        fill_pose(message, *estimate.tracking_from_body);
        if (estimate.uncertainty)
        {
            const auto& matrix = estimate.uncertainty->matrix;
            std::copy(matrix.begin(), matrix.end(), message.pose.covariance.begin());
        }
        else
        {
            message.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Pose uncertainty unavailable; publishing unknown covariance as NaN");
        }
        image_pose_pub_->publish(message);
        publish_camera_transform(message);
        geometry_msgs::msg::PoseStamped trajectory_pose;
        trajectory_pose.header = message.header;
        trajectory_pose.pose = message.pose.pose;
        trajectory_poses_.push_back(std::move(trajectory_pose));
        while (trajectory_poses_.size() > static_cast<std::size_t>(trajectory_max_length_))
        {
            trajectory_poses_.pop_front();
        }
        const auto now = std::chrono::steady_clock::now();
        if (!trajectory_published_ ||
            now - last_trajectory_publish_ >=
                std::chrono::milliseconds(trajectory_publish_period_ms_))
        {
            nav_msgs::msg::Path trajectory;
            trajectory.header = message.header;
            trajectory.poses.assign(trajectory_poses_.begin(), trajectory_poses_.end());
            trajectory_pub_->publish(trajectory);
            last_trajectory_publish_ = now;
            trajectory_published_ = true;
        }
        last_image_frame_index_ = frame_index;
        publish_visualization(message);
    }

    void publish_camera_transform(const nav_msgs::msg::Odometry& message)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = message.header;
        transform.child_frame_id = rig_frame_;
        transform.transform.translation.x = message.pose.pose.position.x;
        transform.transform.translation.y = message.pose.pose.position.y;
        transform.transform.translation.z = message.pose.pose.position.z;
        transform.transform.rotation = message.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }

    void publish_visualization(const nav_msgs::msg::Odometry& message)
    {
        const auto now = std::chrono::steady_clock::now();
        if (visualization_published_ &&
            now - last_visualization_publish_ <
                std::chrono::milliseconds(visualization_publish_period_ms_))
        {
            return;
        }

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker trajectory;
        trajectory.header = message.header;
        trajectory.ns = "vio_trajectory";
        trajectory.id = 0;
        trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory.action = visualization_msgs::msg::Marker::ADD;
        trajectory.pose.orientation.w = 1.0;
        trajectory.scale.x = trajectory_line_width_;
        trajectory.color.r = 0.10F;
        trajectory.color.g = 0.85F;
        trajectory.color.b = 1.00F;
        trajectory.color.a = 1.00F;
        trajectory.points.reserve(trajectory_poses_.size());
        for (const auto& pose : trajectory_poses_)
        {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            trajectory.points.push_back(point);
        }
        markers.markers.push_back(std::move(trajectory));

        const Eigen::Vector3f origin(static_cast<float>(message.pose.pose.position.x),
            static_cast<float>(message.pose.pose.position.y),
            static_cast<float>(message.pose.pose.position.z));
        const Eigen::Quaternionf world_from_rig(static_cast<float>(message.pose.pose.orientation.w),
            static_cast<float>(message.pose.pose.orientation.x),
            static_cast<float>(message.pose.pose.orientation.y),
            static_cast<float>(message.pose.pose.orientation.z));
        const std::array<Eigen::Vector3f, 3> axes{
            Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ()};
        const std::array<std::array<float, 3>, 3> colors{{
            {{1.00F, 0.15F, 0.15F}},
            {{0.15F, 1.00F, 0.15F}},
            {{0.15F, 0.35F, 1.00F}},
        }};
        for (std::size_t index = 0; index < axes.size(); ++index)
        {
            visualization_msgs::msg::Marker axis;
            axis.header = message.header;
            axis.ns = "camera_rig_axes";
            axis.id = static_cast<std::int32_t>(index);
            axis.type = visualization_msgs::msg::Marker::ARROW;
            axis.action = visualization_msgs::msg::Marker::ADD;
            axis.scale.x = camera_axis_shaft_diameter_;
            axis.scale.y = camera_axis_shaft_diameter_ * 2.0;
            axis.scale.z = camera_axis_shaft_diameter_ * 3.0;
            axis.color.r = colors[index][0];
            axis.color.g = colors[index][1];
            axis.color.b = colors[index][2];
            axis.color.a = 1.0F;
            geometry_msgs::msg::Point start;
            start.x = origin.x();
            start.y = origin.y();
            start.z = origin.z();
            const Eigen::Vector3f end = origin + world_from_rig * axes[index] * camera_axis_length_;
            geometry_msgs::msg::Point finish;
            finish.x = end.x();
            finish.y = end.y();
            finish.z = end.z();
            axis.points = {start, finish};
            markers.markers.push_back(std::move(axis));
        }
        visualization_pub_->publish(markers);
        last_visualization_publish_ = now;
        visualization_published_ = true;
    }

    void publish_slam_pose(std::int64_t timestamp_ns, const localization::Pose& pose)
    {
        if (!slam_pose_pub_ || !slam_trajectory_pub_)
        {
            return;
        }
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(timestamp_ns);
        message.header.frame_id = localized_ ? slam_map_frame_ : slam_local_frame_;
        message.child_frame_id = rig_frame_;
        fill_pose(message, pose);
        slam_pose_pub_->publish(message);

        geometry_msgs::msg::PoseStamped trajectory_pose;
        trajectory_pose.header = message.header;
        trajectory_pose.pose = message.pose.pose;
        slam_trajectory_poses_.push_back(std::move(trajectory_pose));
        while (slam_trajectory_poses_.size() > static_cast<std::size_t>(trajectory_max_length_))
        {
            slam_trajectory_poses_.pop_front();
        }
        publish_slam_trajectory(message);
    }

    void publish_slam_trajectory(const nav_msgs::msg::Odometry& message)
    {
        const auto now = std::chrono::steady_clock::now();
        if (slam_trajectory_published_ &&
            now - last_slam_trajectory_publish_ <
                std::chrono::milliseconds(trajectory_publish_period_ms_))
        {
            return;
        }
        nav_msgs::msg::Path trajectory;
        trajectory.header = message.header;
        trajectory.poses.assign(slam_trajectory_poses_.begin(), slam_trajectory_poses_.end());
        slam_trajectory_pub_->publish(trajectory);
        last_slam_trajectory_publish_ = now;
        slam_trajectory_published_ = true;
    }

    void update_map_metrics(double submit_ms)
    {
        const auto diagnostics = session_->map_stats();
        for (const auto& event : session_->loop_closures())
            loop_closure_timestamps_.insert(event.timestamp_ns);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "mapping submit=%.2fms correction_events=%zu diagnostic_metrics=%zu", submit_ms,
            loop_closure_timestamps_.size(), diagnostics.metrics.size());
    }

    void publish_features(
        const rclcpp::Publisher<common::msg::FeaturePointArray>::SharedPtr& publisher,
        std::int64_t timestamp_ns, std::uint32_t frame_index, std::uint32_t image_width,
        std::uint32_t image_height,
        const std::vector<localization::FeatureObservation>& observations,
        const localization::FrameQuality* diagnostics = nullptr, bool pose_valid = false,
        bool pose_inputs_only = false)
    {
        if (!publisher)
            return;
        common::msg::FeaturePointArray message;
        message.header.stamp = rclcpp::Time(timestamp_ns);
        message.header.frame_id = "camera_left";
        message.source_frame_index = frame_index;
        message.image_width = image_width;
        message.image_height = image_height;
        if (diagnostics)
        {
            feature_diagnostics_.populate(
                message, timestamp_ns, observations, *diagnostics, pose_valid);
            publisher->publish(message);
            return;
        }
        message.pose_valid = pose_valid;
        message.pose_inputs_available = pose_inputs_only;
        message.points.reserve(observations.size());
        for (const auto& observation : observations)
        {
            if (observation.sensor_id != 0 || !std::isfinite(observation.pixel[0]) ||
                !std::isfinite(observation.pixel[1]))
            {
                continue;
            }
            common::msg::FeaturePoint point;
            point.id = observation.track_id;
            point.x = observation.pixel[0];
            point.y = observation.pixel[1];
            point.pose_input = pose_inputs_only;
            point.track_observation_count = 1;
            message.points.push_back(point);
        }
        message.tracked_feature_count = static_cast<std::uint32_t>(message.points.size());
        message.pose_input_count = pose_inputs_only ? message.tracked_feature_count : 0;
        publisher->publish(message);
    }

    void publish_propagated_pose(const PropagatedState& state)
    {
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(state.timestamp_ns);
        message.header.frame_id = world_frame_;
        message.child_frame_id = rig_frame_;
        message.pose.pose.position.x = state.position.x();
        message.pose.pose.position.y = state.position.y();
        message.pose.pose.position.z = state.position.z();
        message.pose.pose.orientation.x = state.world_from_rig.x();
        message.pose.pose.orientation.y = state.world_from_rig.y();
        message.pose.pose.orientation.z = state.world_from_rig.z();
        message.pose.pose.orientation.w = state.world_from_rig.w();
        const Eigen::Vector3f velocity_rig = state.world_from_rig.conjugate() * state.velocity;
        message.twist.twist.linear.x = velocity_rig.x();
        message.twist.twist.linear.y = velocity_rig.y();
        message.twist.twist.linear.z = velocity_rig.z();
        imu_pose_pub_->publish(message);
    }

    void fill_pose(nav_msgs::msg::Odometry& message, const localization::Pose& pose) const
    {
        message.pose.pose.position.x = pose.translation_m[0];
        message.pose.pose.position.y = pose.translation_m[1];
        message.pose.pose.position.z = pose.translation_m[2];
        message.pose.pose.orientation.x = pose.orientation_xyzw[0];
        message.pose.pose.orientation.y = pose.orientation_xyzw[1];
        message.pose.pose.orientation.z = pose.orientation_xyzw[2];
        message.pose.pose.orientation.w = pose.orientation_xyzw[3];
    }

    void reset_propagator(std::int64_t timestamp_ns, const localization::Pose& pose,
        const localization::TrackingResult& result)
    {
        const auto& imu_state = result.imu;
        const auto& gravity = result.gravity;
        const Eigen::Quaternionf world_from_rig = quaternion_from_pose(pose);
        const Eigen::Vector3f position = vector_from_pose(pose);
        const auto rig_from_imu = rig_from_imu_rotation();

        Eigen::Vector3f gravity_rig;
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
        Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
        bool gravity_publish_ready = false;
        if (imu_state && gravity)
        {
            gravity_rig = Eigen::Vector3f((*gravity)[0], (*gravity)[1], (*gravity)[2]);
            velocity = Eigen::Vector3f(imu_state->velocity_tracking_m_s[0],
                imu_state->velocity_tracking_m_s[1], imu_state->velocity_tracking_m_s[2]);
            gyro_bias = Eigen::Vector3f(imu_state->estimated_bias.angular_rad_s[0],
                imu_state->estimated_bias.angular_rad_s[1],
                imu_state->estimated_bias.angular_rad_s[2]);
            accel_bias = Eigen::Vector3f(imu_state->estimated_bias.linear_m_s2[0],
                imu_state->estimated_bias.linear_m_s2[1], imu_state->estimated_bias.linear_m_s2[2]);
            gravity_publish_ready = true;
        }
        else
        {
            // spacemit_vslam exposes gravity/bias only after inertial
            // initialization.  Do not leave the required high-rate pose
            // topic silent during that interval: use the newest camera IMU
            // sample to establish a gravity direction, then replace this
            // seed with the tracker state as soon as it becomes available.
            std::optional<ImuSample> latest;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                for (auto it = imu_history_.rbegin(); it != imu_history_.rend(); ++it)
                {
                    if (it->timestamp_ns <= timestamp_ns)
                    {
                        latest = *it;
                        break;
                    }
                }
            }
            if (!latest)
                return;
            Eigen::Vector3f acceleration = latest->acceleration;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (static_gravity_ready_)
                {
                    acceleration = static_acceleration_;
                    gravity_publish_ready = true;
                }
            }
            const Eigen::Vector3f accel_rig = rig_from_imu * acceleration;
            const float magnitude = accel_rig.norm();
            if (!(magnitude > 1.0e-3F) || !std::isfinite(magnitude))
            {
                return;
            }
            gravity_rig = -accel_rig * (kGravityMps2 / magnitude);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "propagator seeded from raw IMU while spacemit_vslam "
                "inertial state initializes");
        }
        const Eigen::Vector3f gravity_world = world_from_rig * gravity_rig;
        if (gravity_publish_ready)
        {
            geometry_msgs::msg::Vector3Stamped message;
            message.header.stamp = rclcpp::Time(timestamp_ns);
            message.header.frame_id = world_frame_;
            message.vector.x = gravity_world.x();
            message.vector.y = gravity_world.y();
            message.vector.z = gravity_world.z();
            gravity_pub_->publish(message);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "gravity world=[%.5f %.5f %.5f] norm=%.5f source=%s", gravity_world.x(),
                gravity_world.y(), gravity_world.z(), gravity_world.norm(),
                (imu_state && gravity) ? "spacemit_vslam" : "static_imu");
        }

        if (!enable_external_propagation_)
        {
            return;
        }

        std::lock_guard<std::mutex> propagation_lock(propagation_mutex_);
        std::deque<ImuSample> imu_history;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_history = imu_history_;
        }

        propagator_.reset(timestamp_ns, world_from_rig, position, velocity, gravity_world,
            gyro_bias, accel_bias, rig_from_imu);
        for (const auto& sample : imu_history)
        {
            if (sample.timestamp_ns > timestamp_ns)
            {
                propagator_.integrate(sample);
            }
        }
    }

    Eigen::Matrix3f rig_from_imu_rotation() const
    {
        // spacemit_vslam owns the parsed rig. The Kalibr convention uses the
        // left camera as the rig frame, so the extrinsic is cached at startup.
        return rig_from_imu_;
    }

    message_filters::Subscriber<Frame> left_sub_;
    message_filters::Subscriber<Frame> right_sub_;
    std::unique_ptr<message_filters::Synchronizer<StereoSyncPolicy>> sync_;
    std::vector<ReplayKey> replay_keys_;
    std::size_t replay_position_ = 0;
    bool replay_failed_ = false;
    bool replay_paused_ = false;
    bool replay_record_failed_ = false;
    std::atomic<bool> replay_imu_overflow_{false};
    bool replay_pair_pending_ = false;
    Frame::ConstSharedPtr replay_left_, replay_right_;
    rclcpp::Subscription<Frame>::SharedPtr replay_left_sub_, replay_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replay_pause_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr replay_ack_pub_, replay_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr replay_imu_ack_pub_;
    rclcpp::TimerBase::SharedPtr replay_watchdog_;
    std::chrono::steady_clock::time_point replay_deadline_;
    std::chrono::steady_clock::duration replay_timeout_{};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::CallbackGroup::SharedPtr stereo_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr image_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_mode_pub_;
    std::string last_tracking_mode_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr slam_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slam_ready_pub_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr slam_graph_;
    rclcpp::Event::SharedPtr slam_graph_event_;
    std::shared_ptr<ExecutorEvent> slam_readiness_event_;
    std::atomic<bool> slam_graph_stop_{false};
    std::thread slam_graph_thread_;
    bool slam_ready_value_{false};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_hint_sub_;
    std::shared_ptr<ExecutorEvent> localization_event_;
    std::shared_ptr<LocalizationRequest> localization_request_ =
        std::make_shared<LocalizationRequest>();
    std::unique_ptr<LocalizationHistory> localization_history_;
    localization::SearchRegion localization_settings_{};
    std::int64_t localization_max_age_ns_{};
    Eigen::Isometry3d map_from_odom_ = Eigen::Isometry3d::Identity();
    bool localized_{false};
    bool publish_map_to_odom_tf_{false};
    bool slam_save_attempted_{false};
    double slam_save_timeout_sec_{60};
    std::ofstream localization_audit_;
    std::string slam_map_save_directory_, slam_map_load_directory_;
    std::string slam_local_frame_, slam_map_frame_;
    rclcpp::Publisher<common::msg::FeaturePointArray>::SharedPtr feature_pub_;
    rclcpp::Publisher<common::msg::FeaturePointArray>::SharedPtr pose_input_pub_;
    FrameFeatureDiagnostics feature_diagnostics_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::deque<geometry_msgs::msg::PoseStamped> trajectory_poses_;
    std::deque<geometry_msgs::msg::PoseStamped> slam_trajectory_poses_;
    std::unique_ptr<localization::Session> session_;
    float minimum_observation_depth_m_ = 0.0F;
    std::shared_mutex session_mutex_;
    std::atomic<bool> enabled_{false};
    std::int64_t session_start_ns_{0};
    localization::RigCalibration rig_config_;
    localization::Options session_options_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    std::mutex odometry_mutex_;

    std::string world_frame_;
    std::string rig_frame_;
    std::unique_ptr<FrameRecorder> frame_recorder_;
    std::unique_ptr<FrameRecorder> slam_frame_recorder_;
    std::string image_pose_topic_;
    std::string imu_pose_topic_;
    std::string gravity_topic_;
    std::string trajectory_topic_;
    std::string feature_topic_;
    std::string pose_input_topic_;
    std::string slam_pose_topic_;
    std::string slam_trajectory_topic_;
    bool use_imu_ = true;
    camera_sdk::KalibrTimeAlignment imu_time_alignment_;
    bool enable_external_propagation_ = true;
    bool publish_static_gravity_fallback_ = true;
    bool publish_features_ = true;
    bool publish_frame_diagnostics_ = false;
    bool publish_pose_inputs_ = false;
    bool slam_enabled_ = false;
    int trajectory_max_length_{2000};
    int trajectory_publish_period_ms_{100};
    int visualization_publish_period_ms_{50};
    double trajectory_line_width_{0.012};
    double camera_axis_length_{0.15};
    double camera_axis_shaft_diameter_{0.012};
    bool trajectory_published_{false};
    bool slam_trajectory_published_{false};
    std::chrono::steady_clock::time_point last_trajectory_publish_{};
    std::chrono::steady_clock::time_point last_slam_trajectory_publish_{};
    std::chrono::steady_clock::time_point last_visualization_publish_{};
    bool visualization_published_{false};
    std::set<std::int64_t> loop_closure_timestamps_;
    Eigen::Matrix3f rig_from_imu_ = Eigen::Matrix3f::Identity();
    int gravity_static_min_samples_{120};
    double gravity_accel_tolerance_m_s2_{0.75};
    double gravity_gyro_threshold_rad_s_{0.08};
    int expected_image_width_{0};
    int expected_image_height_{0};

    std::mutex imu_mutex_;
    std::mutex stereo_pending_mutex_;
    std::unique_ptr<PendingStereo> pending_stereo_;
    std::shared_ptr<ExecutorEvent> tracking_event_;
    std::deque<ImuSample> imu_history_;
    std::deque<ImuSample> pending_imu_;
    Eigen::Vector3f static_acceleration_sum_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f static_acceleration_ = Eigen::Vector3f::Zero();
    int static_gravity_samples_{0};
    bool static_gravity_ready_{false};
    std::int64_t last_imu_timestamp_ = 0;
    std::int64_t last_registered_imu_timestamp_ = 0;
    std::int64_t last_image_timestamp_ns_ = 0;
    std::mutex propagation_mutex_;
    ImuPropagator propagator_;
    std::uint64_t tracked_frames_ = 0;
    std::uint64_t imu_samples_ = 0;
    std::uint64_t propagated_samples_ = 0;
    std::uint32_t last_image_frame_index_ = 0;
    bool sys_initialized_{false};
    bool vb_initialized_{false};
    std::shared_ptr<DmaImportAudit> dma_import_audit_ = std::make_shared<DmaImportAudit>();
    std::unique_ptr<std::FILE, DmaAuditFileCloser> dma_audit_file_;
};

} // namespace vio

RCLCPP_COMPONENTS_REGISTER_NODE(vio::RobotVioNode)
