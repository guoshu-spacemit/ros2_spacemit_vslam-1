#include "camera_mpp_publisher.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <stdexcept>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>

#include "decoder_input_pool_mpp.h"
#include "demux/demux_api.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "spacemit_camera/imu_frequency.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace camera_io::detail
{

namespace
{

std::optional<int> ros_domain_id_from_environment()
{
    const char* value = std::getenv("ROS_DOMAIN_ID");
    if (value == nullptr || value[0] == '\0')
    {
        return std::nullopt;
    }

    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < 0 || parsed > 232)
    {
        throw std::runtime_error("ROS_DOMAIN_ID must be an integer in the range [0, 232]");
    }
    return static_cast<int>(parsed);
}

CropConfig load_crop(const boost::property_tree::ptree& root, const std::string& path)
{
    CropConfig crop;
    crop.x = root.get<int>(path + ".x");
    crop.y = root.get<int>(path + ".y");
    crop.width = root.get<int>(path + ".width");
    crop.height = root.get<int>(path + ".height");
    return crop;
}

StreamConfig load_stream(const boost::property_tree::ptree& root, const std::string& path)
{
    StreamConfig stream;
    stream.topic = root.get<std::string>(path + ".topic");
    stream.camera_info_topic = root.get<std::string>(path + ".camera_info_topic", "");
    stream.frame_id = root.get<std::string>(path + ".frame_id");
    stream.crop = load_crop(root, path + ".crop");
    stream.output_width = root.get<int>(path + ".output.width", stream.crop.width);
    stream.output_height = root.get<int>(path + ".output.height", stream.crop.height);
    return stream;
}

} // namespace

BridgeConfig load_config(const std::string& path, int fps_override)
{
    if (fps_override < 0)
    {
        throw std::runtime_error("fps must be zero (JSON) or positive");
    }
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);

    BridgeConfig config;
    config.input_type = root.get<std::string>("input.type", "uvc");
    config.input_file = root.get<std::string>("input.file", "");
    if (!config.input_file.empty())
    {
        std::filesystem::path input_path(config.input_file);
        if (input_path.is_relative())
        {
            input_path = std::filesystem::path(path).parent_path() / input_path;
        }
        config.input_file = std::filesystem::weakly_canonical(input_path).string();
    }
    config.device = root.get<std::string>("camera.device", "");
    config.width = root.get<int>("camera.width");
    config.height = root.get<int>("camera.height");
    config.fps = fps_override > 0 ? fps_override : root.get<int>("camera.fps");
    config.input_format = root.get<std::string>("camera.input_format");
    config.decoded_format = root.get<std::string>("camera.decoded_format");
    config.decode_enabled = root.get<bool>("camera.decode", true);
    config.timestamp_source = root.get<std::string>("timestamp.source", "ros_clock");
    config.gpio_sync_enabled = root.get<bool>("timestamp.gpio.enabled", false);
    config.startup_alignment.enabled =
        root.get<bool>("timestamp.gpio.startup_alignment.enabled", false);
    if (config.startup_alignment.enabled)
    {
        if (!config.gpio_sync_enabled)
            throw std::runtime_error("startup_alignment requires timestamp.gpio.enabled");
        const auto& options = root.get_child("timestamp.gpio.startup_alignment");
        config.startup_alignment.timeout_ms = options.get<int>("timeout_ms", 10000);
        config.startup_alignment.frames_per_step = options.get<unsigned>("frames_per_step", 4);
        config.startup_alignment.tolerance_us = options.get<int>("tolerance_us", 100);
        if (!options.get<bool>("restore_exposure", true))
            throw std::runtime_error("startup_alignment must restore the original exposure");
        if (auto sequence = options.get_child_optional("exposure_us"))
        {
            config.startup_alignment.exposure_us.clear();
            for (const auto& entry : *sequence)
                config.startup_alignment.exposure_us.push_back(entry.second.get_value<int>());
        }
        camera_sdk::StartupAlignment::validate(config.startup_alignment);
    }
    if (config.gpio_sync_enabled)
    {
        auto duration = [&](const char* key, std::int64_t fallback, std::int64_t scale)
        {
            const auto value = root.get<std::int64_t>(key, fallback);
            if (value <= 0 || value > 1000000)
                throw std::runtime_error(std::string("invalid GPIO duration: ") + key);
            return value * scale;
        };
        config.gpio_chip = root.get<std::string>("timestamp.gpio.chip");
        config.gpio_line = root.get<unsigned>("timestamp.gpio.line");
        config.gpio_active_low = root.get<bool>("timestamp.gpio.active_low", false);
        config.gpio_match.window = root.get<unsigned>("timestamp.gpio.window", 32);
        config.gpio_match.capacity = root.get<unsigned>("timestamp.gpio.capacity", 256);
        config.gpio_match.width_tolerance_ns =
            duration("timestamp.gpio.width_tolerance_us", 100, 1000);
        config.gpio_match.interval_tolerance_ns =
            duration("timestamp.gpio.interval_tolerance_us", 200, 1000);
        config.gpio_match.max_age_ns = duration("timestamp.gpio.max_age_ms", 500, 1000000);
        config.gpio_match.diversity_ns = duration("timestamp.gpio.diversity_us", 500, 1000);
        (void)camera_sdk::ExposureMatcher(config.gpio_match);
    }
    const std::string calibration_format = root.get<std::string>("calibration.format", "");
    if (!calibration_format.empty() && calibration_format != "kalibr")
    {
        throw std::runtime_error("unsupported calibration format");
    }
    config.calibration_file = root.get<std::string>("calibration.file", "");
    config.publish_imu_calibration_tf = root.get<bool>("calibration.publish_imu_tf", true);
    if (!config.calibration_file.empty())
    {
        std::filesystem::path calibration_path(config.calibration_file);
        if (calibration_path.is_relative())
        {
            calibration_path = std::filesystem::path(path).parent_path() / calibration_path;
        }
        config.calibration_file = std::filesystem::weakly_canonical(calibration_path).string();
    }
    config.metadata_enabled = root.get<bool>("metadata.enabled", false);
    if (config.metadata_enabled)
    {
        config.metadata_parser = root.get<std::string>("metadata.parser");
        config.metadata_region = load_crop(root, "metadata.region");
    }
    config.full_only = root.get<bool>("streams.full_only", false);
    if (root.get_child_optional("streams.left"))
    {
        config.left = load_stream(root, "streams.left");
    }
    if (root.get_child_optional("streams.right"))
    {
        config.right = load_stream(root, "streams.right");
    }
    if (root.get_child_optional("streams.full"))
    {
        config.full = load_stream(root, "streams.full");
        config.full_enabled = true;
    }
    if (root.get_child_optional("streams.compressed"))
    {
        const auto& stream = root.get_child("streams.compressed");
        config.compressed.topic = stream.get<std::string>("topic");
        config.compressed.frame_id = stream.get<std::string>("frame_id");
        config.compressed_enabled = stream.get<bool>("enabled", true);
    }
    config.imu_enabled = root.get<bool>("imu.enabled", false);
    if (config.imu_enabled)
    {
        config.imu_raw_dds_topic = root.get<std::string>("imu.raw.dds_topic");
        config.imu_raw_ros_topic = root.get<std::string>("imu.raw.ros_topic");
        config.imu_frame_id = root.get<std::string>("imu.frame_id");
        config.domain_id = root.get<int>("imu.domain_id", 0);
        if (const auto environment_domain = ros_domain_id_from_environment())
        {
            config.domain_id = *environment_domain;
            config.domain_id_from_environment = true;
        }
        config.imu_queue_depth = root.get<int>("imu.queue_depth", 16);
        const auto nominal_frequency = root.get_child_optional("imu.nominal_frequency_hz");
        const std::optional<double> nominal_frequency_hz = nominal_frequency
            ? std::make_optional(nominal_frequency->get_value<double>())
            : std::nullopt;
        const int samples_per_frame =
            nominal_frequency ? 0 : root.get<int>("imu.samples_per_frame");
        config.imu_nominal_frequency_hz = static_cast<float>(
            camera_sdk::ResolveImuFrequencyHz(nominal_frequency_hz, samples_per_frame, config.fps));
        config.ahrs_enabled = root.get<bool>("imu.ahrs.enabled", false);
        if (config.ahrs_enabled)
        {
            config.imu_fused_dds_topic = root.get<std::string>("imu.ahrs.dds_topic");
            config.imu_fused_ros_topic = root.get<std::string>("imu.ahrs.ros_topic");
            config.ahrs_config.kp = root.get<float>("imu.ahrs.kp", 1.0F);
            config.ahrs_config.ki = root.get<float>("imu.ahrs.ki", 0.0F);
            config.ahrs_config.accel_rejection_m_s2 =
                root.get<float>("imu.ahrs.accel_rejection_m_s2", 3.0F);
            config.ahrs_config.max_dt_seconds =
                root.get<float>("imu.ahrs.max_dt_ms", 50.0F) * 1.0e-3F;
            const float orientation_stddev =
                root.get<float>("imu.ahrs.orientation_stddev_rad", 0.05F);
            config.orientation_variance = orientation_stddev * orientation_stddev;
        }
    }
    config.max_exported_pairs = root.get<int>("buffers.max_exported_pairs", 6);
    config.decoder_buffer_count = root.get<int>("buffers.decoder_buffer_count", 0);
    config.decoder_input_pool = load_decoder_input_pool(root);
    return config;
}

} // namespace camera_io::detail

namespace camera_io
{
namespace
{

using detail::CropConfig;

bool region_is_valid(const CropConfig& region, int image_width, int image_height)
{
    return region.x >= 0 && region.y >= 0 && region.width > 0 && region.height > 0 &&
        region.x + region.width <= image_width && region.y + region.height <= image_height;
}

camera_sdk::Transform4d inverse_rigid_transform(const camera_sdk::Transform4d& transform)
{
    camera_sdk::Transform4d inverse{transform[0], transform[4], transform[8], 0.0, transform[1],
        transform[5], transform[9], 0.0, transform[2], transform[6], transform[10], 0.0, 0.0, 0.0,
        0.0, 1.0};
    for (int row = 0; row < 3; ++row)
    {
        inverse[row * 4 + 3] = -(inverse[row * 4] * transform[3] +
            inverse[row * 4 + 1] * transform[7] + inverse[row * 4 + 2] * transform[11]);
    }
    return inverse;
}

geometry_msgs::msg::Transform transform_message(const camera_sdk::Transform4d& matrix)
{
    geometry_msgs::msg::Transform message;
    message.translation.x = matrix[3];
    message.translation.y = matrix[7];
    message.translation.z = matrix[11];

    const double trace = matrix[0] + matrix[5] + matrix[10];
    double qw;
    double qx;
    double qy;
    double qz;
    if (trace > 0.0)
    {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * scale;
        qx = (matrix[9] - matrix[6]) / scale;
        qy = (matrix[2] - matrix[8]) / scale;
        qz = (matrix[4] - matrix[1]) / scale;
    }
    else if (matrix[0] > matrix[5] && matrix[0] > matrix[10])
    {
        const double scale = std::sqrt(1.0 + matrix[0] - matrix[5] - matrix[10]) * 2.0;
        qw = (matrix[9] - matrix[6]) / scale;
        qx = 0.25 * scale;
        qy = (matrix[1] + matrix[4]) / scale;
        qz = (matrix[2] + matrix[8]) / scale;
    }
    else if (matrix[5] > matrix[10])
    {
        const double scale = std::sqrt(1.0 + matrix[5] - matrix[0] - matrix[10]) * 2.0;
        qw = (matrix[2] - matrix[8]) / scale;
        qx = (matrix[1] + matrix[4]) / scale;
        qy = 0.25 * scale;
        qz = (matrix[6] + matrix[9]) / scale;
    }
    else
    {
        const double scale = std::sqrt(1.0 + matrix[10] - matrix[0] - matrix[5]) * 2.0;
        qw = (matrix[4] - matrix[1]) / scale;
        qx = (matrix[2] + matrix[8]) / scale;
        qy = (matrix[6] + matrix[9]) / scale;
        qz = 0.25 * scale;
    }
    message.rotation.w = qw;
    message.rotation.x = qx;
    message.rotation.y = qy;
    message.rotation.z = qz;
    return message;
}

} // namespace

using detail::StreamConfig;

sensor_msgs::msg::CameraInfo CameraMppPublisher::make_camera_info(
    const camera_sdk::KalibrCamera& calibration, const StreamConfig& stream) const
{
    if (calibration.camera_model != "pinhole")
    {
        throw std::runtime_error(
            calibration.name + ": only Kalibr pinhole cameras are currently supported");
    }
    sensor_msgs::msg::CameraInfo message;
    message.header.frame_id = stream.frame_id;
    message.width = static_cast<std::uint32_t>(stream.output_width);
    message.height = static_cast<std::uint32_t>(stream.output_height);
    if (calibration.distortion_model == "radtan")
    {
        message.distortion_model = "plumb_bob";
    }
    else if (calibration.distortion_model == "equidistant")
    {
        message.distortion_model = "equidistant";
    }
    else
    {
        throw std::runtime_error(
            calibration.name + ": unsupported distortion model " + calibration.distortion_model);
    }
    message.d = calibration.distortion_coeffs;
    const double scale_x =
        static_cast<double>(stream.output_width) / static_cast<double>(calibration.resolution[0]);
    const double scale_y =
        static_cast<double>(stream.output_height) / static_cast<double>(calibration.resolution[1]);
    const double fx = calibration.intrinsics[0] * scale_x;
    const double fy = calibration.intrinsics[1] * scale_y;
    const double cx = calibration.intrinsics[2] * scale_x;
    const double cy = calibration.intrinsics[3] * scale_y;
    message.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    message.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return message;
}

void CameraMppPublisher::publish_calibration_transforms()
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    const auto& left = calibration_->cameras[0];
    const auto& right = calibration_->cameras[1];
    if (config_.imu_enabled && config_.publish_imu_calibration_tf && left.t_cam_imu)
    {
        geometry_msgs::msg::TransformStamped imu_to_left;
        imu_to_left.header.stamp = now();
        imu_to_left.header.frame_id = config_.imu_frame_id;
        imu_to_left.child_frame_id = config_.left.frame_id;
        imu_to_left.transform = transform_message(inverse_rigid_transform(*left.t_cam_imu));
        transforms.push_back(std::move(imu_to_left));
    }
    if (right.t_cn_cnm1)
    {
        geometry_msgs::msg::TransformStamped left_to_right;
        left_to_right.header.stamp = now();
        left_to_right.header.frame_id = config_.left.frame_id;
        left_to_right.child_frame_id = config_.right.frame_id;
        left_to_right.transform = transform_message(inverse_rigid_transform(*right.t_cn_cnm1));
        transforms.push_back(std::move(left_to_right));
    }
    if (!transforms.empty())
    {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        static_tf_broadcaster_->sendTransform(transforms);
    }
}

void CameraMppPublisher::validate_config()
{
    detail::validate_decoder_input_pool(config_.decoder_input_pool, config_.decode_enabled);
    if (config_.input_type != "uvc" && config_.input_type != "file")
    {
        throw std::runtime_error("input.type must be uvc or file");
    }
    if (!config_.decode_enabled && !config_.compressed_enabled)
    {
        throw std::runtime_error("camera.decode=false requires streams.compressed");
    }
    if (uvc_device_id_ < 0 || uvc_device_id_ > 15 || uvc_channel_id_ < 0 || uvc_channel_id_ > 15 ||
        vdec_channel_id_ < 0 || vdec_channel_id_ > 63)
    {
        throw std::runtime_error("invalid MPP channel ID override");
    }
    if (config_.decode_enabled && config_.decoded_format != "nv12")
    {
        throw std::runtime_error("this MPP pipeline currently supports NV12 decoder output");
    }
    if (is_file_input())
    {
        if (config_.input_format != "h264" && config_.input_format != "h265" &&
            config_.input_format != "mjpeg")
        {
            throw std::runtime_error(
                "file input requires camera.input_format h264, h265, or mjpeg");
        }
        if (config_.input_file.empty() || !std::filesystem::is_regular_file(config_.input_file))
        {
            throw std::runtime_error("input.file must name an existing MP4 file");
        }
        if (config_.input_file.size() >= DEMUX_URL_MAX_LEN)
        {
            throw std::runtime_error("input.file path exceeds MPP limit");
        }
    }
    else
    {
        if (config_.device.empty())
        {
            throw std::runtime_error("camera.device is required for UVC input");
        }
        if (config_.input_format != "mjpeg")
        {
            throw std::runtime_error("UVC input currently supports only MJPEG");
        }
    }
    if (config_.width <= 0 || config_.height <= 0 || config_.fps <= 0 ||
        config_.max_exported_pairs <= 0 || config_.decoder_buffer_count < 0 ||
        config_.decoder_buffer_count > 64)
    {
        throw std::runtime_error("invalid camera or buffer values in JSON");
    }
    const auto valid_nv12_output = [](const StreamConfig& stream)
    {
        return stream.output_width > 0 && stream.output_height > 0 &&
            stream.output_width % 2 == 0 && stream.output_height % 2 == 0;
    };
    if (config_.full_only && (!config_.decode_enabled || !config_.full_enabled))
    {
        throw std::runtime_error("full_only requires decoded full stream");
    }
    if (config_.decode_enabled && !config_.full_only)
    {
        if (!region_is_valid(config_.left.crop, config_.width, config_.height) ||
            !region_is_valid(config_.right.crop, config_.width, config_.height))
        {
            throw std::runtime_error("stream crop is outside the input frame");
        }
        if (config_.left.crop.width != config_.right.crop.width ||
            config_.left.crop.height != config_.right.crop.height)
        {
            throw std::runtime_error("left and right stream dimensions must match");
        }
        if (!valid_nv12_output(config_.left) || !valid_nv12_output(config_.right))
        {
            throw std::runtime_error("stereo output dimensions must be positive and even for NV12");
        }
        if (config_.left.output_width != config_.right.output_width ||
            config_.left.output_height != config_.right.output_height)
        {
            throw std::runtime_error("left and right output dimensions must match");
        }
    }
    if (config_.full_enabled &&
        (!region_is_valid(config_.full.crop, config_.width, config_.height) ||
            config_.full.crop.x != 0 || config_.full.crop.y != 0 ||
            config_.full.crop.width != config_.width || config_.full.crop.height != config_.height))
    {
        throw std::runtime_error("full stream crop must cover the complete decoded frame");
    }
    if (calibration_ && !config_.decode_enabled)
    {
        throw std::runtime_error("calibration publication requires camera.decode=true");
    }
    if ((config_.metadata_enabled || config_.imu_enabled) && !config_.decode_enabled)
    {
        throw std::runtime_error("embedded metadata and IMU parsing require camera.decode=true");
    }
    if (calibration_)
    {
        if (calibration_->cameras.size() < 2)
        {
            throw std::runtime_error("the stereo bridge requires Kalibr cam0 and cam1");
        }
        if (config_.left.camera_info_topic.empty() || config_.right.camera_info_topic.empty())
        {
            throw std::runtime_error("camera_info_topic is required when calibration is set");
        }
        const auto matches_stream =
            [](const camera_sdk::KalibrCamera& camera, const CropConfig& crop)
        { return camera.resolution[0] == crop.width && camera.resolution[1] == crop.height; };
        if (!matches_stream(calibration_->cameras[0], config_.left.crop) ||
            !matches_stream(calibration_->cameras[1], config_.right.crop))
        {
            throw std::runtime_error("Kalibr cam0/cam1 resolution does not match stream crops");
        }
    }
    if (config_.metadata_enabled)
    {
        if (config_.metadata_parser != "camera_sdk_embedded")
        {
            throw std::runtime_error("unsupported metadata parser");
        }
        if (!region_is_valid(config_.metadata_region, config_.width, config_.height) ||
            config_.metadata_region.width < 160 || config_.metadata_region.height < 768)
        {
            throw std::runtime_error("invalid embedded metadata region");
        }
    }
    if (config_.timestamp_source != "ros_clock" &&
        config_.timestamp_source != "embedded_exposure_start" &&
        config_.timestamp_source != "embedded_exposure_midpoint")
    {
        throw std::runtime_error("unsupported timestamp source");
    }
    if ((config_.timestamp_source == "embedded_exposure_start" ||
            config_.timestamp_source == "embedded_exposure_midpoint") &&
        !config_.metadata_enabled)
    {
        throw std::runtime_error("embedded exposure timestamp requires metadata.enabled=true");
    }
    if (config_.gpio_sync_enabled &&
        (is_file_input() || !config_.decode_enabled || !config_.metadata_enabled ||
            config_.timestamp_source == "ros_clock"))
    {
        throw std::runtime_error(
            "GPIO sync requires live decoded input and embedded exposure timestamps");
    }
    if (config_.compressed_enabled)
    {
        if (is_file_input())
        {
            throw std::runtime_error("compressed stream publication currently requires UVC input");
        }
        if (config_.input_format != "mjpeg")
        {
            throw std::runtime_error("compressed stream publication currently supports MJPEG");
        }
        if (config_.compressed.topic.empty() || config_.compressed.frame_id.empty())
        {
            throw std::runtime_error("compressed stream topic and frame_id are required");
        }
    }
    if (config_.imu_enabled && !config_.metadata_enabled)
    {
        throw std::runtime_error("embedded IMU requires metadata.enabled=true");
    }
    if (config_.imu_enabled && config_.imu_queue_depth <= 0)
    {
        throw std::runtime_error("imu.queue_depth must be positive");
    }
    if (config_.imu_enabled &&
        (!(config_.imu_nominal_frequency_hz > 0.0F) || config_.imu_nominal_frequency_hz > 5000.0F))
    {
        throw std::runtime_error("resolved IMU frequency must be in (0, 5000]");
    }
    if (config_.ahrs_enabled && config_.orientation_variance < 0.0F)
    {
        throw std::runtime_error("imu.ahrs.orientation_stddev_rad must be non-negative");
    }
}

} // namespace camera_io
