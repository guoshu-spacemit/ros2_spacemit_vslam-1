/**
 * @file camera_mpp_publisher.cpp
 * @brief Publish camera DMA-BUF descriptors and synchronized image/IMU metadata.
 */
#include "camera_mpp_publisher.h"

#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <stdexcept>

#include "ImuPublisher.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "spacemit_camera/gpio_exposure.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace camera_io
{

CameraMppPublisher::CameraMppPublisher(
    const rclcpp::NodeOptions& options, const std::string& node_name)
    : AsyncLifecycleNode(node_name, options)
{
    config_file_ = declare_parameter<std::string>("config_file", "");
    input_file_override_ = declare_parameter<std::string>("input_file", "");
    file_packet_reader_ = declare_parameter<std::string>("file_packet_reader", "mpp");
    device_override_ = declare_parameter<std::string>("device", "");
    width_override_ = declare_parameter<int>("width", 0);
    height_override_ = declare_parameter<int>("height", 0);
    fps_override_ = declare_parameter<int>("fps", 0);
    uvc_device_id_ = declare_parameter<int>("uvc_device_id", 0);
    uvc_channel_id_ = declare_parameter<int>("uvc_channel_id", 0);
    vdec_channel_id_ = declare_parameter<int>("vdec_channel_id", 0);
    file_start_ready_topic_ = declare_parameter<std::string>("file_start_ready_topic", "");
}

CameraMppPublisher::~CameraMppPublisher() { release_resources(); }

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_configure(const rclcpp_lifecycle::State&)
{
    try
    {
        config_file_ = get_parameter("config_file").as_string();
        if (config_file_.empty())
        {
            throw std::runtime_error("config_file parameter is required");
        }
        fps_override_ = static_cast<int>(get_parameter("fps").as_int());
        config_ = detail::load_config(config_file_, fps_override_);
        input_file_override_ = get_parameter("input_file").as_string();
        file_packet_reader_ = get_parameter("file_packet_reader").as_string();
        device_override_ = get_parameter("device").as_string();
        width_override_ = static_cast<int>(get_parameter("width").as_int());
        height_override_ = static_cast<int>(get_parameter("height").as_int());
        uvc_device_id_ = static_cast<int>(get_parameter("uvc_device_id").as_int());
        uvc_channel_id_ = static_cast<int>(get_parameter("uvc_channel_id").as_int());
        vdec_channel_id_ = static_cast<int>(get_parameter("vdec_channel_id").as_int());
        file_start_ready_topic_ = get_parameter("file_start_ready_topic").as_string();
        if (!input_file_override_.empty())
        {
            config_.input_type = "file";
            config_.input_file = std::filesystem::weakly_canonical(input_file_override_).string();
        }
        if (!device_override_.empty())
        {
            config_.device = device_override_;
        }
        if (width_override_ > 0)
        {
            config_.width = width_override_;
        }
        if (height_override_ > 0)
        {
            config_.height = height_override_;
        }
        if (!config_.calibration_file.empty())
        {
            calibration_ = camera_sdk::load_kalibr_calibration(config_.calibration_file);
        }
        prepare_configuration();
        if (file_packet_reader_ != "mpp" && file_packet_reader_ != "libavformat")
            throw std::runtime_error("file_packet_reader must be mpp or libavformat");
        if (direct_file_packets())
        {
            if (!is_file_input() || config_.input_format != "mjpeg" || !config_.decode_enabled)
                throw std::runtime_error(
                    "libavformat packet reader requires decoded MJPEG file input");
        }
        validate_config();

        file_start_ready_.store(file_start_ready_topic_.empty());
        if (is_file_input() && !file_start_ready_topic_.empty())
        {
            const auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            file_start_ready_sub_ =
                create_subscription<std_msgs::msg::Bool>(file_start_ready_topic_, ready_qos,
                    [this](const std_msgs::msg::Bool::SharedPtr message)
                    {
                        if (!message->data)
                        {
                            return;
                        }
                        file_start_ready_.store(true);
                        try
                        {
                            start_file_input_if_ready();
                        }
                        catch (const std::exception& error)
                        {
                            RCLCPP_ERROR(get_logger(), "playback ready-event start failed: %s",
                                error.what());
                        }
                    });
        }

        if (config_.decode_enabled && !config_.full_only)
        {
            left_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                config_.left.topic, publication_qos());
            right_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                config_.right.topic, publication_qos());
        }
        if (config_.full_enabled)
        {
            full_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                config_.full.topic, publication_qos());
        }
        if (config_.compressed_enabled)
        {
            compressed_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                config_.compressed.topic, publication_qos());
        }
        if (calibration_ && config_.decode_enabled)
        {
            left_camera_info_ = make_camera_info(calibration_->cameras[0], config_.left);
            right_camera_info_ = make_camera_info(calibration_->cameras[1], config_.right);
            left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                config_.left.camera_info_topic, publication_qos());
            right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                config_.right.camera_info_topic, publication_qos());
            publish_calibration_transforms();
        }
        initialize_imu();
        open_pipeline();

        RCLCPP_INFO(get_logger(),
            "configured %s: input=%s (%s) %dx%d@%d -> left=%s %dx%d "
            "right=%s %dx%d "
            "timestamp=%s imu_raw=%s imu_fused=%s imu_rate=%.1fHz "
            "imu_domain=%d(%s) calibration=%s",
            config_file_.c_str(), config_.input_type.c_str(), input_description().c_str(),
            config_.width, config_.height, config_.fps, config_.left.topic.c_str(),
            config_.left.output_width, config_.left.output_height, config_.right.topic.c_str(),
            config_.right.output_width, config_.right.output_height,
            config_.timestamp_source.c_str(),
            config_.imu_enabled ? config_.imu_raw_dds_topic.c_str() : "disabled",
            config_.ahrs_enabled ? config_.imu_fused_dds_topic.c_str() : "disabled",
            config_.imu_enabled ? config_.imu_nominal_frequency_hz : 0.0F, config_.domain_id,
            config_.domain_id_from_environment ? "ROS_DOMAIN_ID" : "config",
            calibration_ ? config_.calibration_file.c_str() : "disabled");
        return CallbackReturn::SUCCESS;
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
        release_resources();
        return CallbackReturn::FAILURE;
    }
}

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_activate(const rclcpp_lifecycle::State&)
{
    if (config_.gpio_sync_enabled)
    {
        try
        {
            if (get_parameter("use_sim_time").as_bool())
                throw std::runtime_error("GPIO synchronization requires system ROS time");
            gpio_sync_ = std::make_unique<camera_sdk::GpioExposure>(
                config_.gpio_chip, config_.gpio_line, config_.gpio_active_low, config_.gpio_match);
            auto offset = gpio_clock_offset();
            if (!offset)
                throw std::runtime_error("unable to sample ROS/monotonic clocks within 100us");
            gpio_ros_offset_ns_ = *offset;
            if (config_.startup_alignment.enabled)
            {
                startup_alignment_ = std::make_unique<camera_sdk::StartupAlignment>(
                    config_.startup_alignment, camera_sdk::open_v4l2_exposure(config_.device),
                    [this](const std::string& message)
                    { RCLCPP_INFO(get_logger(), "GPIO startup alignment: %s", message.c_str()); });
            }
        }
        catch (const std::exception& error)
        {
            startup_alignment_.reset();
            gpio_sync_.reset();
            RCLCPP_ERROR(get_logger(), "GPIO activation failed: %s", error.what());
            return CallbackReturn::FAILURE;
        }
    }
    reset_capture_stats();
    if (is_file_input())
    {
        file_playback_active_.store(true);
        try
        {
            start_file_input_if_ready();
        }
        catch (const std::exception& error)
        {
            file_playback_active_.store(false);
            capture_running_.store(false);
            stop_capture();
            RCLCPP_ERROR(get_logger(), "playback activation failed: %s", error.what());
            return CallbackReturn::FAILURE;
        }
    }
    else
    {
        capture_running_.store(true);
        if (config_.compressed_enabled)
        {
            compressed_thread_ = std::thread(&CameraMppPublisher::compressed_capture_loop, this);
        }
        if (config_.decode_enabled)
        {
            decode_thread_ = std::thread(&CameraMppPublisher::decode_loop, this);
        }
    }
    const char* pipeline = "asynchronous compressed UVC passthrough";
    if (config_.decode_enabled && config_.compressed_enabled)
    {
        pipeline = "asynchronous UVC/VDEC dual-output pipeline";
    }
    else if (config_.decode_enabled)
    {
        pipeline = "event-driven VDEC output";
    }
    RCLCPP_INFO(get_logger(), "%s activated (%s%s)",
        is_file_input() ? "MP4 playback" : "camera capture", pipeline,
        is_file_input() && !file_start_ready_.load() ? ", waiting for ready event" : "");
    return CallbackReturn::SUCCESS;
}

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_deactivate(const rclcpp_lifecycle::State&)
{
    file_playback_active_.store(false);
    stop_file_input();
    stop_capture();
    RCLCPP_INFO(
        get_logger(), "%s deactivated", is_file_input() ? "MP4 playback" : "camera capture");
    return CallbackReturn::SUCCESS;
}

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_cleanup(const rclcpp_lifecycle::State&)
{
    release_resources();
    return CallbackReturn::SUCCESS;
}

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_shutdown(const rclcpp_lifecycle::State&)
{
    release_resources();
    return CallbackReturn::SUCCESS;
}

CameraMppPublisher::CallbackReturn CameraMppPublisher::on_error(const rclcpp_lifecycle::State&)
{
    release_resources();
    return CallbackReturn::SUCCESS;
}

void CameraMppPublisher::release_resources() noexcept
{
    stop_capture();
    close_pipeline();
    static_tf_broadcaster_.reset();
    imu_fused_pub_.reset();
    imu_raw_pub_.reset();
    imu_fused_ros_pub_.reset();
    imu_raw_ros_pub_.reset();
    ahrs_.reset();
    left_info_pub_.reset();
    right_info_pub_.reset();
    left_pub_.reset();
    right_pub_.reset();
    full_pub_.reset();
    compressed_pub_.reset();
    file_start_ready_sub_.reset();
    calibration_.reset();
}

void CameraMppPublisher::require_ok(S32 result, const char* operation)
{
    if (result != 0)
    {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
    }
}

} // namespace camera_io

RCLCPP_COMPONENTS_REGISTER_NODE(camera_io::CameraMppPublisher)
