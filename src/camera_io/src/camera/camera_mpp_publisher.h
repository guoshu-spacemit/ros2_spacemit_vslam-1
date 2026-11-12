#ifndef CAMERA_MPP_PUBLISHER_H
#define CAMERA_MPP_PUBLISHER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "camera_mpp_config.h"
#include "common/lifecycle/async_lifecycle_node.h"
#include "mpp_msgs/msg/mpp_video_frame_desc.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "spacemit_camera/camera_clock.h"
#include "spacemit_camera/kalibr_calibration.h"
#include "std_msgs/msg/bool.hpp"
#include "sys/vb_type.h"

class ImuPublisher;
namespace my_sensor_msgs::msg
{
class Imu;
} // namespace my_sensor_msgs::msg

namespace tf2_ros
{
class StaticTransformBroadcaster;
} // namespace tf2_ros

namespace camera_sdk
{
struct EmbeddedMetadata;
class GpioExposure;
} // namespace camera_sdk

namespace camera_io
{

class CameraMppPublisher : public common::lifecycle::AsyncLifecycleNode
{
public:
    explicit CameraMppPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
        const std::string& node_name = "camera_io");
    ~CameraMppPublisher() override;

protected:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    static constexpr int kDemuxChn = 0;

    struct ExportedPair
    {
        UL left_handle{0};
        UL right_handle{0};
        UL full_handle{0};
    };

    struct PendingCompressed
    {
        VideoFrameInfo frame{};
        U64 token{0};
    };

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State&) override;
    void release_resources() noexcept;
    void require_ok(S32 result, const char* operation);

    virtual void prepare_configuration() {}
    virtual rclcpp::QoS publication_qos() const { return rclcpp::SensorDataQoS(); }

    sensor_msgs::msg::CameraInfo make_camera_info(
        const camera_sdk::KalibrCamera& calibration, const detail::StreamConfig& stream) const;
    void publish_calibration_transforms();
    void validate_config();

    bool is_file_input() const;
    bool direct_file_packets() const;
    const std::string& input_description() const;
    MppStreamCodecType decoder_codec() const;
    void open_decoder();
    void configure_input_pool(const MppNode& source, const MppNode& sink);
    virtual void open_pipeline();
    void open_eye_pool();
    void close_pipeline() noexcept;
    void stop_file_input() noexcept;
    void start_file_input_if_ready();
    bool send_file_packet(const StreamBufferInfo& stream);
    void file_packet_loop() noexcept;
    void compressed_capture_loop();
    std::size_t max_pending_compressed() const;
    std::optional<PendingCompressed> take_pending_compressed(U64 pts);
    void decode_once();
    void decode_loop();
    void stop_capture() noexcept;
    void reset_capture_stats();
    void report_capture_stats(const std::chrono::steady_clock::time_point now);
    bool crop_eyes(const VideoFrameInfo& source, VideoFrameInfo& first, VideoFrameInfo& second);
    void release_oldest_export();

    virtual void initialize_imu();
    virtual bool write_imu(my_sensor_msgs::msg::Imu& message, bool fused);
    virtual bool raw_imu_enabled() const;
    virtual bool fused_imu_enabled() const;
    std::optional<std::int64_t> gpio_clock_offset();
    std::optional<rclcpp::Time> process_decoded(const VideoFrameInfo& decoded,
        std::uint32_t descriptor_index,
        std::optional<std::int64_t> recorded_image_stamp = std::nullopt, bool publish_images = true,
        bool parse_metadata = true, bool publish_stereo = true);
    void publish_descriptor(rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>& publisher,
        const VideoFrameInfo& frame, U64 token, const rclcpp::Time& stamp,
        const std::string& frame_id, std::uint32_t descriptor_index);
    void publish_imu(const camera_sdk::EmbeddedMetadata& metadata);

    std::string config_file_;
    std::string input_file_override_;
    std::string file_packet_reader_;
    std::string file_start_ready_topic_;
    std::string device_override_;
    int width_override_{0};
    int height_override_{0};
    int fps_override_{0};
    int uvc_device_id_{0};
    int uvc_channel_id_{0};
    int vdec_channel_id_{0};
    detail::BridgeConfig config_;
    std::uint32_t input_frame_index_{0};
    std::uint32_t decoded_frame_index_{0};
    std::uint64_t last_imu_timestamp_ns_{0};
    std::uint64_t last_image_timestamp_ns_{0};
    std::uint64_t imu_duplicate_count_{0};
    std::uint64_t imu_backward_count_{0};
    std::uint64_t image_timestamp_regression_count_{0};
    std::uint64_t imu_metadata_sample_count_{0};
    std::uint64_t imu_invalid_sample_count_{0};
    std::uint64_t imu_raw_write_successes_{0};
    std::uint64_t imu_raw_write_failures_{0};
    std::uint64_t imu_fused_write_failures_{0};
    camera_sdk::CameraClock clock_;
    std::unique_ptr<camera_sdk::GpioExposure> gpio_sync_;
    std::unique_ptr<camera_sdk::StartupAlignment> startup_alignment_;
    std::int64_t gpio_ros_offset_ns_{0};
    std::string gpio_sync_status_;
    std::optional<std::uint32_t> gpio_last_imu_raw_;
    std::uint64_t metadata_timestamp_rejection_count_{0};

    bool sys_initialized_{false};
    bool vb_initialized_{false};
    bool uvc_initialized_{false};
    bool demux_initialized_{false};
    bool vdec_initialized_{false};
    bool demux_created_{false};
    bool demux_started_{false};
    std::atomic<bool> file_start_ready_{true};
    std::atomic<bool> file_playback_active_{false};
    std::mutex file_start_mutex_;
    bool uvc_created_{false};
    bool uvc_device_enabled_{false};
    bool uvc_channel_enabled_{false};
    bool vdec_created_{false};
    bool vdec_enabled_{false};
    bool demux_vdec_bound_{false};
    bool uvc_vdec_bound_{false};
    MppNode demux_source_{};
    MppNode vdec_sink_{};
    UL eye_pool_{0};
    std::deque<ExportedPair> exports_;
    std::deque<UL> compressed_exports_;
    std::mutex pending_mutex_;
    std::condition_variable pending_cv_;
    std::deque<PendingCompressed> pending_compressed_;

    std::unique_ptr<ImuPublisher> imu_raw_pub_;
    std::unique_ptr<ImuPublisher> imu_fused_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_ros_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_fused_ros_pub_;
    std::unique_ptr<camera_sdk::MahonyAhrs> ahrs_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr left_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr right_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr full_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr compressed_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr file_start_ready_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
    std::optional<camera_sdk::KalibrCalibration> calibration_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::atomic<bool> capture_running_{false};
    std::thread compressed_thread_;
    std::thread decode_thread_;
    std::thread file_packet_thread_;
    std::mutex file_packet_mutex_;
    std::condition_variable file_packet_cv_;
    bool file_packet_started_{false};
    std::atomic<std::uint64_t> file_packets_submitted_{0};
    std::atomic<bool> file_packets_read_complete_{false};
    std::atomic<bool> file_packet_error_{false};
    std::chrono::steady_clock::time_point capture_stats_start_;
    std::chrono::steady_clock::time_point capture_stats_last_report_;
    std::uint64_t capture_frames_{0};
    std::uint64_t capture_failures_{0};
    std::atomic<std::uint64_t> compressed_capture_frames_{0};
    std::atomic<std::uint64_t> compressed_capture_failures_{0};
    std::atomic<std::uint64_t> compressed_correlation_drops_{0};
    std::uint64_t vdec_wait_ns_{0};
    std::uint64_t process_ns_{0};
    std::uint64_t release_ns_{0};
};

} // namespace camera_io

#endif // CAMERA_MPP_PUBLISHER_H
