/**
 * @file esdf_drm_node.cpp
 * @brief Fuse image-time poses and depth descriptors into TSDF/ESDF display output.
 */
#include "esdf_drm/display_gravity_seed.h"
#include "esdf_drm/esdf_update_pipeline.h"
#include "esdf_drm/exact_pair_queue.h"
#include "esdf_drm/gles_render_pipeline.h"
#include "esdf_drm/local_map_window.h"
#include "esdf_drm/mouse_input.h"

#include "drm_recorder.h"
#include "render_output_buffer.h"
#include "render_output_layout.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/sys_api.h>
#include <sys/vb_api.h>
#include <v2d/v2d_api.h>

#include <unistd.h>

#ifdef SUCCESS
#undef SUCCESS
#endif
#ifdef FAILURE
#undef FAILURE
#endif

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "esdf_drm/map_updates.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/msg/dma_depth_frame_desc.hpp>
#include <common/msg/feature_point_array.hpp>

namespace esdf_drm
{

using PoseMessage = nav_msgs::msg::Odometry;
using DepthMessage = common::msg::DmaDepthFrameDesc;
using PointCloudMessage = sensor_msgs::msg::PointCloud2;
using GravityMessage = geometry_msgs::msg::Vector3Stamped;
using RenderFrameMessage = mpp_msgs::msg::MppVideoFrameDesc;
using FeatureMessage = common::msg::FeaturePointArray;

constexpr std::uint32_t kDepthEncodingFloat32Meters = 1;
constexpr std::uint32_t kRgbEncodingRgb8 = 1;

std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(stamp.nanosec);
}

struct Vec3
{
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

struct InteractiveViewState
{
    Vec3 target{};
    float distance{3.2F};
    float yaw{0.0F};
    float pitch{0.0F};
    bool active{false};
};

Vec3 add(const Vec3& left, const Vec3& right)
{
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 subtract(const Vec3& left, const Vec3& right)
{
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 scale(const Vec3& value, float factor)
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

float dot(const Vec3& left, const Vec3& right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec3 cross(const Vec3& left, const Vec3& right)
{
    return {left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x};
}

Vec3 normalize(const Vec3& value)
{
    const float length = std::sqrt(dot(value, value));
    return length > 1.0e-6F ? scale(value, 1.0F / length) : Vec3{0.0F, 0.0F, 1.0F};
}

esdf_drm::mapping::Pose gravity_aligned_from_world(const Vec3& gravity_world)
{
    const Vec3 down = normalize(gravity_world);
    const Vec3 up = scale(down, -1.0F);
    Vec3 x_axis = subtract({1.0F, 0.0F, 0.0F}, scale(up, up.x));
    if (dot(x_axis, x_axis) < 1.0e-6F)
    {
        x_axis = subtract({0.0F, 1.0F, 0.0F}, scale(up, up.y));
    }
    x_axis = normalize(x_axis);
    const Vec3 y_axis = normalize(cross(up, x_axis));
    esdf_drm::mapping::Pose result;
    result.rotation = {
        x_axis.x, x_axis.y, x_axis.z, y_axis.x, y_axis.y, y_axis.z, up.x, up.y, up.z};
    result.translation = {0.0F, 0.0F, 0.0F};
    return result;
}

std::array<float, 16> multiply(
    const std::array<float, 16>& left, const std::array<float, 16>& right)
{
    std::array<float, 16> result{};
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int inner = 0; inner < 4; ++inner)
            {
                result[static_cast<std::size_t>(column * 4 + row)] +=
                    left[static_cast<std::size_t>(inner * 4 + row)] *
                    right[static_cast<std::size_t>(column * 4 + inner)];
            }
        }
    }
    return result;
}

// The optical frame uses +Z forward and +Y down. Return a column-major GLES
// projection from world coordinates, with Y flipped for the render target.
std::array<float, 16> MakeFollowViewProjection(
    const esdf_drm::mapping::Pose& world_from_camera, float aspect_ratio)
{
    const Vec3 position{world_from_camera.translation[0], world_from_camera.translation[1],
        world_from_camera.translation[2]};
    const Vec3 forward = normalize({world_from_camera.rotation[2], world_from_camera.rotation[5],
        world_from_camera.rotation[8]});
    const Vec3 down = normalize({world_from_camera.rotation[1], world_from_camera.rotation[4],
        world_from_camera.rotation[7]});
    const Vec3 up = scale(down, -1.0F);
    constexpr float kFollowDistanceM = 2.2F;
    constexpr float kFollowHeightM = 0.8F;
    constexpr float kLookAheadDistanceM = 0.8F;
    const Vec3 eye =
        add(subtract(position, scale(forward, kFollowDistanceM)), scale(up, kFollowHeightM));
    const Vec3 target = add(position, scale(forward, kLookAheadDistanceM));
    const Vec3 view_forward = normalize(subtract(target, eye));
    const Vec3 side = normalize(cross(view_forward, up));
    const Vec3 view_up = cross(side, view_forward);
    const std::array<float, 16> view = {side.x, view_up.x, -view_forward.x, 0.0F, side.y, view_up.y,
        -view_forward.y, 0.0F, side.z, view_up.z, -view_forward.z, 0.0F, -dot(side, eye),
        -dot(view_up, eye), dot(view_forward, eye), 1.0F};
    constexpr float kNear = 0.05F;
    constexpr float kFar = 100.0F;
    constexpr float kVerticalFieldOfViewRadians = 1.0471975512F;
    const float focal = 1.0F / std::tan(kVerticalFieldOfViewRadians * 0.5F);
    const std::array<float, 16> projection = {focal / aspect_ratio, 0.0F, 0.0F, 0.0F, 0.0F, -focal,
        0.0F, 0.0F, 0.0F, 0.0F, (kFar + kNear) / (kNear - kFar), -1.0F, 0.0F, 0.0F,
        (2.0F * kFar * kNear) / (kNear - kFar), 0.0F};
    return multiply(projection, view);
}

std::array<float, 16> MakeInteractiveViewProjection(
    const InteractiveViewState& state, float aspect_ratio)
{
    const float horizontal_distance = state.distance * std::cos(state.pitch);
    const Vec3 eye{state.target.x + horizontal_distance * std::cos(state.yaw),
        state.target.y + horizontal_distance * std::sin(state.yaw),
        state.target.z + state.distance * std::sin(state.pitch)};
    const Vec3 view_forward = normalize(subtract(state.target, eye));
    const Vec3 side = normalize(cross(view_forward, {0.0F, 0.0F, 1.0F}));
    const Vec3 view_up = cross(side, view_forward);
    const std::array<float, 16> view = {side.x, view_up.x, -view_forward.x, 0.0F, side.y, view_up.y,
        -view_forward.y, 0.0F, side.z, view_up.z, -view_forward.z, 0.0F, -dot(side, eye),
        -dot(view_up, eye), dot(view_forward, eye), 1.0F};
    constexpr float kNear = 0.05F;
    constexpr float kFar = 100.0F;
    constexpr float kVerticalFieldOfViewRadians = 1.0471975512F;
    const float focal = 1.0F / std::tan(kVerticalFieldOfViewRadians * 0.5F);
    const std::array<float, 16> projection = {focal / aspect_ratio, 0.0F, 0.0F, 0.0F, 0.0F, -focal,
        0.0F, 0.0F, 0.0F, 0.0F, (kFar + kNear) / (kNear - kFar), -1.0F, 0.0F, 0.0F,
        (2.0F * kFar * kNear) / (kNear - kFar), 0.0F};
    return multiply(projection, view);
}

std::array<float, 16> slice_view_projection(
    const esdf_drm::mapping::Pose& map_from_camera, float aspect_ratio, float half_view_height)
{
    constexpr float kViewHeight = 5.0F;
    constexpr float kNear = 0.05F;
    constexpr float kFar = 100.0F;
    const float half_view_width = half_view_height * aspect_ratio;
    const std::array<float, 16> view = {
        1.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        1.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        1.0F,
        0.0F,
        -map_from_camera.translation[0],
        -map_from_camera.translation[1],
        -kViewHeight,
        1.0F,
    };
    const std::array<float, 16> projection = {
        1.0F / half_view_width,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        -1.0F / half_view_height,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        -2.0F / (kFar - kNear),
        0.0F,
        0.0F,
        0.0F,
        -(kFar + kNear) / (kFar - kNear),
        1.0F,
    };
    return multiply(projection, view);
}

esdf_drm::mapping::Pose transform_from_pose(const PoseMessage& message)
{
    const auto& q = message.pose.pose.orientation;
    const float x = static_cast<float>(q.x);
    const float y = static_cast<float>(q.y);
    const float z = static_cast<float>(q.z);
    const float w = static_cast<float>(q.w);
    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    const float inverse_norm = norm > 1.0e-8F ? 1.0F / norm : 1.0F;
    const float nx = x * inverse_norm;
    const float ny = y * inverse_norm;
    const float nz = z * inverse_norm;
    const float nw = w * inverse_norm;
    esdf_drm::mapping::Pose result;
    result.rotation = {1.0F - 2.0F * (ny * ny + nz * nz), 2.0F * (nx * ny - nz * nw),
        2.0F * (nx * nz + ny * nw), 2.0F * (nx * ny + nz * nw), 1.0F - 2.0F * (nx * nx + nz * nz),
        2.0F * (ny * nz - nx * nw), 2.0F * (nx * nz - ny * nw), 2.0F * (ny * nz + nx * nw),
        1.0F - 2.0F * (nx * nx + ny * ny)};
    result.translation = {static_cast<float>(message.pose.pose.position.x),
        static_cast<float>(message.pose.pose.position.y),
        static_cast<float>(message.pose.pose.position.z)};
    return result;
}

esdf_drm::mapping::Pose transform_from_depth_message(const DepthMessage& message)
{
    esdf_drm::mapping::Pose result;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            result.rotation[static_cast<std::size_t>(row * 3 + column)] = static_cast<float>(
                message.left_from_depth[static_cast<std::size_t>(row * 4 + column)]);
        }
        result.translation[static_cast<std::size_t>(row)] =
            static_cast<float>(message.left_from_depth[static_cast<std::size_t>(row * 4 + 3)]);
    }
    return result;
}

esdf_drm::mapping::Pose compose(
    const esdf_drm::mapping::Pose& left, const esdf_drm::mapping::Pose& right)
{
    esdf_drm::mapping::Pose result;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            result.rotation[static_cast<std::size_t>(row * 3 + column)] =
                left.rotation[static_cast<std::size_t>(row * 3)] *
                    right.rotation[static_cast<std::size_t>(column)] +
                left.rotation[static_cast<std::size_t>(row * 3 + 1)] *
                    right.rotation[static_cast<std::size_t>(3 + column)] +
                left.rotation[static_cast<std::size_t>(row * 3 + 2)] *
                    right.rotation[static_cast<std::size_t>(6 + column)];
        }
        result.translation[static_cast<std::size_t>(row)] =
            left.rotation[static_cast<std::size_t>(row * 3)] * right.translation[0] +
            left.rotation[static_cast<std::size_t>(row * 3 + 1)] * right.translation[1] +
            left.rotation[static_cast<std::size_t>(row * 3 + 2)] * right.translation[2] +
            left.translation[static_cast<std::size_t>(row)];
    }
    return result;
}

class ImportedBuffer
{
public:
    explicit ImportedBuffer(std::uint64_t token)
    {
        if (token == 0 || VB_Import(token, &handle_) != 0 || handle_ == 0)
        {
            throw std::runtime_error("VB_Import failed");
        }
        if (VB_GetVirAddr(handle_, &address_) != 0 || address_ == nullptr)
        {
            (void)VB_ReleaseBuffer(handle_);
            handle_ = 0;
            throw std::runtime_error("VB_GetVirAddr failed");
        }
    }

    ~ImportedBuffer()
    {
        if (handle_ != 0)
        {
            (void)VB_ReleaseBuffer(handle_);
        }
    }

    ImportedBuffer(const ImportedBuffer&) = delete;
    ImportedBuffer& operator=(const ImportedBuffer&) = delete;

    const std::uint8_t* data() const { return static_cast<const std::uint8_t*>(address_); }

private:
    UL handle_{0};
    void* address_{nullptr};
};

struct RetainedDepth
{
    explicit RetainedDepth(std::shared_ptr<const DepthMessage> message)
        : descriptor(std::move(message)), depth(descriptor->token), rgb(descriptor->rgb_token)
    {
    }

    std::shared_ptr<const DepthMessage> descriptor;
    ImportedBuffer depth;
    ImportedBuffer rgb;
};

using PairQueue =
    ExactPairQueue<std::shared_ptr<const PoseMessage>, std::shared_ptr<RetainedDepth>>;

struct Pair
{
    std::shared_ptr<const PoseMessage> pose;
    std::shared_ptr<const DepthMessage> depth;
    std::shared_ptr<RetainedDepth> retained;
};

class RobotEsdfDrmNode final : public rclcpp::Node
{
public:
    explicit RobotEsdfDrmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("esdf_drm", options)
    {
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/vio/pose/image");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/las2/depth_desc");
        gravity_topic_ = declare_parameter<std::string>("gravity_topic", "/vio/gravity");
        preview_overlay_topic_ =
            declare_parameter<std::string>("preview_overlay_topic", "/las2/preview/frame_desc");
        feature_panel_enabled_ = declare_parameter<bool>("feature_panel_enabled", false);
        feature_panel_age_max_frames_ = declare_parameter<int>("feature_panel_age_max_frames", 14);
        if (feature_panel_age_max_frames_ < 2 || feature_panel_age_max_frames_ > 1000000)
            throw std::runtime_error("feature_panel_age_max_frames must be in [2, 1000000]");
        feature_panel_image_topic_ =
            declare_parameter<std::string>("feature_panel_image_topic", "/camera/left/frame_desc");
        feature_panel_topic_ =
            declare_parameter<std::string>("feature_panel_topic", "/vio/features");
        world_frame_ = declare_parameter<std::string>("world_frame", "vio_world");
        pointcloud_topic_ =
            declare_parameter<std::string>("pointcloud_topic", "/esdf/colored_pointcloud");
        pair_buffer_depth_ = declare_parameter<int>("pair_buffer_depth", 64);
        queue_depth_ = declare_parameter<int>("queue_depth", 2);
        voxel_size_ = declare_parameter<double>("voxel_size_m", 0.05);
        truncation_distance_ = declare_parameter<double>("truncation_distance_m", 0.15);
        max_integration_distance_ = declare_parameter<double>("max_integration_distance_m", 3.0);
        map_clearing_radius_m_ = declare_parameter<double>("map_clearing_radius_m", 0.0);
        trajectory_duration_s_ = declare_parameter<double>("render_trajectory_duration_s", 0.0);
        clear_map_rate_hz_ = declare_parameter<double>("clear_map_outside_radius_rate_hz", 1.0);
        invalid_depth_decay_factor_ =
            declare_parameter<double>("projective_tsdf_integrator_invalid_depth_decay_factor", 0.8);
        max_esdf_distance_ = declare_parameter<double>("max_esdf_distance_m", 2.0);
        esdf_update_hz_ = declare_parameter<double>("esdf_update_hz", 5.0);
        esdf_max_pending_blocks_ = declare_parameter<int>("esdf_max_pending_blocks", 16384);
        pointcloud_publish_period_ms_ = declare_parameter<int>("pointcloud_publish_period_ms", 500);
        pointcloud_max_points_ = declare_parameter<int>("pointcloud_max_points", 30000);
        pointcloud_min_weight_ = declare_parameter<double>("pointcloud_min_weight", 1.0);
        pointcloud_surface_band_m_ = declare_parameter<double>("pointcloud_surface_band_m", -1.0);
        publish_debug_pointcloud_ = declare_parameter<bool>("publish_debug_pointcloud", false);
        mapping_enabled_ = declare_parameter<bool>("mapping_enabled", true);
        enable_gles_drm_ = declare_parameter<bool>("enable_gles_drm", true);
        headless_render_ = declare_parameter<bool>("headless_render", false);
        publish_render_output_ = declare_parameter<bool>("publish_render_output", false);
        render_output_topic_ =
            declare_parameter<std::string>("render_output_topic", "/esdf/render/frame_desc");
        render_output_pool_depth_ = declare_parameter<int>("render_output_pool_depth", 6);
        const auto render_format = declare_parameter<std::string>("render_output_format", "nv12");
        if (render_format != "nv12" && render_format != "bgra")
        {
            throw std::invalid_argument("render_output_format must be nv12 or bgra");
        }
        render_output_bgra_ = render_format == "bgra";
        recorder_config_.enabled = declare_parameter<bool>("record_enabled", false);
        recorder_config_.use_source_timestamps =
            declare_parameter<bool>("record_use_source_timestamps", false);
        record_timestamp_topic_ = declare_parameter<std::string>("record_timestamp_topic", "");
        record_timestamp_use_frame_pts_ =
            declare_parameter<bool>("record_timestamp_use_frame_pts", true);
        recorder_config_.source_timestamp_origin_zero =
            declare_parameter<bool>("record_timestamp_origin_zero", false);
        recorder_config_.output_pattern = declare_parameter<std::string>(
            "record_output_pattern", "recordings/esdf_%Y%m%d_%H%M%S.mp4");
        recorder_config_.fps = declare_parameter<int>("record_fps", 60);
        recorder_config_.bitrate_bps = declare_parameter<int>("record_bitrate_bps", 8000000);
        recorder_config_.gop = declare_parameter<int>("record_gop", 60);
        recorder_config_.pool_depth = declare_parameter<int>("record_pool_depth", 8);
        recorder_config_.venc_channel = declare_parameter<int>("record_venc_channel", 2);
        recorder_config_.mux_channel = declare_parameter<int>("record_mux_channel", 2);
        record_wait_for_map_ = declare_parameter<bool>("record_wait_for_map", false);
        render_fps_ = declare_parameter<double>("render_fps", 60.0);
        render_device_ = declare_parameter<std::string>("render_device", "/dev/dri/renderD128");
        display_device_ = declare_parameter<std::string>("display_device", "");
        display_width_ = declare_parameter<int>("display_width", 1920);
        display_height_ = declare_parameter<int>("display_height", 1080);
        render_max_blocks_ = declare_parameter<int>("render_max_blocks", 8192);
        render_max_block_updates_per_frame_ =
            declare_parameter<int>("render_max_block_updates_per_frame", 0);
        render_max_points_per_block_ = declare_parameter<int>("render_max_points_per_block", 0);
        render_point_size_ = declare_parameter<double>("render_point_size", 8.0);
        render_slice_view_half_height_m_ =
            declare_parameter<double>("render_slice_view_half_height_m", 3.0);
        render_measure_gpu_time_ = declare_parameter<bool>("render_measure_gpu_time", false);
        render_async_page_flip_ = declare_parameter<bool>("render_async_page_flip", true);
        render_frustum_culling_ = declare_parameter<bool>("render_frustum_culling", true);
        render_tsdf_interpolated_points_ =
            declare_parameter<bool>("render_tsdf_interpolated_points", false);
        render_secondary_camera_indicator_ =
            declare_parameter<bool>("render_secondary_camera_indicator", true);
        render_secondary_camera_fov_degrees_ =
            declare_parameter<double>("render_secondary_camera_fov_degrees", 70.0);
        render_secondary_camera_fov_range_m_ =
            declare_parameter<double>("render_secondary_camera_fov_range_m", 1.5);
        render_secondary_camera_marker_radius_m_ =
            declare_parameter<double>("render_secondary_camera_marker_radius_m", 0.12);
        render_secondary_camera_marker_size_pixels_ =
            declare_parameter<double>("render_secondary_camera_marker_size_pixels", 18.0);
        mouse_navigation_enabled_ = declare_parameter<bool>("mouse_navigation_enabled", true);
        mouse_input_device_ = declare_parameter<std::string>("mouse_input_device", "");
        gravity_align_map_ = declare_parameter<bool>("gravity_align_map", true);
        const auto initial_gravity = declare_parameter<std::vector<double>>(
            "initial_gravity_camera", std::vector<double>{0.0, 0.0, 0.0});
        if (initial_gravity.size() != 3 ||
            !std::all_of(initial_gravity.begin(), initial_gravity.end(),
                [](double value) { return std::isfinite(value); }))
        {
            throw std::runtime_error("initial_gravity_camera must contain three finite values");
        }
        initial_gravity_camera_ = {static_cast<float>(initial_gravity[0]),
            static_cast<float>(initial_gravity[1]), static_cast<float>(initial_gravity[2])};
        const double initial_gravity_norm =
            std::sqrt(dot(initial_gravity_camera_, initial_gravity_camera_));
        if (initial_gravity_norm != 0.0 &&
            (!std::isfinite(initial_gravity_norm) || initial_gravity_norm < 1.0 ||
                initial_gravity_norm > 20.0))
        {
            throw std::runtime_error("initial_gravity_camera must be zero or have norm 1..20");
        }
        const auto display_imu_topic =
            declare_parameter<std::string>("display_gravity_imu_topic", "");
        const auto display_rotation = declare_parameter<std::vector<double>>(
            "display_gravity_imu_rotation", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
        const double display_shift =
            declare_parameter<double>("display_gravity_imu_time_shift_s", 0.0);
        if (display_rotation.size() != 9 || !std::isfinite(display_shift) ||
            std::abs(display_shift) > 1.0)
            throw std::runtime_error(
                "invalid display IMU rotation size or time shift outside [-1,1]s");
        std::copy(display_rotation.begin(), display_rotation.end(), display_imu_rotation_.begin());
        if (!valid_display_rotation(display_imu_rotation_))
            throw std::runtime_error("display_gravity_imu_rotation must be a proper rotation");
        if (!display_imu_topic.empty() && mapping_enabled_ && gravity_align_map_ &&
            initial_gravity_norm == 0.0)
        {
            const auto shift_ns = static_cast<std::int64_t>(std::llround(display_shift * 1.0e9));
            display_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(display_imu_topic,
                rclcpp::SensorDataQoS(),
                [this, shift_ns](sensor_msgs::msg::Imu::ConstSharedPtr message)
                {
                    std::lock_guard<std::mutex> lock(gravity_mutex_);
                    if (gravity_alignment_ready_)
                        return;
                    const auto& a = message->linear_acceleration;
                    const auto& g = message->angular_velocity;
                    // Invalid/moving input also clears a previously ready seed.
                    display_gravity_candidate_ =
                        display_gravity_seed_.add(stamp_ns(message->header.stamp) - shift_ns,
                            {a.x, a.y, a.z}, {g.x, g.y, g.z});
                });
            RCLCPP_INFO(get_logger(),
                "VO display gravity: quiet IMU window >=0.3s; shift=%.9fs; "
                "display only, no IMU pose fusion",
                display_shift);
        }
        follow_slice_height_ = declare_parameter<bool>("follow_slice_height", true);
        slice_thickness_m_ = declare_parameter<double>("slice_thickness_m", 0.05);
        slice_height_offset_m_ = declare_parameter<double>("slice_height_offset_m", 0.0);
        slice_min_z_m_ = declare_parameter<double>("slice_min_z_m", -0.25);
        slice_max_z_m_ = declare_parameter<double>("slice_max_z_m", 0.25);
        slice_output_z_m_ = declare_parameter<double>("slice_output_z_m", 0.0);
        if (pair_buffer_depth_ < 2 || pair_buffer_depth_ > 64 || queue_depth_ < 1 ||
            queue_depth_ > 32 || voxel_size_ <= 0.0 || pointcloud_publish_period_ms_ < 0 ||
            !std::isfinite(trajectory_duration_s_) || trajectory_duration_s_ < 0.0 ||
            trajectory_duration_s_ > 3600.0 || pointcloud_max_points_ < 1000 ||
            pointcloud_min_weight_ <= 0.0 || pointcloud_surface_band_m_ == 0.0 ||
            render_fps_ <= 0.0 || render_fps_ > 240.0 || display_width_ <= 0 ||
            display_height_ <= 0 || render_max_blocks_ < 64 ||
            render_max_block_updates_per_frame_ < 0 ||
            render_max_block_updates_per_frame_ > render_max_blocks_ ||
            render_max_points_per_block_ < 0 || render_max_points_per_block_ > 32768 ||
            render_point_size_ < 1.0 || render_point_size_ > 32.0 ||
            render_slice_view_half_height_m_ <= 0.0 ||
            render_secondary_camera_fov_degrees_ <= 0.0 ||
            render_secondary_camera_fov_degrees_ >= 180.0 ||
            render_secondary_camera_fov_range_m_ <= 0.0 ||
            render_secondary_camera_marker_radius_m_ <= 0.0 ||
            render_secondary_camera_marker_size_pixels_ <= 0.0 || esdf_update_hz_ <= 0.0 ||
            esdf_update_hz_ > 60.0 || esdf_max_pending_blocks_ < 64 ||
            slice_min_z_m_ > slice_max_z_m_ || slice_thickness_m_ <= 0.0 ||
            slice_thickness_m_ > 10.0 || render_output_pool_depth_ < 2 ||
            render_output_pool_depth_ > 32)
        {
            throw std::runtime_error("invalid ESDF point-cloud parameters");
        }
        if (invalid_depth_decay_factor_ > 1.0)
        {
            throw std::runtime_error("invalid TSDF invalid-depth decay factor");
        }
        if (recorder_config_.enabled && !enable_gles_drm_)
        {
            throw std::runtime_error("record_enabled requires enable_gles_drm=true");
        }
        if (!mapping_enabled_ && recorder_config_.enabled && record_wait_for_map_)
        {
            throw std::runtime_error("record_wait_for_map requires mapping_enabled=true");
        }
        if (!mapping_enabled_ && recorder_config_.enabled &&
            recorder_config_.use_source_timestamps && record_timestamp_topic_.empty())
        {
            throw std::runtime_error(
                "source-timed recording without mapping requires record_timestamp_topic");
        }

        if (SYS_Init() != 0)
        {
            throw std::runtime_error("SYS_Init failed for ESDF");
        }
        sys_initialized_ = true;
        if (VB_Init() != 0)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
            throw std::runtime_error("VB_Init failed for ESDF");
        }
        vb_initialized_ = true;

        if (publish_render_output_)
        {
            create_render_output_pool();
            render_output_pub_ = create_publisher<RenderFrameMessage>(
                render_output_topic_, rclcpp::SensorDataQoS().keep_last(4));
        }

        if (mapping_enabled_)
        {
            tsdf_params_.resolution_m = static_cast<float>(voxel_size_);
            tsdf_params_.surface_truncation_m = static_cast<float>(truncation_distance_);
            tsdf_params_.maximum_observation_distance_m =
                static_cast<float>(max_integration_distance_);
            tsdf_params_.invalid_observation_decay =
                static_cast<float>(invalid_depth_decay_factor_);
            local_map_ = std::make_unique<LocalMapWindow>(
                tsdf_params_, map_clearing_radius_m_, clear_map_rate_hz_);
            tsdf_ = &local_map_->tsdf();
            esdf_params_.maximum_distance_m = static_cast<float>(max_esdf_distance_);
        }

        pose_sub_ = create_subscription<PoseMessage>(pose_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RobotEsdfDrmNode::pose_callback, this, std::placeholders::_1));
        if (declare_parameter<bool>("performance_overlay", true))
        {
            performance_ = std::make_shared<PerformanceStats>();
            tracking_mode_sub_ = create_subscription<std_msgs::msg::String>(
                declare_parameter<std::string>("tracking_mode_topic", "/vio/tracking_mode"),
                rclcpp::SensorDataQoS().keep_last(1),
                [this](std_msgs::msg::String::ConstSharedPtr message)
                { performance_->tracking_mode.record(message->data); });
            las2_timing_sub_ = create_subscription<std_msgs::msg::Float64>(
                declare_parameter<std::string>("las2_timing_topic", "/las2/preprocess_gpu_ms"),
                rclcpp::SensorDataQoS().keep_last(64),
                [this](std_msgs::msg::Float64::ConstSharedPtr message)
                {
                    if (std::isfinite(message->data) && message->data >= 0.0)
                        performance_->las2.record(message->data);
                });
        }
        if (mapping_enabled_)
        {
            depth_sub_ = create_subscription<DepthMessage>(depth_topic_, rclcpp::SensorDataQoS(),
                std::bind(&RobotEsdfDrmNode::depth_callback, this, std::placeholders::_1));
            gravity_sub_ = create_subscription<GravityMessage>(gravity_topic_,
                rclcpp::SensorDataQoS().keep_last(8),
                std::bind(&RobotEsdfDrmNode::gravity_callback, this, std::placeholders::_1));
        }
        if (recorder_config_.enabled && recorder_config_.use_source_timestamps &&
            !record_timestamp_topic_.empty())
        {
            record_timestamp_sub_ = create_subscription<RenderFrameMessage>(record_timestamp_topic_,
                rclcpp::SensorDataQoS().keep_last(2),
                std::bind(
                    &RobotEsdfDrmNode::record_timestamp_callback, this, std::placeholders::_1));
        }
        if (enable_gles_drm_ && !preview_overlay_topic_.empty())
        {
            preview_overlay_sub_ = create_subscription<RenderFrameMessage>(preview_overlay_topic_,
                rclcpp::SensorDataQoS().keep_last(1),
                std::bind(
                    &RobotEsdfDrmNode::preview_overlay_callback, this, std::placeholders::_1));
        }
        if (enable_gles_drm_ && feature_panel_enabled_)
        {
            if (feature_panel_image_topic_.empty() || feature_panel_topic_.empty())
            {
                throw std::runtime_error("feature panel requires image and feature topics");
            }
            feature_panel_image_sub_ = create_subscription<RenderFrameMessage>(
                feature_panel_image_topic_, rclcpp::SensorDataQoS().keep_last(1),
                std::bind(
                    &RobotEsdfDrmNode::feature_panel_image_callback, this, std::placeholders::_1));
            feature_panel_sub_ = create_subscription<FeatureMessage>(feature_panel_topic_,
                rclcpp::SensorDataQoS().keep_last(1),
                std::bind(&RobotEsdfDrmNode::feature_panel_callback, this, std::placeholders::_1));
        }
        rclcpp::QoS pointcloud_qos(1);
        pointcloud_qos.best_effort();
        if (publish_debug_pointcloud_)
        {
            pointcloud_pub_ =
                create_publisher<PointCloudMessage>(pointcloud_topic_, pointcloud_qos);
        }

        start_pipelines();

        stopping_.store(false);
        if (mapping_enabled_)
        {
            mapping_thread_ = std::thread(&RobotEsdfDrmNode::mapping_loop, this);
        }
        start_mouse_input();
        RCLCPP_INFO(get_logger(),
            "ESDF ready: mode=%s pose=%s depth=%s cloud=%s RVV=auto cloud_period=%dms "
            "gles=%s render=%s@%.1fHz tsdf_geometry=%s "
            "decay=%.2f "
            "esdf=%s@%.1fHz gravity_align=%s gravity=%s slice_follow=%s "
            "headless=%s output=%s overlay=%s",
            mapping_enabled_ ? "mapping" : "vio-display", pose_topic_.c_str(),
            mapping_enabled_ ? depth_topic_.c_str() : "off", pointcloud_topic_.c_str(),
            pointcloud_publish_period_ms_, enable_gles_drm_ ? "on" : "off",
            mapping_enabled_ ? "tsdf_points_esdf_slice" : "trajectory", render_fps_,
            mapping_enabled_ ? "compute" : "off", invalid_depth_decay_factor_,
            mapping_enabled_ ? "async" : "off", esdf_update_hz_,
            mapping_enabled_ && gravity_align_map_ ? "on" : "off",
            mapping_enabled_ ? gravity_topic_.c_str() : "off",
            mapping_enabled_ && follow_slice_height_ ? "on" : "off",
            headless_render_ ? "on" : "off",
            publish_render_output_ ? render_output_topic_.c_str() : "off",
            preview_overlay_sub_ ? preview_overlay_topic_.c_str() : "off");
    }

    ~RobotEsdfDrmNode() override
    {
        mouse_input_.reset();
        stopping_.store(true);
        pair_cv_.notify_all();
        if (mapping_thread_.joinable())
        {
            mapping_thread_.join();
        }
        if (pair_queue_)
        {
            RCLCPP_INFO(get_logger(),
                "pair summary: depth=%" PRIu64 " matched=%" PRIu64 " late=%" PRIu64
                " unmatched_depth_evicted=%" PRIu64 " ready_dropped=%" PRIu64 " mapped=%" PRIu64,
                pair_queue_->received_second(), pair_queue_->matched(),
                pair_queue_->dropped_late_second(), pair_queue_->dropped_unmatched_second(),
                pair_queue_->dropped_ready(), mapped_frames_.load());
        }
        if (esdf_pipeline_)
        {
            esdf_pipeline_->stop();
            esdf_pipeline_.reset();
        }
        {
            std::lock_guard<std::mutex> lock(render_pipeline_lifetime_mutex_);
            if (render_pipeline_)
            {
                render_pipeline_->stop();
                render_pipeline_.reset();
            }
        }
        if (drm_recorder_)
        {
            drm_recorder_->stop();
            drm_recorder_.reset();
        }
        {
            std::lock_guard<std::mutex> lock(render_output_mutex_);
            while (!render_output_handles_.empty())
            {
                (void)VB_Unexport(render_output_handles_.front());
                render_output_handles_.pop_front();
            }
        }
        if (render_output_pool_ != 0)
        {
            (void)VB_DestroyPool(render_output_pool_);
            render_output_pool_ = 0;
        }
        tsdf_ = nullptr;
        local_map_.reset();
        if (vb_initialized_)
        {
            (void)VB_Exit();
            vb_initialized_ = false;
        }
        if (sys_initialized_)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
        }
    }

private:
    void start_mouse_input()
    {
        if (!enable_gles_drm_ || headless_render_ || !mouse_navigation_enabled_)
        {
            return;
        }
        auto input = std::make_unique<MouseInput>(
            mouse_input_device_, [this](const MouseInputEvent& event) { handle_mouse(event); });
        std::string selected_device;
        std::string error;
        if (!input->start(&selected_device, &error))
        {
            RCLCPP_WARN(get_logger(), "mouse navigation disabled: %s", error.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(),
            "mouse navigation ready: device=%s left=orbit right=pan wheel=zoom "
            "middle=reset",
            selected_device.c_str());
        mouse_input_ = std::move(input);
    }

    void initialize_interactive_view(const esdf_drm::mapping::Pose& world_from_camera)
    {
        const Vec3 position{world_from_camera.translation[0], world_from_camera.translation[1],
            world_from_camera.translation[2]};
        const Vec3 forward = normalize({world_from_camera.rotation[2],
            world_from_camera.rotation[5], world_from_camera.rotation[8]});
        const Vec3 down = normalize({world_from_camera.rotation[1], world_from_camera.rotation[4],
            world_from_camera.rotation[7]});
        const Vec3 up = scale(down, -1.0F);
        const Vec3 eye = add(subtract(position, scale(forward, 2.2F)), scale(up, 0.8F));
        interactive_view_.target = add(position, scale(forward, 0.8F));
        const Vec3 offset = subtract(eye, interactive_view_.target);
        interactive_view_.distance = std::max(0.2F, std::sqrt(dot(offset, offset)));
        interactive_view_.yaw = std::atan2(offset.y, offset.x);
        interactive_view_.pitch =
            std::asin(std::clamp(offset.z / interactive_view_.distance, -1.0F, 1.0F));
        interactive_view_.active = true;
    }

    void submit_current_view(const InteractiveViewState& state,
        const esdf_drm::mapping::Pose& world_from_camera, std::int64_t source_timestamp_ns)
    {
        std::lock_guard<std::mutex> pipeline_lock(render_pipeline_lifetime_mutex_);
        if (!render_pipeline_)
        {
            return;
        }
        const float viewport_width = mapping_enabled_ ? 0.5F * static_cast<float>(display_width_)
                                                        : static_cast<float>(display_width_);
        const float aspect_ratio = viewport_width / display_height_;
        if (!state.active)
        {
            render_pipeline_->SubmitDualView(
                MakeFollowViewProjection(world_from_camera, aspect_ratio),
                slice_view_projection(world_from_camera, aspect_ratio,
                    static_cast<float>(render_slice_view_half_height_m_)),
                world_from_camera, source_timestamp_ns);
            return;
        }
        esdf_drm::mapping::Pose slice_center = world_from_camera;
        slice_center.translation = {state.target.x, state.target.y, state.target.z};
        const float slice_half_height = static_cast<float>(render_slice_view_half_height_m_) *
            std::clamp(state.distance / 3.2F, 0.1F, 10.0F);
        render_pipeline_->SubmitDualView(MakeInteractiveViewProjection(state, aspect_ratio),
            slice_view_projection(slice_center, aspect_ratio, slice_half_height), world_from_camera,
            source_timestamp_ns);
    }

    void handle_mouse(const MouseInputEvent& event)
    {
        InteractiveViewState state;
        esdf_drm::mapping::Pose camera;
        std::int64_t source_timestamp_ns = 0;
        {
            std::lock_guard<std::mutex> lock(interactive_view_mutex_);
            if (!latest_render_pose_valid_)
            {
                return;
            }
            camera = latest_render_pose_;
            source_timestamp_ns = latest_render_pose_stamp_;
            if (event.reset)
            {
                interactive_view_ = {};
            }
            else
            {
                if (!event.left_down && !event.right_down && event.wheel == 0)
                {
                    return;
                }
                if (!interactive_view_.active)
                {
                    initialize_interactive_view(camera);
                }
                if (event.left_down)
                {
                    constexpr float kOrbitRadiansPerPixel = 0.006F;
                    constexpr float kMaxPitchRadians = 1.45F;
                    interactive_view_.yaw -=
                        static_cast<float>(event.relative_x) * kOrbitRadiansPerPixel;
                    interactive_view_.pitch = std::clamp(interactive_view_.pitch +
                            static_cast<float>(event.relative_y) * kOrbitRadiansPerPixel,
                        -kMaxPitchRadians, kMaxPitchRadians);
                }
                if (event.right_down)
                {
                    const float horizontal_distance =
                        interactive_view_.distance * std::cos(interactive_view_.pitch);
                    const Vec3 eye{interactive_view_.target.x +
                            horizontal_distance * std::cos(interactive_view_.yaw),
                        interactive_view_.target.y +
                            horizontal_distance * std::sin(interactive_view_.yaw),
                        interactive_view_.target.z +
                            interactive_view_.distance * std::sin(interactive_view_.pitch)};
                    const Vec3 view_forward = normalize(subtract(interactive_view_.target, eye));
                    const Vec3 side = normalize(cross(view_forward, {0.0F, 0.0F, 1.0F}));
                    const Vec3 view_up = cross(side, view_forward);
                    const float motion_scale = interactive_view_.distance * 0.0015F;
                    interactive_view_.target = add(interactive_view_.target,
                        add(scale(side, -static_cast<float>(event.relative_x) * motion_scale),
                            scale(view_up, static_cast<float>(event.relative_y) * motion_scale)));
                }
                if (event.wheel != 0)
                {
                    interactive_view_.distance = std::clamp(interactive_view_.distance *
                            std::exp(-static_cast<float>(event.wheel) * 0.12F),
                        0.2F, 50.0F);
                }
            }
            state = interactive_view_;
        }
        submit_current_view(state, camera, source_timestamp_ns);
    }

    void create_render_output_pool()
    {
        VbPoolCfg config{};
        config.u32BufSize =
            make_render_output_layout(display_width_, display_height_, render_output_bgra_)
                .stVFrame.u32TotalSize;
        config.u32BufCnt = static_cast<U32>(render_output_pool_depth_);
        config.eModId = MPP_ID_V2D;
        config.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
        render_output_pool_ = VB_CreatePool(&config);
        if (render_output_pool_ == 0)
        {
            throw std::runtime_error("VB_CreatePool failed for ESDF render output");
        }
        RCLCPP_INFO(get_logger(), "Render output: format=%s %dx%d pool=%d snapshot=%s",
            render_output_bgra_ ? "BGRA" : "NV12", display_width_, display_height_,
            render_output_pool_depth_, render_output_bgra_ ? "BGRA copy" : "BGRA-to-NV12");
    }

    VideoFrameInfo make_render_source(const esdf_drm::mapping::RenderFrame& source) const
    {
        VideoFrameInfo frame{};
        frame.eFrameType = FRAME_TYPE_COMMON;
        frame.eModId = MPP_ID_V2D;
        frame.u32Idx = static_cast<U32>(source.sequence);
        frame.stCommFrameInfo.u32Width = static_cast<U32>(source.width);
        frame.stCommFrameInfo.u32Height = static_cast<U32>(source.height);
        frame.stCommFrameInfo.u32Align = 1;
        frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_BGRA;
        frame.stCommFrameInfo.eCompressMode = COMPRESS_MODE_NONE;
        frame.stVFrame.u32PlaneNum = 1;
        frame.stVFrame.u32TotalSize = static_cast<U32>(source.stride_bytes * source.height);
        frame.stVFrame.u32PlaneStride[0] = static_cast<U32>(source.stride_bytes);
        frame.stVFrame.u32PlaneSize[0] = frame.stVFrame.u32TotalSize;
        frame.stVFrame.u32PlaneSizeValid[0] = frame.stVFrame.u32TotalSize;
        frame.stVFrame.u32Fd[0] = static_cast<UL>(source.dma_fd);
        return frame;
    }

    VideoFrameInfo make_render_destination(UL output, std::int64_t timestamp)
    {
        void* address = nullptr;
        S32 dma_fd = -1;
        U64 physical = 0;
        if (VB_GetVirAddr(output, &address) != 0 || address == nullptr ||
            VB_GetDmaBufFd(output, &dma_fd) != 0 || dma_fd < 0 ||
            VB_GetPhyAddr(output, &physical) != 0)
        {
            throw std::runtime_error("failed to resolve ESDF render DMA-BUF");
        }
        auto frame =
            make_render_output_layout(display_width_, display_height_, render_output_bgra_);
        frame.ulPoolId = render_output_pool_;
        frame.ulBufferId = output;
        frame.u32Idx = render_output_index_.load();
        frame.stVFrame.u64PTS = static_cast<U64>(timestamp);
        const auto base = reinterpret_cast<std::uintptr_t>(address);
        U32 offset = 0;
        for (U32 plane = 0; plane < frame.stVFrame.u32PlaneNum; ++plane)
        {
            frame.stVFrame.u32Fd[plane] = static_cast<UL>(dma_fd);
            frame.stVFrame.u64PlanePhyAddr[plane] = physical + offset;
            frame.stVFrame.ulPlaneVirAddr[plane] = static_cast<UL>(base + offset);
            offset += frame.stVFrame.u32PlaneSize[plane];
        }
        return frame;
    }

    void PublishRenderFrame(const esdf_drm::mapping::RenderFrame& source)
    {
        if (!publish_render_output_ || source.dma_fd < 0 ||
            source.width != static_cast<std::uint32_t>(display_width_) ||
            source.height != static_cast<std::uint32_t>(display_height_))
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(render_output_mutex_);
            while (static_cast<int>(render_output_handles_.size()) >= render_output_pool_depth_)
            {
                (void)VB_Unexport(render_output_handles_.front());
                render_output_handles_.pop_front();
            }
        }
        const UL output = VB_GetBuffer(render_output_pool_, 0);
        if (output == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "ESDF render output pool exhausted");
            return;
        }
        try
        {
            RenderOutputBuffer buffer(output);
            const std::int64_t timestamp = now().nanoseconds();
            VideoFrameInfo input_frame = make_render_source(source);
            VideoFrameInfo output_frame = make_render_destination(output, timestamp);
            V2DHandle job = 0;
            S32 result = V2D_BeginJob(&job);
            if (result != 0)
            {
                throw std::runtime_error("V2D_BeginJob failed: " + std::to_string(result));
            }
            V2DArea area{0, 0, static_cast<U16>(display_width_), static_cast<U16>(display_height_)};
            result = V2D_AddBitblitTask(job, &input_frame, &area, &output_frame, &area,
                render_output_bgra_ ? V2D_CSC_MODE_BUTT : V2D_CSC_MODE_RGB_2_BT709NARROW);
            if (result != 0)
            {
                (void)V2D_CancelJob(job);
                throw std::runtime_error("V2D render snapshot failed: " + std::to_string(result));
            }
            result = V2D_EndJob(job);
            if (result != 0)
            {
                throw std::runtime_error("V2D_EndJob failed: " + std::to_string(result));
            }
            if (VB_UpdateBufferFrameInfo(output, &output_frame) != 0)
            {
                throw std::runtime_error("VB_UpdateBufferFrameInfo failed for render snapshot");
            }
            const U64 token = buffer.export_token();
            const std::uint32_t frame_index = render_output_index_.fetch_add(1);
            RenderFrameMessage message;
            message.header.stamp.sec = static_cast<std::int32_t>(timestamp / 1000000000LL);
            message.header.stamp.nanosec = static_cast<std::uint32_t>(timestamp % 1000000000LL);
            message.header.frame_id = world_frame_;
            message.version = 1;
            message.token = token;
            message.pool_id = render_output_pool_;
            message.buffer_id = output;
            message.frame_index = frame_index;
            message.width = static_cast<std::uint32_t>(display_width_);
            message.height = static_cast<std::uint32_t>(display_height_);
            message.align = 1;
            message.pixel_format = output_frame.stCommFrameInfo.ePixelFormat;
            message.pts = static_cast<std::uint64_t>(timestamp);
            message.total_size = output_frame.stVFrame.u32TotalSize;
            message.plane_num = output_frame.stVFrame.u32PlaneNum;
            for (std::uint32_t plane = 0; plane < message.plane_num; ++plane)
            {
                message.plane_stride[plane] = output_frame.stVFrame.u32PlaneStride[plane];
                message.plane_size[plane] = output_frame.stVFrame.u32PlaneSize[plane];
                message.plane_size_valid[plane] = output_frame.stVFrame.u32PlaneSizeValid[plane];
            }
            {
                std::lock_guard<std::mutex> lock(render_output_mutex_);
                buffer.retain_export(render_output_handles_);
                render_output_pub_->publish(std::move(message));
            }
        }
        catch (const std::exception& error)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "ESDF render publish failed: %s", error.what());
        }
    }

    void handle_render_frame(const esdf_drm::mapping::RenderFrame& frame)
    {
        if (recorder_config_.enabled && !recorder_start_failed_ &&
            (!record_wait_for_map_ || mapped_frames_.load() > 0U))
        {
            if (!drm_recorder_)
            {
                auto recorder =
                    std::make_unique<DrmRecorder>(get_logger(), get_clock(), recorder_config_);
                std::string error;
                if (!recorder->start(frame.width, frame.height, &error))
                {
                    recorder_start_failed_ = true;
                    RCLCPP_ERROR(
                        get_logger(), "DRM recorder initialization failed: %s", error.c_str());
                }
                else
                {
                    drm_recorder_ = std::move(recorder);
                }
            }
            if (drm_recorder_)
            {
                drm_recorder_->submit(frame, latest_record_source_timestamp_ns_.load());
                const auto stats = drm_recorder_->stats();
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "DRM record: captured=%" PRIu64 " converted=%" PRIu64 " submitted=%" PRIu64
                    " encoded=%" PRIu64 " ignored=%" PRIu64 " dropped=%" PRIu64
                    " source=%.3fs output=%.3fs v2d_avg=%.3fms file=%s",
                    stats.captured_frames, stats.converted_frames, stats.submitted_frames,
                    stats.encoded_frames, stats.ignored_render_frames, stats.dropped_frames,
                    stats.source_duration_seconds, stats.output_duration_seconds,
                    stats.average_v2d_ms,
                    stats.current_file.empty() ? recorder_config_.output_pattern.c_str()
                                                : stats.current_file.c_str());
            }
        }
        PublishRenderFrame(frame);
    }

    void start_pipelines()
    {
        if (enable_gles_drm_)
        {
            GlesRenderPipelineConfig render_config;
            render_config.target_fps = render_fps_;
            render_config.trajectory_duration_s = trajectory_duration_s_;
            render_config.performance = performance_;
            render_config.feature_overlay_enabled = feature_panel_enabled_;
            render_config.preview_overlay_enabled = !preview_overlay_topic_.empty();
            render_config.feature_overlay_age_max_frames =
                static_cast<std::uint32_t>(feature_panel_age_max_frames_);
            render_config.max_block_updates_per_frame = static_cast<std::size_t>(
                render_max_block_updates_per_frame_ > 0 ? render_max_block_updates_per_frame_
                                                        : 128);
            render_config.renderer.render_device = render_device_;
            // Empty application override selects the SDK's display-device
            // default; it is not a usable filesystem path.
            if (!display_device_.empty())
                render_config.renderer.display_device = display_device_;
            render_config.renderer.width = display_width_;
            render_config.renderer.height = display_height_;
            render_config.renderer.headless = headless_render_;
            render_config.renderer.split_view = mapping_enabled_;
            // There are no map updates in display-only mode. Keep the shared
            // renderer's required geometry storage minimal, not a full map VBO.
            render_config.renderer.maximum_chunks =
                mapping_enabled_ ? static_cast<std::size_t>(render_max_blocks_) : 1U;
            render_config.renderer.maximum_vertices_per_chunk = static_cast<std::size_t>(
                render_max_points_per_block_ > 0 ? render_max_points_per_block_ : 0U);
            if (!mapping_enabled_)
            {
                render_config.renderer.maximum_vertices_per_chunk = 1U;
            }
            render_config.renderer.point_size_pixels = static_cast<float>(render_point_size_);
            render_config.interpolate_surface = render_tsdf_interpolated_points_;
            render_config.renderer.wait_for_display = !render_async_page_flip_;
            render_config.renderer.frustum_culling = render_frustum_culling_;
            render_config.renderer.secondary_camera_indicator = render_secondary_camera_indicator_;
            render_config.renderer.secondary_fov_degrees =
                static_cast<float>(render_secondary_camera_fov_degrees_);
            render_config.renderer.secondary_fov_range_m =
                static_cast<float>(render_secondary_camera_fov_range_m_);
            render_config.renderer.secondary_marker_m =
                static_cast<float>(render_secondary_camera_marker_radius_m_);
            render_config.renderer.secondary_marker_pixels =
                static_cast<float>(render_secondary_camera_marker_size_pixels_);
            render_config.renderer.secondary_overlay_height_m =
                static_cast<float>(slice_output_z_m_);
            render_config.voxel_size_m = static_cast<float>(voxel_size_);
            render_config.minimum_weight = static_cast<float>(pointcloud_min_weight_);
            render_config.surface_band_m = pointcloud_surface_band_m_ < 0.0
                ? static_cast<float>(voxel_size_)
                : static_cast<float>(pointcloud_surface_band_m_);
            render_config.slice_min_z_m = static_cast<float>(slice_min_z_m_);
            render_config.slice_max_z_m = static_cast<float>(slice_max_z_m_);
            auto render_pipeline = std::make_unique<GlesRenderPipeline>(
                std::move(render_config), [this](const std::string& message)
                { RCLCPP_ERROR(get_logger(), "%s", message.c_str()); },
                [this](const std::string& message)
                { RCLCPP_INFO(get_logger(), "%s", message.c_str()); },
                [this](const esdf_drm::mapping::RenderFrame& frame)
                { handle_render_frame(frame); });
            render_pipeline->start();
            std::lock_guard<std::mutex> lock(render_pipeline_lifetime_mutex_);
            render_pipeline_ = std::move(render_pipeline);
        }

        if (!mapping_enabled_)
        {
            return;
        }

        EsdfUpdatePipelineConfig esdf_config;
        esdf_config.tsdf = tsdf_params_;
        esdf_config.esdf = esdf_params_;
        esdf_config.target_hz = esdf_update_hz_;
        esdf_config.max_pending_blocks = static_cast<std::size_t>(esdf_max_pending_blocks_);
        esdf_config.follow_slice_height = follow_slice_height_;
        esdf_config.slice_thickness_m = static_cast<float>(slice_thickness_m_);
        esdf_config.slice_height_offset_m = static_cast<float>(slice_height_offset_m_);
        esdf_config.slice_min_z_m = static_cast<float>(slice_min_z_m_);
        esdf_config.slice_max_z_m = static_cast<float>(slice_max_z_m_);
        esdf_config.slice_output_z_m = static_cast<float>(slice_output_z_m_);
        EsdfUpdatePipeline::UpdateCallback update_callback;
        if (enable_gles_drm_)
        {
            update_callback = [this](std::vector<esdf_drm::MapChange> updates)
            {
                std::lock_guard<std::mutex> lock(render_pipeline_lifetime_mutex_);
                if (render_pipeline_)
                {
                    render_pipeline_->SubmitSecondaryUpdates(std::move(updates));
                }
            };
        }
        esdf_pipeline_ = std::make_unique<EsdfUpdatePipeline>(
            std::move(esdf_config), [this](const std::string& message)
            { RCLCPP_ERROR(get_logger(), "%s", message.c_str()); },
            [this](const std::string& message)
            { RCLCPP_INFO(get_logger(), "%s", message.c_str()); }, std::move(update_callback));
        esdf_pipeline_->start();
    }

    void pose_callback(const PoseMessage::ConstSharedPtr& message)
    {
        if (!message)
        {
            return;
        }
        if (performance_)
        {
            performance_->vio.record();
        }
        // Pose display must not depend on depth delivery or successful TSDF
        // integration. Use the same frozen map frame for camera and geometry.
        auto world_from_camera = transform_from_pose(*message);
        bool display_ready = true;
        if (mapping_enabled_ && gravity_align_map_)
        {
            const auto& r = world_from_camera.rotation;
            auto g = initial_gravity_camera_;
            if (display_imu_sub_ && dot(g, g) == 0.0F)
            {
                std::lock_guard<std::mutex> lock(gravity_mutex_);
                if (display_gravity_candidate_ &&
                    display_gravity_candidate_->covers(stamp_ns(message->header.stamp)))
                {
                    const auto& down = display_gravity_candidate_->gravity;
                    const auto& rotation = display_imu_rotation_;
                    g = {static_cast<float>(
                                rotation[0] * down[0] + rotation[1] * down[1] + rotation[2] * down[2]),
                        static_cast<float>(
                            rotation[3] * down[0] + rotation[4] * down[1] + rotation[5] * down[2]),
                        static_cast<float>(
                            rotation[6] * down[0] + rotation[7] * down[1] + rotation[8] * down[2])};
                }
            }
            if (dot(g, g) > 0.0F)
            {
                freeze_gravity({r[0] * g.x + r[1] * g.y + r[2] * g.z,
                    r[3] * g.x + r[4] * g.y + r[5] * g.z, r[6] * g.x + r[7] * g.y + r[8] * g.z});
            }
            std::lock_guard<std::mutex> lock(gravity_mutex_);
            display_ready = gravity_alignment_ready_;
            if (display_ready)
            {
                world_from_camera = compose(gravity_map_from_world_, world_from_camera);
            }
        }
        if (display_ready)
        {
            InteractiveViewState state;
            {
                std::lock_guard<std::mutex> lock(interactive_view_mutex_);
                latest_render_pose_ = world_from_camera;
                latest_render_pose_stamp_ = stamp_ns(message->header.stamp);
                latest_render_pose_valid_ = true;
                state = interactive_view_;
            }
            submit_current_view(state, world_from_camera, stamp_ns(message->header.stamp));
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "display pose: stamp=%" PRId64 " position=[%.3f %.3f %.3f]",
                stamp_ns(message->header.stamp), world_from_camera.translation[0],
                world_from_camera.translation[1], world_from_camera.translation[2]);
        }
        if (!mapping_enabled_)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        pair_queue().push_first(stamp_ns(message->header.stamp), message);
        pair_cv_.notify_one();
    }

    void gravity_callback(const GravityMessage::ConstSharedPtr& message)
    {
        if (!message || !gravity_align_map_)
        {
            return;
        }
        // An explicit initial-camera seed owns the frame for this session;
        // do not race it against an unrelated gravity publisher.
        if (!display_imu_sub_ && dot(initial_gravity_camera_, initial_gravity_camera_) == 0.0F)
        {
            freeze_gravity({static_cast<float>(message->vector.x),
                static_cast<float>(message->vector.y), static_cast<float>(message->vector.z)});
        }
    }

    void freeze_gravity(const Vec3& gravity)
    {
        const float magnitude = std::sqrt(dot(gravity, gravity));
        if (!std::isfinite(magnitude) || magnitude < 1.0F || magnitude > 20.0F)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "ignoring invalid gravity magnitude %.3f", magnitude);
            return;
        }
        std::lock_guard<std::mutex> lock(gravity_mutex_);
        if (gravity_alignment_ready_)
        {
            return;
        }
        gravity_world_ = scale(gravity, 1.0F / magnitude);
        gravity_map_from_world_ = gravity_aligned_from_world(gravity_world_);
        gravity_alignment_ready_ = true;
        RCLCPP_INFO(get_logger(), "gravity map frozen: g_world=[%.5f %.5f %.5f], map +Z is up",
            gravity_world_.x, gravity_world_.y, gravity_world_.z);
    }

    void update_record_source_timestamp(std::int64_t timestamp_ns)
    {
        if (timestamp_ns < 0 ||
            (!recorder_config_.source_timestamp_origin_zero && timestamp_ns == 0))
        {
            return;
        }
        std::int64_t current = latest_record_source_timestamp_ns_.load();
        while (timestamp_ns > current &&
            !latest_record_source_timestamp_ns_.compare_exchange_weak(current, timestamp_ns))
        {
        }
    }

    void record_timestamp_callback(const RenderFrameMessage::ConstSharedPtr& message)
    {
        if (!message)
        {
            return;
        }
        if (record_timestamp_use_frame_pts_)
        {
            constexpr std::uint64_t kMaximumPtsUs =
                static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
            if (message->pts <= kMaximumPtsUs)
            {
                update_record_source_timestamp(static_cast<std::int64_t>(message->pts * 1000U));
            }
            return;
        }
        update_record_source_timestamp(stamp_ns(message->header.stamp));
    }

    void preview_overlay_callback(const RenderFrameMessage::ConstSharedPtr& message)
    {
        if (!message || message->token == 0 || message->pixel_format != MPP_PIXEL_FORMAT_NV12 ||
            message->plane_num < 2 || message->width == 0 || message->height == 0 ||
            (message->width & 1U) != 0U || (message->height & 1U) != 0U ||
            message->plane_stride[0] < message->width ||
            message->plane_stride[1] != message->plane_stride[0])
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "invalid NV12 preview overlay descriptor");
            return;
        }
        const std::size_t y_bytes = static_cast<std::size_t>(message->plane_stride[0]) *
            static_cast<std::size_t>(message->height);
        const std::size_t uv_offset = message->plane_size[0] > 0U
            ? static_cast<std::size_t>(message->plane_size[0])
            : y_bytes;
        const std::size_t uv_bytes = static_cast<std::size_t>(message->plane_stride[1]) *
            static_cast<std::size_t>(message->height / 2U);
        if (uv_offset != y_bytes || uv_offset > message->total_size ||
            uv_bytes > static_cast<std::size_t>(message->total_size) - uv_offset)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "invalid NV12 preview overlay plane layout");
            return;
        }
        UL handle = 0;
        S32 dma_fd = -1;
        if (VB_Import(message->token, &handle) != 0 || handle == 0 ||
            VB_GetDmaBufFd(handle, &dma_fd) != 0 || dma_fd < 0)
        {
            if (handle != 0)
            {
                (void)VB_ReleaseBuffer(handle);
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "failed to import NV12 preview DMA-BUF");
            return;
        }
        const int owned_fd = dup(dma_fd);
        (void)VB_ReleaseBuffer(handle);
        if (owned_fd < 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "failed to duplicate NV12 preview DMA-BUF fd");
            return;
        }
        std::lock_guard<std::mutex> lock(render_pipeline_lifetime_mutex_);
        if (render_pipeline_)
        {
            render_pipeline_->SubmitNv12Overlay(
                {owned_fd, message->width, message->height, message->plane_stride[0]});
        }
        else
        {
            close(owned_fd);
        }
    }

    // Keep only descriptors here, not imported camera buffers. A slow VIO frame
    // may miss the camera export window; in that case skip the diagnostic tile
    // instead of retaining camera buffers and delaying capture.
    void feature_panel_image_callback(const RenderFrameMessage::ConstSharedPtr& message)
    {
        if (!message)
            return;
        FeatureMessage::ConstSharedPtr matched;
        {
            std::lock_guard<std::mutex> lock(feature_pair_mutex_);
            const auto entry =
                std::find_if(pending_feature_messages_.begin(), pending_feature_messages_.end(),
                    [&message](const FeatureMessage::ConstSharedPtr& features)
                    {
                        return features->source_frame_index == message->frame_index &&
                            stamp_ns(features->header.stamp) == stamp_ns(message->header.stamp);
                    });
            if (entry != pending_feature_messages_.end())
            {
                matched = *entry;
                pending_feature_messages_.erase(entry);
            }
            else
            {
                pending_feature_images_.push_back(message);
                while (pending_feature_images_.size() > kFeaturePairDepth)
                    pending_feature_images_.pop_front();
            }
        }
        if (matched)
            submit_feature_panel(*message, *matched);
    }

    void feature_panel_callback(const FeatureMessage::ConstSharedPtr& message)
    {
        if (!message)
            return;
        RenderFrameMessage::ConstSharedPtr matched;
        {
            std::lock_guard<std::mutex> lock(feature_pair_mutex_);
            const auto entry =
                std::find_if(pending_feature_images_.begin(), pending_feature_images_.end(),
                    [&message](const RenderFrameMessage::ConstSharedPtr& image)
                    {
                        return image->frame_index == message->source_frame_index &&
                            stamp_ns(image->header.stamp) == stamp_ns(message->header.stamp);
                    });
            if (entry != pending_feature_images_.end())
            {
                matched = *entry;
                pending_feature_images_.erase(entry);
            }
            else
            {
                pending_feature_messages_.push_back(message);
                while (pending_feature_messages_.size() > kFeaturePairDepth)
                    pending_feature_messages_.pop_front();
            }
        }
        if (matched)
            submit_feature_panel(*matched, *message);
    }

    void submit_feature_panel(const RenderFrameMessage& image, const FeatureMessage& features)
    {
        if (image.token == 0 || image.pixel_format != MPP_PIXEL_FORMAT_NV12 ||
            image.plane_num < 2 || image.width == 0 || image.height == 0 ||
            (image.width & 1U) != 0U || (image.height & 1U) != 0U ||
            image.plane_stride[0] < image.width || image.plane_stride[1] != image.plane_stride[0] ||
            features.image_width != image.width || features.image_height != image.height)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "invalid feature panel image or coordinate dimensions");
            return;
        }
        const std::size_t y_bytes = static_cast<std::size_t>(image.plane_stride[0]) *
            static_cast<std::size_t>(image.height);
        const std::size_t uv_offset =
            image.plane_size[0] > 0U ? static_cast<std::size_t>(image.plane_size[0]) : y_bytes;
        const std::size_t uv_bytes = static_cast<std::size_t>(image.plane_stride[1]) *
            static_cast<std::size_t>(image.height / 2U);
        if (uv_offset != y_bytes || uv_offset > image.total_size ||
            uv_bytes > static_cast<std::size_t>(image.total_size) - uv_offset)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "invalid feature panel NV12 plane layout");
            return;
        }
        UL handle = 0;
        S32 dma_fd = -1;
        if (VB_Import(image.token, &handle) != 0 || handle == 0)
        {
            if (handle != 0)
                (void)VB_ReleaseBuffer(handle);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "feature panel camera buffer expired before VIO features");
            return;
        }
        // A DMA fd/EGLImage retains the allocation, but not its MPP pool slot.
        // Keep the import until the render thread completes its private GPU copy.
        auto source_lease =
            std::shared_ptr<void>(nullptr, [handle](void*) { (void)VB_ReleaseBuffer(handle); });
        if (VB_GetDmaBufFd(handle, &dma_fd) != 0 || dma_fd < 0)
            return;
        const int owned_fd = dup(dma_fd);
        if (owned_fd < 0)
            return;

        FeatureOverlayFrame frame;
        frame.source_lease = std::move(source_lease);
        frame.dma_fd = owned_fd;
        frame.width = static_cast<int>(image.width);
        frame.height = static_cast<int>(image.height);
        frame.stride_bytes = static_cast<int>(image.plane_stride[0]);
        frame.source_frame_index = image.frame_index;
        frame.timestamp_ns = stamp_ns(image.header.stamp);
        frame.tracked_feature_count = features.tracked_feature_count;
        frame.pose_inputs_available = features.pose_inputs_available;
        frame.pose_input_count = features.pose_input_count;
        frame.quality.pose_valid = features.pose_valid;
        frame.quality.available = features.diagnostics_available;
        frame.quality.prediction_available = features.prediction_available;
        frame.quality.prediction_active = features.prediction_active;
        frame.quality.prediction_streak = features.prediction_streak;
        frame.quality.prediction_span_ms = features.prediction_span_ms;
        frame.quality.reprojection_available = features.reprojection_available;
        frame.quality.reprojection_rmse_px = features.reprojection_rmse_px;
        frame.points.reserve(features.points.size());
        for (const auto& point : features.points)
        {
            if (std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0.0F &&
                point.y >= 0.0F && point.x < image.width && point.y < image.height)
                frame.points.push_back({point.x, point.y, point.pose_input ? 1.0F : 0.0F,
                    static_cast<float>(point.track_observation_count)});
        }
        std::lock_guard<std::mutex> lock(render_pipeline_lifetime_mutex_);
        if (render_pipeline_)
            render_pipeline_->SubmitFeatureOverlay(std::move(frame));
        else
            close(owned_fd);
    }

    void depth_callback(const DepthMessage::ConstSharedPtr& message)
    {
        if (!message || message->encoding != kDepthEncodingFloat32Meters || message->token == 0 ||
            message->width == 0 || message->height == 0 ||
            message->stride_bytes < message->width * sizeof(float) ||
            message->rgb_encoding != kRgbEncodingRgb8 || message->rgb_token == 0 ||
            message->rgb_width != message->width || message->rgb_height != message->height ||
            message->rgb_stride_bytes < message->rgb_width * 3U)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "dropping depth frame without aligned RGB DMA descriptor");
            return;
        }
        try
        {
            auto retained = std::make_shared<RetainedDepth>(message);
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            pair_queue().push_second(stamp_ns(message->header.stamp), std::move(retained));
            pair_cv_.notify_one();
        }
        catch (const std::exception& error)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "failed to retain depth/RGB DMA-BUF: %s", error.what());
        }
    }

    PairQueue& pair_queue()
    {
        // Every caller holds buffer_mutex_. Pose metadata is cheap, but an
        // unmatched depth value owns imported DMA-BUF handles, so bound it by
        // the much smaller mapping work queue.
        if (!pair_queue_)
        {
            pair_queue_ = std::make_unique<PairQueue>(static_cast<std::size_t>(pair_buffer_depth_),
                static_cast<std::size_t>(queue_depth_), static_cast<std::size_t>(queue_depth_));
        }
        return *pair_queue_;
    }

    void mapping_loop()
    {
        while (true)
        {
            Pair pair;
            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                pair_cv_.wait(lock, [&] { return stopping_.load() || pair_queue().ready(); });
                if (stopping_.load())
                {
                    pair_queue().clear();
                    return;
                }
                auto matched = pair_queue().pop();
                if (!matched)
                {
                    continue;
                }
                pair.pose = std::move(matched->first);
                pair.depth = matched->second->descriptor;
                pair.retained = std::move(matched->second);
            }
            try
            {
                integrate_pair(pair);
            }
            catch (const std::exception& error)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TSDF/ESDF integration failed: %s", error.what());
            }
        }
    }

    void copy_inputs(const DepthMessage& depth, const ImportedBuffer& depth_buffer,
        const ImportedBuffer& rgb_buffer)
    {
        const std::size_t pixel_count = static_cast<std::size_t>(depth.width) * depth.height;
        depth_scratch_.resize(pixel_count);
        rgb_scratch_.resize(pixel_count);
        const std::size_t depth_row_bytes = static_cast<std::size_t>(depth.width) * sizeof(float);
        const std::size_t rgb_row_bytes = static_cast<std::size_t>(depth.width) * 3U;
        for (std::uint32_t row = 0; row < depth.height; ++row)
        {
            std::memcpy(depth_scratch_.data() + static_cast<std::size_t>(row) * depth.width,
                depth_buffer.data() + static_cast<std::size_t>(row) * depth.stride_bytes,
                depth_row_bytes);
            std::memcpy(reinterpret_cast<std::uint8_t*>(rgb_scratch_.data()) +
                    static_cast<std::size_t>(row) * rgb_row_bytes,
                rgb_buffer.data() + static_cast<std::size_t>(row) * depth.rgb_stride_bytes,
                rgb_row_bytes);
        }
    }

    void integrate_pair(Pair& pair)
    {
        const auto total_begin = std::chrono::steady_clock::now();
        const auto& depth = *pair.depth;
        if (!pair.retained)
        {
            throw std::runtime_error("missing retained depth/RGB DMA-BUF");
        }
        copy_inputs(depth, pair.retained->depth, pair.retained->rgb);
        // CPU scratch owns the pixels now; release both imports before TSDF
        // integration and asynchronous ESDF/render work.
        pair.retained.reset();
        const auto copy_end = std::chrono::steady_clock::now();

        esdf_drm::mapping::Camera camera;
        camera.width = static_cast<int>(depth.width);
        camera.height = static_cast<int>(depth.height);
        camera.fx = static_cast<float>(depth.fx);
        camera.fy = static_cast<float>(depth.fy);
        camera.cx = static_cast<float>(depth.cx);
        camera.cy = static_cast<float>(depth.cy);
        esdf_drm::mapping::Pose world_from_left = transform_from_pose(*pair.pose);
        if (gravity_align_map_)
        {
            esdf_drm::mapping::Pose map_from_world;
            {
                std::lock_guard<std::mutex> lock(gravity_mutex_);
                if (!gravity_alignment_ready_)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "waiting for %s before map integration",
                        display_imu_sub_ ? "quiet display IMU interval covering an image pose"
                                            : "reliable VIO gravity or explicit display seed");
                    return;
                }
                map_from_world = gravity_map_from_world_;
            }
            world_from_left = compose(map_from_world, world_from_left);
        }
        const esdf_drm::mapping::Pose world_from_depth =
            compose(world_from_left, transform_from_depth_message(depth));
        mapping::RgbdFrame input;
        input.timestamp_ns = stamp_ns(depth.header.stamp);
        input.camera = camera;
        input.map_from_camera = world_from_depth;
        input.depth_m = depth_scratch_.data();
        input.depth_bytes = depth_scratch_.size() * sizeof(float);
        input.depth_stride_bytes = static_cast<std::size_t>(depth.width) * sizeof(float);
        input.rgb8 = reinterpret_cast<const std::uint8_t*>(rgb_scratch_.data());
        input.rgb_bytes = rgb_scratch_.size() * sizeof(RgbPixel);
        input.rgb_stride_bytes = static_cast<std::size_t>(depth.width) * sizeof(RgbPixel);
        const auto integration = tsdf_->integrate(input);
        std::vector<mapping::ChunkKey> changed_blocks;
        for (const auto& change : tsdf_->capture_changes().changes())
            changed_blocks.push_back(change.key);
        const auto integrated_chunks = changed_blocks.size();
        const auto clear_begin = std::chrono::steady_clock::now();
        // Depth may finish later than pose. Use the latest displayed camera in
        // the SAME frozen mapping frame, not the delayed integration camera.
        esdf_drm::mapping::Vec3 clear_center{world_from_left.translation[0],
            world_from_left.translation[1], world_from_left.translation[2]};
        {
            std::lock_guard<std::mutex> lock(interactive_view_mutex_);
            if (latest_render_pose_valid_)
                clear_center = {latest_render_pose_.translation[0],
                    latest_render_pose_.translation[1], latest_render_pose_.translation[2]};
        }
        const auto removed = local_map_->clear(clear_center, clear_begin);
        changed_blocks.insert(changed_blocks.end(), removed.begin(), removed.end());
        if (!removed.empty())
            RCLCPP_INFO(get_logger(),
                "map clear: radius=%.2fm rate=%.2fHz removed=%zu retained=%zu time=%.3fms "
                "center=[%.3f %.3f %.3f]",
                map_clearing_radius_m_, clear_map_rate_hz_, removed.size(), tsdf_->chunk_count(),
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - clear_begin)
                    .count(),
                clear_center.x, clear_center.y, clear_center.z);
        const auto tsdf_end = std::chrono::steady_clock::now();
        if (performance_)
            performance_->tsdf.record();
        if (esdf_pipeline_ && follow_slice_height_)
        {
            esdf_pipeline_->setSliceCenter(world_from_left.translation[2]);
        }
        if (esdf_pipeline_ || render_pipeline_)
        {
            auto block_updates = split_changes(tsdf_->capture_keys(changed_blocks));
            if (esdf_pipeline_)
            {
                esdf_pipeline_->submit(block_updates);
            }
            if (render_pipeline_)
            {
                render_pipeline_->SubmitTsdfUpdates(std::move(block_updates));
            }
        }
        const auto esdf_end = std::chrono::steady_clock::now();
        double prepare_ms = 0.0;
        double submit_ms = 0.0;
        if (render_pipeline_)
        {
            if (recorder_config_.use_source_timestamps && !record_timestamp_sub_)
            {
                if (record_timestamp_use_frame_pts_)
                {
                    constexpr std::uint64_t kMaximumPtsUs =
                        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
                    if (depth.pts <= kMaximumPtsUs)
                    {
                        update_record_source_timestamp(
                            static_cast<std::int64_t>(depth.pts * 1000U));
                    }
                }
                else
                {
                    update_record_source_timestamp(stamp_ns(depth.header.stamp));
                }
            }
            const auto prepare_end = std::chrono::steady_clock::now();
            // The pose callback owns the current camera view. A delayed depth
            // result must not move it backwards to that older image pose.
            const auto submit_end = std::chrono::steady_clock::now();
            prepare_ms = std::chrono::duration<double, std::milli>(prepare_end - esdf_end).count();
            submit_ms = std::chrono::duration<double, std::milli>(submit_end - prepare_end).count();
        }
        if (publish_debug_pointcloud_)
        {
            publish_surface_pointcloud(depth.header);
        }
        ++mapped_frames_;
        const auto total_end = std::chrono::steady_clock::now();
        const double copy_ms =
            std::chrono::duration<double, std::milli>(copy_end - total_begin).count();
        const double tsdf_ms =
            std::chrono::duration<double, std::milli>(tsdf_end - copy_end).count();
        const double esdf_ms =
            std::chrono::duration<double, std::milli>(esdf_end - tsdf_end).count();
        const double total_ms =
            std::chrono::duration<double, std::milli>(total_end - total_begin).count();
        const EsdfUpdatePipelineStats esdf_pipeline_stats =
            esdf_pipeline_ ? esdf_pipeline_->stats() : EsdfUpdatePipelineStats{};
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "mapped frame=%u tsdf_blocks=%zu esdf_blocks=%zu "
            "dirty=%zu candidates=%zu esdf_propagated=%zu "
            "timing=copy:%.2f tsdf:%.2f esdf:%.2f prepare:%.2f "
            "submit:%.2f total:%.2fms mapped=%" PRIu64,
            depth.frame_index, tsdf_->chunk_count(), esdf_pipeline_stats.esdf_blocks,
            integrated_chunks, static_cast<std::size_t>(integration.candidate_samples),
            esdf_pipeline_stats.last_propagated_blocks, copy_ms, tsdf_ms, esdf_ms, prepare_ms,
            submit_ms, total_ms, mapped_frames_.load());
    }

    void publish_surface_pointcloud(const std_msgs::msg::Header& header)
    {
        const auto now = std::chrono::steady_clock::now();
        if (pointcloud_published_ && pointcloud_publish_period_ms_ > 0 &&
            now - last_pointcloud_publish_ <
                std::chrono::milliseconds(pointcloud_publish_period_ms_))
            return;
        const float band = pointcloud_surface_band_m_ < 0.0
            ? static_cast<float>(voxel_size_)
            : static_cast<float>(pointcloud_surface_band_m_);
        std::size_t available = 0;
        mapping::SurfaceOptions options;
        options.minimum_confidence = static_cast<float>(pointcloud_min_weight_);
        options.surface_band_m = band;
        options.interpolate_surface = false;
        const auto points = tsdf_->surface_points(
            options, static_cast<std::size_t>(pointcloud_max_points_), &available);
        PointCloudMessage message;
        message.header = header;
        message.header.frame_id = world_frame_;
        message.height = 1;
        message.is_bigendian = false;
        message.is_dense = true;
        message.point_step = 16;
        message.fields.resize(4);
        const auto set_field = [&message](std::size_t index, const char* name, std::uint32_t offset)
        {
            auto& field = message.fields[index];
            field.name = name;
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
        };
        set_field(0, "x", 0);
        set_field(1, "y", 4);
        set_field(2, "z", 8);
        set_field(3, "rgb", 12);
        message.data.resize(points.size() * message.point_step);
        for (std::size_t index = 0; index < points.size(); ++index)
        {
            const std::array<float, 3> position{points[index].x, points[index].y, points[index].z};
            const std::uint32_t rgb = points[index].rgba & 0x00ffffffU;
            auto* target = message.data.data() + index * message.point_step;
            std::memcpy(target, position.data(), 3U * sizeof(float));
            std::memcpy(target + 12U, &rgb, sizeof(rgb));
        }
        message.width = static_cast<std::uint32_t>(points.size());
        message.row_step = message.width * message.point_step;
        pointcloud_pub_->publish(std::move(message));
        last_pointcloud_publish_ = now;
        pointcloud_published_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "published colored surface points=%zu source=%" PRIu64, points.size(),
            static_cast<std::uint64_t>(available));
    }

    std::string pose_topic_;
    std::string depth_topic_;
    std::string gravity_topic_;
    std::string preview_overlay_topic_;
    bool feature_panel_enabled_{false};
    int feature_panel_age_max_frames_{14};
    std::string feature_panel_image_topic_;
    std::string feature_panel_topic_;
    std::string world_frame_;
    std::string pointcloud_topic_;
    int pair_buffer_depth_{64};
    int queue_depth_{2};
    double voxel_size_{0.05};
    double truncation_distance_{0.15};
    double max_integration_distance_{3.0};
    double invalid_depth_decay_factor_{0.8};
    double max_esdf_distance_{2.0};
    double esdf_update_hz_{5.0};
    int esdf_max_pending_blocks_{16384};
    int pointcloud_publish_period_ms_{500};
    int pointcloud_max_points_{30000};
    double pointcloud_min_weight_{1.0};
    double pointcloud_surface_band_m_{-1.0};
    bool publish_debug_pointcloud_{false};
    bool mapping_enabled_{true};
    bool enable_gles_drm_{true};
    bool headless_render_{false};
    bool publish_render_output_{false};
    std::string render_output_topic_;
    int render_output_pool_depth_{6};
    bool render_output_bgra_{false};
    DrmRecorderConfig recorder_config_;
    std::string record_timestamp_topic_;
    bool record_timestamp_use_frame_pts_{false};
    bool recorder_start_failed_{false};
    bool record_wait_for_map_{false};
    double render_fps_{60.0};
    std::string render_device_;
    std::string display_device_;
    int display_width_{1920};
    int display_height_{1080};
    int render_max_blocks_{8192};
    int render_max_block_updates_per_frame_{0};
    int render_max_points_per_block_{0};
    double render_point_size_{8.0};
    double render_slice_view_half_height_m_{3.0};
    bool render_measure_gpu_time_{false};
    bool render_async_page_flip_{true};
    bool render_frustum_culling_{true};
    bool render_tsdf_interpolated_points_{false};
    bool render_secondary_camera_indicator_{true};
    double render_secondary_camera_fov_degrees_{70.0};
    double render_secondary_camera_fov_range_m_{1.5};
    double render_secondary_camera_marker_radius_m_{0.12};
    double render_secondary_camera_marker_size_pixels_{18.0};
    bool mouse_navigation_enabled_{true};
    std::string mouse_input_device_;
    bool gravity_align_map_{true};
    bool follow_slice_height_{true};
    double slice_min_z_m_{-0.25};
    double slice_max_z_m_{0.25};
    double slice_output_z_m_{0.0};
    double slice_thickness_m_{0.05};
    double slice_height_offset_m_{0.0};
    esdf_drm::mapping::MapOptions tsdf_params_{};
    esdf_drm::mapping::DistanceOptions esdf_params_{};
    bool sys_initialized_{false};
    bool vb_initialized_{false};
    UL render_output_pool_{0};
    std::deque<UL> render_output_handles_;
    std::mutex render_output_mutex_;
    std::atomic<std::uint32_t> render_output_index_{0};
    bool pointcloud_published_{false};
    std::chrono::steady_clock::time_point last_pointcloud_publish_{};

    rclcpp::Subscription<PoseMessage>::SharedPtr pose_sub_;
    rclcpp::Subscription<DepthMessage>::SharedPtr depth_sub_;
    rclcpp::Subscription<GravityMessage>::SharedPtr gravity_sub_;
    rclcpp::Subscription<RenderFrameMessage>::SharedPtr record_timestamp_sub_;
    rclcpp::Subscription<RenderFrameMessage>::SharedPtr preview_overlay_sub_;
    rclcpp::Subscription<RenderFrameMessage>::SharedPtr feature_panel_image_sub_;
    rclcpp::Subscription<FeatureMessage>::SharedPtr feature_panel_sub_;
    static constexpr std::size_t kFeaturePairDepth = 6;
    std::mutex feature_pair_mutex_;
    std::deque<RenderFrameMessage::ConstSharedPtr> pending_feature_images_;
    std::deque<FeatureMessage::ConstSharedPtr> pending_feature_messages_;
    std::shared_ptr<PerformanceStats> performance_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracking_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr las2_timing_sub_;
    rclcpp::Publisher<PointCloudMessage>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<RenderFrameMessage>::SharedPtr render_output_pub_;
    std::unique_ptr<PairQueue> pair_queue_;
    std::vector<float> depth_scratch_;
    std::vector<esdf_drm::RgbPixel> rgb_scratch_;
    std::mutex buffer_mutex_;
    std::mutex gravity_mutex_;
    Vec3 gravity_world_{0.0F, 1.0F, 0.0F};
    esdf_drm::mapping::Pose gravity_map_from_world_{};
    bool gravity_alignment_ready_{false};
    Vec3 initial_gravity_camera_{};
    std::condition_variable pair_cv_;
    std::thread mapping_thread_;
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint64_t> mapped_frames_{0};
    std::atomic<std::int64_t> latest_record_source_timestamp_ns_{0};
    std::mutex interactive_view_mutex_;
    InteractiveViewState interactive_view_{};
    esdf_drm::mapping::Pose latest_render_pose_{};
    std::int64_t latest_render_pose_stamp_{};
    bool latest_render_pose_valid_{false};

    std::unique_ptr<MouseInput> mouse_input_;
    std::unique_ptr<GlesRenderPipeline> render_pipeline_;
    std::mutex render_pipeline_lifetime_mutex_;
    std::unique_ptr<DrmRecorder> drm_recorder_;
    double map_clearing_radius_m_{0.0};
    double trajectory_duration_s_{0.0};
    double clear_map_rate_hz_{1.0};
    std::unique_ptr<LocalMapWindow> local_map_;
    esdf_drm::mapping::Map* tsdf_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr display_imu_sub_;
    DisplayGravitySeed display_gravity_seed_;
    std::optional<DisplayGravitySeed::Seed> display_gravity_candidate_;
    std::array<double, 9> display_imu_rotation_{};
    std::unique_ptr<EsdfUpdatePipeline> esdf_pipeline_;
};

} // namespace esdf_drm

RCLCPP_COMPONENTS_REGISTER_NODE(esdf_drm::RobotEsdfDrmNode)
