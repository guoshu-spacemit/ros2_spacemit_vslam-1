#include "camera_mpp_publisher.h"

#include <algorithm>
#include <cinttypes>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include "Imu.h"
#include "ImuPublisher.h"
#include "imu_transport.h"
#include "spacemit_camera/embedded_metadata.h"
#include "spacemit_camera/gpio_exposure.h"
#include "sys/vb_api.h"

namespace camera_io
{
namespace
{

VideoFrameInfo make_nv12_frame(UL pool, UL buffer, std::uint32_t width, std::uint32_t height,
    std::uint32_t frame_index, std::uint64_t pts)
{
    VideoFrameInfo frame{};
    void* address = nullptr;
    S32 fd = -1;
    U64 physical = 0;
    if (VB_GetVirAddr(buffer, &address) != 0 || address == nullptr ||
        VB_GetDmaBufFd(buffer, &fd) != 0 || fd < 0 || VB_GetPhyAddr(buffer, &physical) != 0)
    {
        throw std::runtime_error("failed to resolve an eye VB buffer");
    }

    const U32 y_size = width * height;
    frame.eFrameType = FRAME_TYPE_COMMON;
    frame.eModId = MPP_ID_V2D;
    frame.ulPoolId = pool;
    frame.ulBufferId = buffer;
    frame.u32Idx = frame_index;
    frame.stCommFrameInfo.u32Width = width;
    frame.stCommFrameInfo.u32Height = height;
    frame.stCommFrameInfo.u32Align = 1;
    frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
    frame.stVFrame.u64PTS = pts;
    frame.stVFrame.u32PlaneNum = 2;
    frame.stVFrame.u32TotalSize = y_size + y_size / 2U;

    frame.stVFrame.u32PlaneStride[0] = width;
    frame.stVFrame.u32PlaneSize[0] = y_size;
    frame.stVFrame.u32PlaneSizeValid[0] = y_size;
    frame.stVFrame.u32Fd[0] = static_cast<UL>(fd);
    frame.stVFrame.u64PlanePhyAddr[0] = physical;
    frame.stVFrame.ulPlaneVirAddr[0] = reinterpret_cast<UL>(address);

    frame.stVFrame.u32PlaneStride[1] = width;
    frame.stVFrame.u32PlaneSize[1] = y_size / 2U;
    frame.stVFrame.u32PlaneSizeValid[1] = y_size / 2U;
    frame.stVFrame.u32Fd[1] = static_cast<UL>(fd);
    frame.stVFrame.u64PlanePhyAddr[1] = physical + y_size;
    frame.stVFrame.ulPlaneVirAddr[1] = reinterpret_cast<UL>(address) + y_size;
    return frame;
}

VideoFrameInfo normalize_decoded_nv12_frame(const VideoFrameInfo& decoded,
    std::uint32_t fallback_width, std::uint32_t fallback_height, std::uint32_t frame_index)
{
    VideoFrameInfo frame = decoded;
    const auto decoded_width = decoded.stCommFrameInfo.u32Width != 0
        ? decoded.stCommFrameInfo.u32Width
        : decoded.stVdecFrameInfo.stCommFrameInfo.u32Width;
    const auto decoded_height = decoded.stCommFrameInfo.u32Height != 0
        ? decoded.stCommFrameInfo.u32Height
        : decoded.stVdecFrameInfo.stCommFrameInfo.u32Height;
    const U32 width = decoded_width != 0 ? decoded_width : fallback_width;
    const U32 height = decoded_height != 0 ? decoded_height : fallback_height;
    if (width == 0 || height == 0)
    {
        throw std::runtime_error("decoded NV12 frame has no valid dimensions");
    }

    frame.u32Idx = frame_index;
    frame.stCommFrameInfo.u32Width = width;
    frame.stCommFrameInfo.u32Height = height;
    frame.stCommFrameInfo.u32Align =
        decoded.stCommFrameInfo.u32Align != 0 ? decoded.stCommFrameInfo.u32Align : 1;
    frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
    frame.stVFrame.u32PlaneNum = std::max<U32>(decoded.stVFrame.u32PlaneNum, 2U);

    U32 y_stride = decoded.stVFrame.u32PlaneStride[0];
    if (y_stride < width && decoded.stVFrame.u32PlaneSize[0] >= width * height &&
        decoded.stVFrame.u32PlaneSize[0] % height == 0)
    {
        y_stride = decoded.stVFrame.u32PlaneSize[0] / height;
    }
    if (y_stride < width)
    {
        y_stride = width;
    }
    const U32 chroma_height = (height + 1U) / 2U;
    U32 uv_stride = decoded.stVFrame.u32PlaneStride[1];
    if (uv_stride < width && chroma_height != 0 &&
        decoded.stVFrame.u32PlaneSize[1] >= width * chroma_height &&
        decoded.stVFrame.u32PlaneSize[1] % chroma_height == 0)
    {
        uv_stride = decoded.stVFrame.u32PlaneSize[1] / chroma_height;
    }
    if (uv_stride < width)
    {
        uv_stride = y_stride;
    }
    frame.stVFrame.u32PlaneStride[0] = y_stride;
    frame.stVFrame.u32PlaneStride[1] = uv_stride;

    const U32 minimum_y_size = y_stride * height;
    const U32 minimum_uv_size = uv_stride * chroma_height;
    frame.stVFrame.u32PlaneSize[0] = std::max(decoded.stVFrame.u32PlaneSize[0], minimum_y_size);
    frame.stVFrame.u32PlaneSize[1] = std::max(decoded.stVFrame.u32PlaneSize[1], minimum_uv_size);
    frame.stVFrame.u32PlaneSizeValid[0] =
        std::max(decoded.stVFrame.u32PlaneSizeValid[0], minimum_y_size);
    frame.stVFrame.u32PlaneSizeValid[1] =
        std::max(decoded.stVFrame.u32PlaneSizeValid[1], minimum_uv_size);
    frame.stVFrame.u32TotalSize = std::max(decoded.stVFrame.u32TotalSize,
        frame.stVFrame.u32PlaneSize[0] + frame.stVFrame.u32PlaneSize[1]);
    return frame;
}

} // namespace

void CameraMppPublisher::initialize_imu()
{
    if (config_.imu_enabled)
    {
        std::size_t node_domain = 0;
        if (rcl_context_get_domain_id(
                get_node_base_interface()->get_context()->get_rcl_context().get(), &node_domain) !=
            RCL_RET_OK)
            throw std::runtime_error("failed to resolve the ROS IMU publication domain");
        // Same-domain publication must use the node's RMW transport policy,
        // including localhost-only discovery. Retain explicit cross-domain DDS.
        if (node_domain == static_cast<std::size_t>(config_.domain_id))
        {
            const auto qos = rclcpp::SensorDataQoS().keep_last(
                static_cast<std::size_t>(std::max(64, config_.imu_queue_depth)));
            imu_raw_ros_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(
                *this, config_.imu_raw_ros_topic, qos);
            if (config_.ahrs_enabled)
            {
                ahrs_ = std::make_unique<camera_sdk::MahonyAhrs>(config_.ahrs_config);
                imu_fused_ros_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(
                    *this, config_.imu_fused_ros_topic, qos);
            }
            RCLCPP_INFO(
                get_logger(), "IMU transport: native ROS publisher, domain=%zu", node_domain);
            return;
        }
        imu_raw_pub_ = std::make_unique<ImuPublisher>();
        if (!imu_raw_pub_->init(config_.imu_raw_dds_topic, config_.domain_id))
        {
            throw std::runtime_error("failed to create raw IMU DDS writer");
        }
        if (config_.ahrs_enabled)
        {
            ahrs_ = std::make_unique<camera_sdk::MahonyAhrs>(config_.ahrs_config);
            imu_fused_pub_ = std::make_unique<ImuPublisher>();
            if (!imu_fused_pub_->init(config_.imu_fused_dds_topic, config_.domain_id))
            {
                throw std::runtime_error("failed to create fused IMU DDS writer");
            }
        }
    }
}

bool CameraMppPublisher::write_imu(my_sensor_msgs::msg::Imu& message, bool fused)
{
    const auto& ros_publisher = fused ? imu_fused_ros_pub_ : imu_raw_ros_pub_;
    if (ros_publisher)
    {
        ros_publisher->publish(detail::ToRosImu(message));
        return true;
    }
    return (fused ? imu_fused_pub_ : imu_raw_pub_)->writer_->write(&message);
}

bool CameraMppPublisher::raw_imu_enabled() const
{
    return bool(imu_raw_pub_) || bool(imu_raw_ros_pub_);
}

bool CameraMppPublisher::fused_imu_enabled() const
{
    return bool(imu_fused_pub_) || bool(imu_fused_ros_pub_);
}

std::optional<std::int64_t> CameraMppPublisher::gpio_clock_offset()
{
    std::int64_t best_span = 100001, best_offset = 0;
    for (int i = 0; i < 5; ++i)
    {
        const auto before = camera_sdk::GpioExposure::monotonic_ns();
        const auto ros = now().nanoseconds();
        const auto after = camera_sdk::GpioExposure::monotonic_ns();
        if (after - before < best_span)
        {
            best_span = after - before;
            best_offset = ros - before - (after - before) / 2;
        }
    }
    if (best_span > 100000)
        return std::nullopt;
    return best_offset;
}

std::optional<rclcpp::Time> CameraMppPublisher::process_decoded(const VideoFrameInfo& decoded,
    std::uint32_t descriptor_index, std::optional<std::int64_t> recorded_image_stamp,
    bool publish_images, bool parse_metadata, bool publish_stereo)
{
    rclcpp::Time image_stamp = recorded_image_stamp ? rclcpp::Time(*recorded_image_stamp) : now();
    if (config_.metadata_enabled && parse_metadata)
    {
        camera_sdk::EmbeddedMetadata metadata;
        const auto metadata_offset = static_cast<std::size_t>(config_.metadata_region.y) *
                decoded.stVFrame.u32PlaneStride[0] +
            static_cast<std::size_t>(config_.metadata_region.x);
        camera_sdk::LumaView luma{
            reinterpret_cast<const std::uint8_t*>(decoded.stVFrame.ulPlaneVirAddr[0]) +
                metadata_offset,
            static_cast<std::uint32_t>(config_.metadata_region.width),
            static_cast<std::uint32_t>(config_.metadata_region.height),
            decoded.stVFrame.u32PlaneStride[0], 1};
        std::string metadata_error;
        const bool metadata_ok =
            camera_sdk::decode_embedded_metadata(luma, metadata, &metadata_error);
        if (metadata_ok)
        {
            camera_sdk::CameraClock::FrameTimestamps timestamps;
            // A session sidecar preserves the configured exposure event,
            // not necessarily exposure START. Anchor IMU and images using
            // that same convention, including midpoint recordings.
            const auto reference = config_.timestamp_source == "embedded_exposure_midpoint"
                ? camera_sdk::CameraClock::ExposureReference::Midpoint
                : camera_sdk::CameraClock::ExposureReference::Start;
            bool clock_ok;
            if (recorded_image_stamp)
            {
                clock_ok = clock_.preview_recorded(metadata.exposure_start_raw_us,
                    metadata.exposure_end_raw_us, *recorded_image_stamp, reference, &timestamps,
                    &metadata_error);
            }
            else
            {
                clock_ok =
                    clock_.preview(metadata.exposure_start_raw_us, metadata.exposure_end_raw_us,
                        image_stamp.nanoseconds(), &timestamps, &metadata_error);
            }
            if (!clock_ok)
            {
                if (recorded_image_stamp)
                {
                    throw std::runtime_error(metadata_error);
                }
                if (gpio_sync_)
                {
                    gpio_sync_->reset();
                    clock_.reset();
                    gpio_last_imu_raw_.reset();
                }
                ++metadata_timestamp_rejection_count_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "metadata timestamp rejected before publish: %s "
                    "total=%" PRIu64,
                    metadata_error.c_str(), metadata_timestamp_rejection_count_);
                return std::nullopt;
            }
            if (gpio_sync_)
            {
                const auto offset = gpio_clock_offset();
                if (!offset)
                    return std::nullopt;
                if (std::llabs(*offset - gpio_ros_offset_ns_) > 5000000)
                {
                    gpio_sync_->reset();
                    gpio_ros_offset_ns_ = *offset;
                }
                const auto duration_us = static_cast<std::uint32_t>(
                    metadata.exposure_end_raw_us - metadata.exposure_start_raw_us);
                const auto end_ns = timestamps.exposure_end_extended_us * 1000;
                auto matched = gpio_sync_->match({end_ns - duration_us * 1000LL, end_ns});
                const auto state = gpio_sync_->status();
                if (state != gpio_sync_status_)
                {
                    RCLCPP_INFO(get_logger(), "GPIO exposure sync: %s", state.c_str());
                    gpio_sync_status_ = state;
                }
                const bool startup_ready = !startup_alignment_ ||
                    startup_alignment_->observe(duration_us, matched.has_value());
                if (!matched || !startup_ready)
                {
                    clock_.commit(timestamps); // Unwrap metadata, but publish
                                               // neither images nor IMU.
                    return std::nullopt;
                }
                timestamps.exposure_start_ns = matched->start_ns + gpio_ros_offset_ns_;
                timestamps.exposure_end_ns = matched->end_ns + gpio_ros_offset_ns_;
                timestamps.epoch_offset_ns = timestamps.exposure_end_ns - end_ns;
            }
            if (config_.timestamp_source == "embedded_exposure_start" ||
                config_.timestamp_source == "embedded_exposure_midpoint")
            {
                image_stamp = rclcpp::Time(config_.timestamp_source == "embedded_exposure_midpoint"
                        ? timestamps.exposure_midpoint_ns()
                        : timestamps.exposure_start_ns);
                if (image_stamp.nanoseconds() <=
                    static_cast<std::int64_t>(last_image_timestamp_ns_))
                {
                    ++image_timestamp_regression_count_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "non-monotonic image timestamp rejected before "
                        "metadata commit; total=%" PRIu64,
                        image_timestamp_regression_count_);
                    return std::nullopt;
                }
            }
            // The decoded metadata is the only source of embedded IMU
            // samples. Commit and write the native-timestamp IMU block
            // before V2D or image publication so downstream video work
            // cannot delay its delivery.
            clock_.commit(timestamps);
            publish_imu(metadata);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "embedded metadata decode failed: %s", metadata_error.c_str());
            if (config_.timestamp_source == "embedded_exposure_start" ||
                config_.timestamp_source == "embedded_exposure_midpoint")
            {
                return std::nullopt;
            }
        }
    }

    if (!publish_images)
        return image_stamp;

    // Embedded exposure time is the synchronization authority.  A VDEC
    // recycle/source-change can occasionally return an older metadata
    // block; forwarding it would make downstream visual tracking reject
    // the stereo pair.  Drop it here instead of substituting a receive
    // timestamp.
    const std::uint64_t image_timestamp_ns = static_cast<std::uint64_t>(image_stamp.nanoseconds());
    if (image_timestamp_ns <= last_image_timestamp_ns_)
    {
        ++image_timestamp_regression_count_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "non-monotonic image timestamp dropped; total=%" PRIu64,
            image_timestamp_regression_count_);
        return std::nullopt;
    }
    last_image_timestamp_ns_ = image_timestamp_ns;

    if (config_.full_only || !publish_stereo)
    {
        if (!config_.full_enabled || !full_pub_)
            return image_stamp;
        const auto full =
            normalize_decoded_nv12_frame(decoded, static_cast<std::uint32_t>(config_.width),
                static_cast<std::uint32_t>(config_.height), descriptor_index);
        U64 token = 0;
        if (VB_Export(decoded.ulBufferId, &token) != 0)
        {
            throw std::runtime_error("full VB_Export failed");
        }
        try
        {
            exports_.push_back({0, 0, decoded.ulBufferId});
        }
        catch (...)
        {
            (void)VB_Unexport(decoded.ulBufferId);
            throw;
        }
        while (static_cast<int>(exports_.size()) > config_.max_exported_pairs)
        {
            release_oldest_export();
        }
        publish_descriptor(
            *full_pub_, full, token, image_stamp, config_.full.frame_id, descriptor_index);
        return image_stamp;
    }

    UL first_buffer = VB_GetBuffer(eye_pool_, 100);
    UL second_buffer = VB_GetBuffer(eye_pool_, 100);
    if (first_buffer == 0 || second_buffer == 0)
    {
        if (first_buffer != 0)
            (void)VB_ReleaseBuffer(first_buffer);
        if (second_buffer != 0)
            (void)VB_ReleaseBuffer(second_buffer);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "eye VB pool exhausted");
        return std::nullopt;
    }

    try
    {
        VideoFrameInfo first = make_nv12_frame(eye_pool_, first_buffer, config_.left.output_width,
            config_.left.output_height, descriptor_index, decoded.stVFrame.u64PTS);
        VideoFrameInfo second =
            make_nv12_frame(eye_pool_, second_buffer, config_.right.output_width,
                config_.right.output_height, descriptor_index, decoded.stVFrame.u64PTS);
        if (!crop_eyes(decoded, first, second))
        {
            throw std::runtime_error("V2D stereo crop failed");
        }
        (void)VB_UpdateBufferFrameInfo(first_buffer, &first);
        (void)VB_UpdateBufferFrameInfo(second_buffer, &second);

        VideoFrameInfo& left = first;
        VideoFrameInfo& right = second;
        // VDEC does not guarantee that every public frame-info field is
        // populated on the source-change/recycled output path.  The
        // exported full-frame descriptor must nevertheless carry a
        // complete NV12 layout for consumers such as LAS2.  Rebuild the
        // descriptor from the live VB block instead of forwarding the
        // partially populated VDEC metadata.
        VideoFrameInfo full = decoded;
        if (config_.full_enabled)
        {
            full = normalize_decoded_nv12_frame(decoded, static_cast<std::uint32_t>(config_.width),
                static_cast<std::uint32_t>(config_.height), descriptor_index);
            RCLCPP_INFO_ONCE(get_logger(),
                "full decoded NV12 layout: %ux%u stride=%u/%u "
                "size=%u/%u total=%u",
                full.stCommFrameInfo.u32Width, full.stCommFrameInfo.u32Height,
                full.stVFrame.u32PlaneStride[0], full.stVFrame.u32PlaneStride[1],
                full.stVFrame.u32PlaneSize[0], full.stVFrame.u32PlaneSize[1],
                full.stVFrame.u32TotalSize);
        }
        U64 left_token = 0;
        U64 right_token = 0;
        U64 full_token = 0;
        if (VB_Export(left.ulBufferId, &left_token) != 0)
        {
            throw std::runtime_error("left VB_Export failed");
        }
        if (VB_Export(right.ulBufferId, &right_token) != 0)
        {
            (void)VB_Unexport(left.ulBufferId);
            throw std::runtime_error("right VB_Export failed");
        }
        if (config_.full_enabled && VB_Export(decoded.ulBufferId, &full_token) != 0)
        {
            (void)VB_Unexport(left.ulBufferId);
            (void)VB_Unexport(right.ulBufferId);
            throw std::runtime_error("full VB_Export failed");
        }
        bool export_recorded = false;
        try
        {
            exports_.push_back(
                {left.ulBufferId, right.ulBufferId, config_.full_enabled ? decoded.ulBufferId : 0});
            export_recorded = true;

            publish_descriptor(
                *left_pub_, left, left_token, image_stamp, config_.left.frame_id, descriptor_index);
            publish_descriptor(*right_pub_, right, right_token, image_stamp, config_.right.frame_id,
                descriptor_index);
            if (config_.full_enabled && full_pub_)
            {
                publish_descriptor(*full_pub_, full, full_token, image_stamp, config_.full.frame_id,
                    descriptor_index);
            }
            if (left_info_pub_ && right_info_pub_)
            {
                left_camera_info_.header.stamp = image_stamp;
                right_camera_info_.header.stamp = image_stamp;
                left_info_pub_->publish(left_camera_info_);
                right_info_pub_->publish(right_camera_info_);
            }
        }
        catch (...)
        {
            // The descriptor queue owns the export references only after
            // the pair is inserted.  If publishing/allocation fails,
            // revoke all exports immediately to avoid pinning VDEC/VB.
            if (export_recorded)
            {
                const auto pair = exports_.back();
                exports_.pop_back();
                if (pair.left_handle)
                    (void)VB_Unexport(pair.left_handle);
                if (pair.right_handle)
                    (void)VB_Unexport(pair.right_handle);
                if (pair.full_handle != 0)
                {
                    (void)VB_Unexport(pair.full_handle);
                }
            }
            else
            {
                (void)VB_Unexport(left.ulBufferId);
                (void)VB_Unexport(right.ulBufferId);
                if (config_.full_enabled)
                {
                    (void)VB_Unexport(decoded.ulBufferId);
                }
            }
            throw;
        }
    }
    catch (const std::exception& error)
    {
        if (recorded_image_stamp)
        {
            (void)VB_ReleaseBuffer(first_buffer);
            (void)VB_ReleaseBuffer(second_buffer);
            throw;
        }
        RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
    catch (...)
    {
        (void)VB_ReleaseBuffer(first_buffer);
        (void)VB_ReleaseBuffer(second_buffer);
        throw;
    }

    (void)VB_ReleaseBuffer(first_buffer);
    (void)VB_ReleaseBuffer(second_buffer);
    while (static_cast<int>(exports_.size()) > config_.max_exported_pairs)
    {
        release_oldest_export();
    }
    return image_stamp;
}

void CameraMppPublisher::publish_descriptor(
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>& publisher, const VideoFrameInfo& frame,
    U64 token, const rclcpp::Time& stamp, const std::string& frame_id,
    std::uint32_t descriptor_index)
{
    mpp_msgs::msg::MppVideoFrameDesc message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.version = 1;
    message.token = token;
    message.pool_id = frame.ulPoolId;
    message.buffer_id = frame.ulBufferId;
    message.frame_index = descriptor_index;
    message.width = frame.stCommFrameInfo.u32Width;
    message.height = frame.stCommFrameInfo.u32Height;
    message.align = frame.stCommFrameInfo.u32Align;
    message.pixel_format = frame.stCommFrameInfo.ePixelFormat;
    message.compress_mode = frame.stCommFrameInfo.eCompressMode;
    message.color_space = frame.stCommFrameInfo.eColorSpace;
    message.pts = frame.stVFrame.u64PTS;
    message.total_size = frame.stVFrame.u32TotalSize;
    message.plane_num = frame.stVFrame.u32PlaneNum;
    for (std::size_t i = 0; i < message.plane_stride.size() && i < frame.stVFrame.u32PlaneNum; ++i)
    {
        message.plane_stride[i] = frame.stVFrame.u32PlaneStride[i];
        message.plane_size[i] = frame.stVFrame.u32PlaneSize[i];
        message.plane_size_valid[i] = frame.stVFrame.u32PlaneSizeValid[i];
        message.plane_phy_addr[i] = frame.stVFrame.u64PlanePhyAddr[i];
    }
    publisher.publish(std::move(message));
}

void CameraMppPublisher::publish_imu(const camera_sdk::EmbeddedMetadata& metadata)
{
    if (!config_.imu_enabled || !raw_imu_enabled())
        return;
    imu_metadata_sample_count_ += metadata.imu_count;
    for (std::size_t i = 0; i < metadata.imu_count; ++i)
    {
        const auto& sample = metadata.imu[i];
        if (!sample.valid || !clock_.initialized())
        {
            ++imu_invalid_sample_count_;
            continue;
        }
        if (gpio_sync_ && gpio_last_imu_raw_)
        {
            const auto delta =
                static_cast<std::int32_t>(sample.timestamp_raw_us - *gpio_last_imu_raw_);
            if (delta <= 0)
            {
                if (delta == 0)
                    ++imu_duplicate_count_;
                else
                    ++imu_backward_count_;
                continue;
            }
        }
        const std::int64_t timestamp_ns = clock_.ros_time_ns(sample.timestamp_raw_us);
        if (timestamp_ns <= 0 || static_cast<std::uint64_t>(timestamp_ns) <= last_imu_timestamp_ns_)
        {
            if (static_cast<std::uint64_t>(timestamp_ns) == last_imu_timestamp_ns_)
            {
                ++imu_duplicate_count_;
            }
            else
            {
                ++imu_backward_count_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "backward IMU timestamp dropped; total=%" PRIu64, imu_backward_count_);
            }
            continue;
        }
        last_imu_timestamp_ns_ = static_cast<std::uint64_t>(timestamp_ns);
        if (gpio_sync_)
            gpio_last_imu_raw_ = sample.timestamp_raw_us;

        my_sensor_msgs::msg::Imu message;
        message.header().stamp().sec(static_cast<std::int32_t>(timestamp_ns / 1000000000LL));
        message.header().stamp().nanosec(static_cast<std::uint32_t>(timestamp_ns % 1000000000LL));
        message.header().frame_id(config_.imu_frame_id);
        message.orientation().w(1.0);
        message.orientation_covariance().fill(0.0);
        message.orientation_covariance()[0] = -1.0;
        message.angular_velocity().x(sample.gyro_rad_s[0]);
        message.angular_velocity().y(sample.gyro_rad_s[1]);
        message.angular_velocity().z(sample.gyro_rad_s[2]);
        message.angular_velocity_covariance().fill(0.0);
        message.linear_acceleration().x(sample.accel_m_s2[0]);
        message.linear_acceleration().y(sample.accel_m_s2[1]);
        message.linear_acceleration().z(sample.accel_m_s2[2]);
        message.linear_acceleration_covariance().fill(0.0);
        const bool raw_written = write_imu(message, false);
        if (!raw_written)
        {
            ++imu_raw_write_failures_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "raw IMU DDS write failed; total=%" PRIu64, imu_raw_write_failures_);
        }
        else
        {
            ++imu_raw_write_successes_;
        }

        if (ahrs_ && fused_imu_enabled() &&
            ahrs_->update(sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2],
                sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2], timestamp_ns))
        {
            const auto& orientation = ahrs_->orientation();
            message.orientation().w(orientation.w);
            message.orientation().x(orientation.x);
            message.orientation().y(orientation.y);
            message.orientation().z(orientation.z);
            message.orientation_covariance().fill(0.0);
            message.orientation_covariance()[0] = config_.orientation_variance;
            message.orientation_covariance()[4] = config_.orientation_variance;
            message.orientation_covariance()[8] = config_.orientation_variance;
            if (!write_imu(message, true))
            {
                ++imu_fused_write_failures_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "fused IMU DDS write failed; total=%" PRIu64, imu_fused_write_failures_);
            }
        }
    }
}

} // namespace camera_io
