/**
 * @file session_replay_publisher.cpp
 * @brief Replay source-timed camera frames and recorded session metadata.
 */
#include "camera_mpp_publisher.h"
#include "imu_transport.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "session_replay_control.h"
#include "session_replay_imu.h"
#include "session_replay_index.h"
#include "session_replay_pose.h"
#include "session_replay_window.h"
#include "spacemit_camera/kalibr_time_alignment.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sys/sys_api.h"
#include "sys/vb_api.h"
#include "vdec/vdec_api.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <fstream>
#include <limits>
#include <linux/dma-buf.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/mathematics.h>
}

namespace camera_io
{
class SessionReplayPublisher final : public CameraMppPublisher
{
public:
    explicit SessionReplayPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : CameraMppPublisher(options, "session_replay_publisher")
    {
        declare_parameter<std::string>("session_directory", "");
        declare_parameter<std::string>("offline_source_manifest", "");
        declare_parameter<std::string>("output_directory", "");
        declare_parameter<bool>("replay_publish_imu", true);
        declare_parameter<std::string>("replay_pose_file", "");
        declare_parameter<std::string>("replay_pose_topic", "/replay/final_pose");
        declare_parameter<std::string>("replay_pose_frame_id", "map");
        declare_parameter<std::string>("replay_pose_child_frame_id", "camera_rig");
        declare_parameter<bool>("replay_require_pose_subscriber", true);
        declare_parameter<std::string>("offline_capture_directory", "");
        declare_parameter<int>("offline_capture_every_n", 8);
        declare_parameter<std::string>("replay_ack_topic", "/replay/vio/ack");
        declare_parameter<std::string>("replay_ready_topic", "/replay/vio/ready");
        declare_parameter<std::string>("replay_completion_topic", "");
        declare_parameter<std::string>("replay_imu_ack_topic", "/replay/vio/imu_ack");
        declare_parameter<double>("replay_timeout_sec", 30.0);
        declare_parameter<bool>("replay_publish_full", false);
        declare_parameter<bool>("replay_realtime", false);
        declare_parameter<std::string>("replay_display_ready_topic", "");
        declare_parameter<int>("replay_prefeed_frames", 4);
        declare_parameter<int>("replay_imu_lookahead_frames", 4);
        declare_parameter<int>("replay_max_inflight", 1);
        declare_parameter<std::string>("session_camera_name", "");
        declare_parameter<std::string>("replay_control_topic", "/replay/control");
        declare_parameter<std::string>("replay_control_state_topic", "/replay/control_state");
        declare_parameter<std::string>("replay_pause_topic", "/replay/paused");
        declare_parameter<bool>("replay_start_paused", false);
        declare_parameter<double>("replay_fast_forward_sec", 5.0);
        status_ = rclcpp::create_publisher<std_msgs::msg::String>(
            *this, "/replay/status", rclcpp::QoS(1).reliable().transient_local());
        control_state_ = rclcpp::create_publisher<std_msgs::msg::String>(*this,
            get_parameter("replay_control_state_topic").as_string(),
            rclcpp::QoS(1).reliable().transient_local());
        paused_ = rclcpp::create_publisher<std_msgs::msg::Bool>(*this,
            get_parameter("replay_pause_topic").as_string(),
            rclcpp::QoS(1).reliable().transient_local());
    }

    ~SessionReplayPublisher() override { stop(); }
    int exit_code() const { return exit_code_.load(); }
    void stop_and_join() noexcept { stop(); }

protected:
    using Steady = std::chrono::steady_clock;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override
    {
        try
        {
            output_ = get_parameter("output_directory").as_string();
            if (output_.empty() || !std::filesystem::is_directory(output_) ||
                std::filesystem::exists(output_ / "replay_summary.json"))
                throw std::runtime_error("output_directory must exist with no replay_summary.json");
            output_valid_ = true;
            realtime_ = get_parameter("replay_realtime").as_bool();
            fast_forward_sec_ = get_parameter("replay_fast_forward_sec").as_double();
            if (!std::isfinite(fast_forward_sec_) || fast_forward_sec_ < 1e-9 ||
                fast_forward_sec_ > 86400.0)
                throw std::runtime_error("replay_fast_forward_sec must be in [1e-9, 86400]");
            fast_forward_ns_ = static_cast<std::int64_t>(fast_forward_sec_ * 1e9);
            control_ = replay::Control(get_parameter("replay_start_paused").as_bool());
            const auto display_topic = get_parameter("replay_display_ready_topic").as_string();
            if (!display_topic.empty())
            {
                display_ready_ = create_subscription<std_msgs::msg::Bool>(display_topic,
                    rclcpp::QoS(1).reliable().transient_local(),
                    [this](std_msgs::msg::Bool::ConstSharedPtr message)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        display_ready_received_ = message->data;
                        cv_.notify_all();
                    });
            }
            const double seconds = get_parameter("replay_timeout_sec").as_double();
            if (!std::isfinite(seconds) || seconds <= 0 || seconds > 86400)
                throw std::runtime_error("replay_timeout_sec must be in (0, 86400]");
            timeout_ = std::chrono::duration_cast<Steady::duration>(
                std::chrono::duration<double>(seconds));
            prefeed_ = get_parameter("replay_prefeed_frames").as_int();
            lookahead_ = get_parameter("replay_imu_lookahead_frames").as_int();
            if (prefeed_ < 1 || lookahead_ < 0 || prefeed_ + lookahead_ > 48)
                throw std::runtime_error(
                    "invalid replay prefeed/lookahead bound (sum must be <=48)");
            load_source();
            publish_imu_ = get_parameter("replay_publish_imu").as_bool();
            require_pose_subscriber_ = get_parameter("replay_require_pose_subscriber").as_bool();
            const auto requested_inflight = get_parameter("replay_max_inflight").as_int();
            if (requested_inflight < 1 || requested_inflight > 16)
                throw std::runtime_error("replay_max_inflight must be in [1,16]");
            max_inflight_ = static_cast<int>(requested_inflight);
            const auto pose_file = get_parameter("replay_pose_file").as_string();
            if (!pose_file.empty())
            {
                poses_ = replay::read_poses(pose_file, targets_);
                pose_frame_id_ = get_parameter("replay_pose_frame_id").as_string();
                pose_child_frame_id_ = get_parameter("replay_pose_child_frame_id").as_string();
                if (pose_frame_id_.empty() || pose_child_frame_id_.empty())
                    throw std::runtime_error("frozen pose frame IDs must be nonempty");
                pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
                    get_parameter("replay_pose_topic").as_string(),
                    rclcpp::QoS(max_inflight_ > 1 ? 64 : 1).reliable());
            }
            if (!publish_imu_ && poses_.empty())
                throw std::runtime_error(
                    "replay_publish_imu=false requires an exact frozen pose CSV");
            if (max_inflight_ > 1)
            {
                if (publish_imu_ || poses_.empty() || realtime_ ||
                    !get_parameter("replay_publish_full").as_bool())
                    throw std::runtime_error("multi-inflight replay requires frozen poses, "
                                                "full images, IMU disabled and realtime=false");
                window_ = std::make_unique<replay::AckWindow>(targets_, max_inflight_, timeout_);
            }
            const auto completion_topic = get_parameter("replay_completion_topic").as_string();
            if (!completion_topic.empty())
            {
                if (publish_imu_ || poses_.empty())
                    throw std::runtime_error(
                        "replay_completion_topic requires frozen poses and IMU disabled");
                completion_gate_ = std::make_unique<replay::CompletionGate>(true, timeout_);
                completion_sub_ = create_subscription<std_msgs::msg::String>(completion_topic,
                    rclcpp::QoS(1).reliable().transient_local(),
                    [this](std_msgs::msg::String::ConstSharedPtr message)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        try
                        {
                            completion_gate_->receive(message->data, Steady::now());
                        }
                        catch (const std::exception& error)
                        {
                            peer_error_ = error.what();
                        }
                        cv_.notify_all();
                    });
            }
            prepare_capture();
            auto qos = rclcpp::QoS(max_inflight_ > 1 ? 256 : 16).reliable().transient_local();
            ack_ = create_subscription<std_msgs::msg::String>(
                get_parameter("replay_ack_topic").as_string(), qos,
                [this](std_msgs::msg::String::ConstSharedPtr message)
                { receive_ack(message->data); });
            if (publish_imu_)
                imu_ack_ = create_subscription<std_msgs::msg::String>(
                    get_parameter("replay_imu_ack_topic").as_string(),
                    rclcpp::QoS(1).reliable().transient_local(),
                    [this](std_msgs::msg::String::ConstSharedPtr message)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        try
                        {
                            imu_received_ = std::max(imu_received_, replay::number(message->data));
                        }
                        catch (const std::exception& error)
                        {
                            peer_error_ = error.what();
                        }
                        cv_.notify_all();
                    });
            ready_ = create_subscription<std_msgs::msg::String>(
                get_parameter("replay_ready_topic").as_string(),
                rclcpp::QoS(1).reliable().transient_local(),
                [this](std_msgs::msg::String::ConstSharedPtr message)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (message->data == "ready")
                        ready_received_ = true;
                    else if (message->data.rfind("error:", 0) == 0)
                        peer_error_ = message->data;
                    cv_.notify_all();
                });
            if (CameraMppPublisher::on_configure(state) != CallbackReturn::SUCCESS)
                throw std::runtime_error(
                    "camera configuration or decoder initialization failed (see log)");
            commands_ = create_subscription<std_msgs::msg::String>(
                get_parameter("replay_control_topic").as_string(), rclcpp::QoS(16).reliable(),
                [this](std_msgs::msg::String::ConstSharedPtr message)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const auto stamp =
                        acknowledged_ ? targets_[acknowledged_ - 1].stamp : targets_.front().stamp;
                    if (!control_.command(
                            message->data, stamp, targets_.back().stamp, fast_forward_ns_))
                    {
                        RCLCPP_WARN(get_logger(), "Replay command '%s' ignored in %s",
                            message->data.c_str(), control_.name());
                        return;
                    }
                    reanchor_ = true;
                    publish_control_locked();
                    cv_.notify_all();
                });
            {
                std::lock_guard<std::mutex> lock(mutex_);
                publish_control_locked();
            }
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& error)
        {
            finish(error.what());
            return CallbackReturn::FAILURE;
        }
    }

    void prepare_configuration() override
    {
        time_alignment_ = camera_sdk::KalibrTimeAlignment{};
        if (calibration_ && !calibration_->cameras.empty())
            time_alignment_ =
                camera_sdk::KalibrTimeAlignment(calibration_->cameras.front().timeshift_cam_imu);
        RCLCPP_INFO(get_logger(),
            "Replay Kalibr timing: cam0.timeshift_cam_imu=%" PRId64
            "ns; raw IMU coverage = image timestamp + shift",
            time_alignment_.shift_ns());
        // A snapshot may describe a live UVC source. Replay never opens it.
        config_.input_type = "file";
        config_.input_file = video_.string();
        config_.compressed_enabled = false;
        // Legacy session replay keeps its original direct-send behavior. Original
        // MP4 input can use the existing application SYS ring for larger packets.
        if (!original_mp4_)
            config_.decoder_input_pool.enabled = false;
        config_.full_enabled = get_parameter("replay_publish_full").as_bool();
        if (original_mp4_ && source_timestamp_reference_ != config_.timestamp_source)
            throw std::runtime_error("source manifest exposure reference disagrees "
                                        "with camera configuration");
        if (!config_.decode_enabled || config_.full_only || !config_.metadata_enabled ||
            !config_.imu_enabled ||
            (config_.timestamp_source != "embedded_exposure_start" &&
                config_.timestamp_source != "embedded_exposure_midpoint") ||
            config_.input_format != "mjpeg")
            throw std::runtime_error("session replay requires MJPEG stereo with "
                                        "embedded exposure timestamps and IMU");
        config_.decoder_buffer_count =
            std::max(config_.decoder_buffer_count, prefeed_ + lookahead_ + 4);
        if (window_)
        {
            if (config_.max_exported_pairs < max_inflight_)
                throw std::runtime_error("buffers.max_exported_pairs must cover "
                                            "replay_max_inflight");
            const auto minimum = prefeed_ + lookahead_ + max_inflight_ + 4;
            if (minimum > 64)
                throw std::runtime_error("frozen replay window exceeds VDEC's 64 buffer bound");
            config_.decoder_buffer_count = std::max(config_.decoder_buffer_count, minimum);
            RCLCPP_INFO(get_logger(),
                "Frozen replay window=%d export_retention=%d decoder_buffers=%d", max_inflight_,
                config_.max_exported_pairs, config_.decoder_buffer_count);
        }
        // Match the recorded raw topic, not the remapped replay topic or camera
        // name.
        imu_sidecar_.reset();
        imu_tail_samples_ = 0;
        if (publish_imu_ && !original_mp4_ && config_.metadata_parser == "camera_sdk_embedded" &&
            std::filesystem::exists(session_ / "recorder.json"))
        {
            boost::property_tree::ptree recorder, recorded_camera;
            boost::property_tree::read_json(session_path("camera.json").string(), recorded_camera);
            const auto topic = recorded_camera.get<std::string>("imu.raw.ros_topic");
            boost::property_tree::read_json(session_path("recorder.json").string(), recorder);
            if (const auto imus = recorder.get_child_optional("imus"))
            {
                for (const auto& entry : *imus)
                {
                    if (entry.second.get<bool>("enabled", true) &&
                        entry.second.get<std::string>("topic") == topic)
                    {
                        if (imu_sidecar_)
                            throw std::runtime_error("ambiguous raw IMU sidecar");
                        imu_sidecar_ = std::make_unique<replay::EmbeddedImuSidecar>(
                            session_path(entry.second.get<std::string>("name") + "_imu.txt")
                                .string());
                    }
                }
            }
        }
    }

    rclcpp::QoS publication_qos() const override
    {
        return rclcpp::QoS(max_inflight_ > 1 ? 64 : 16).reliable();
    }

    void initialize_imu() override
    {
        if (!publish_imu_)
            return;
        raw_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(*this, config_.imu_raw_ros_topic,
            rclcpp::QoS(std::max(64, config_.imu_queue_depth)).reliable());
        if (config_.ahrs_enabled)
        {
            ahrs_ = std::make_unique<camera_sdk::MahonyAhrs>(config_.ahrs_config);
            fused_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(
                *this, config_.imu_fused_ros_topic, rclcpp::QoS(64).reliable());
        }
    }
    bool raw_imu_enabled() const override { return bool(raw_); }
    bool fused_imu_enabled() const override { return bool(fused_); }

    bool write_imu(my_sensor_msgs::msg::Imu& source, bool fused) override
    {
        // Validate the complete camera tail, but do not fill VIO's bounded
        // pending IMU queue after its final Track has already completed.
        if (acknowledged_ == targets_.size())
            return true;
        auto message = detail::ToRosImu(source);
        if (!fused && imu_sidecar_)
        {
            const auto stamp =
                static_cast<std::uint64_t>(message.header.stamp.sec) * 1000000000ULL +
                message.header.stamp.nanosec;
            imu_sidecar_->verify(stamp,
                {static_cast<float>(message.angular_velocity.x),
                    static_cast<float>(message.angular_velocity.y),
                    static_cast<float>(message.angular_velocity.z),
                    static_cast<float>(message.linear_acceleration.x),
                    static_cast<float>(message.linear_acceleration.y),
                    static_cast<float>(message.linear_acceleration.z)});
        }
        (fused ? fused_ : raw_)->publish(message);
        return true;
    }

    void open_pipeline() override
    {
        require_ok(SYS_Init(), "SYS_Init");
        sys_initialized_ = true;
        require_ok(VB_Init(), "VB_Init");
        vb_initialized_ = true;
        require_ok(VDEC_Init(), "VDEC_Init");
        vdec_initialized_ = true;
        open_decoder();
        if (config_.decoder_input_pool.enabled)
        {
            demux_source_ = {MPP_ID_DEMUX, 0, kDemuxChn};
            vdec_sink_ = {MPP_ID_VDEC, 0, vdec_channel_id_};
            require_ok(
                SYS_Bind(&demux_source_, &vdec_sink_), "SYS_Bind(offline original packets->VDEC)");
            demux_vdec_bound_ = true;
            configure_input_pool(demux_source_, vdec_sink_);
        }
        open_eye_pool();
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
    {
        reset_capture_stats();
        worker_ = std::thread([this] { run(); });
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override
    {
        stop();
        return CameraMppPublisher::on_deactivate(state);
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override
    {
        stop();
        return CameraMppPublisher::on_cleanup(state);
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override
    {
        stop();
        return CameraMppPublisher::on_shutdown(state);
    }
    CallbackReturn on_error(const rclcpp_lifecycle::State& state) override
    {
        stop();
        return CameraMppPublisher::on_error(state);
    }

private:
    struct Decoded
    {
        VideoFrameInfo frame{};
        S32 channel;
        std::size_t index;
        Decoded(S32 ch, std::size_t i) : channel(ch), index(i) {}
        ~Decoded()
        {
            if (frame.ulBufferId)
                (void)VDEC_ReleaseFrame(channel, frame.ulBufferId);
        }
        Decoded(const Decoded&) = delete;
        Decoded& operator=(const Decoded&) = delete;
    };
    struct Input
    {
        AVFormatContext* format{avformat_alloc_context()};
        AVPacket* packet{av_packet_alloc()};
        ~Input()
        {
            av_packet_free(&packet);
            avformat_close_input(&format);
        }
    };

    void load_source()
    {
        const auto source = get_parameter("offline_source_manifest").as_string();
        const auto session = get_parameter("session_directory").as_string();
        if (!source.empty())
        {
            if (!session.empty())
                throw std::runtime_error(
                    "select only offline_source_manifest or session_directory");
            const auto manifest_path = std::filesystem::canonical(source);
            source_manifest_ = manifest_path;
            boost::property_tree::ptree manifest;
            boost::property_tree::read_json(manifest_path.string(), manifest);
            if (manifest.get<int>("format_version") != 1 ||
                manifest.get<std::string>("container") != "mp4" ||
                manifest.get<std::string>("codec") != "mjpeg")
                throw std::runtime_error("unsupported original MP4 source manifest");
            source_timestamp_reference_ = manifest.get<std::string>("timestamp_source");
            if (source_timestamp_reference_ != "embedded_exposure_start" &&
                source_timestamp_reference_ != "embedded_exposure_midpoint")
                throw std::runtime_error("unsupported source exposure reference");
            auto source_path = [&](const std::string& key)
            {
                const auto path = std::filesystem::canonical(
                    manifest_path.parent_path() / manifest.get<std::string>(key));
                if (!std::filesystem::is_regular_file(path))
                    throw std::runtime_error("invalid source manifest file: " + key);
                return path;
            };
            video_ = source_path("video");
            source_video_sha256_ = manifest.get<std::string>("video_sha256", "");
            targets_ = replay::read_targets(source_path("targets").string());
            frames_ = replay::read_camera(source_path("timestamps").string());
            for (const auto& frame : frames_)
                if (frame.adjusted)
                    throw std::runtime_error(
                        "original MP4 index must preserve unadjusted source PTS");
            if (!replay::associate(frames_, targets_))
                throw std::runtime_error("original MP4 index does not contain every exact target");
            original_mp4_ = true;
            return;
        }
        session_ = std::filesystem::canonical(session);
        if (session_ == std::filesystem::canonical(output_))
            throw std::runtime_error("output_directory must differ from source session");
        targets_ = replay::read_targets((session_ / "vio_frames.csv").string());
        boost::property_tree::ptree manifest;
        boost::property_tree::read_json((session_ / "manifest.json").string(), manifest);
        if (manifest.get<int>("format_version") != 1)
            throw std::runtime_error("unsupported session manifest version");
        const auto name = get_parameter("session_camera_name").as_string();
        for (const auto& entry : manifest.get_child("cameras"))
        {
            const auto& camera = entry.second;
            if (!name.empty() && camera.get<std::string>("name") != name)
                continue;
            auto candidate =
                replay::read_camera(session_path(camera.get<std::string>("timestamps")).string());
            if (!replay::associate(candidate, targets_))
                continue;
            if (!frames_.empty())
                throw std::runtime_error("ambiguous session camera; set session_camera_name");
            if (camera.get<std::string>("codec") != "mjpeg" ||
                camera.get<std::string>("container") != "matroska")
                throw std::runtime_error("session replay requires original MJPEG Matroska");
            frames_ = std::move(candidate);
            video_ = session_path(camera.get<std::string>("video"));
        }
        if (frames_.empty())
            throw std::runtime_error("no camera index contains every exact VIO target");
    }

    void prepare_capture()
    {
        const auto directory = get_parameter("offline_capture_directory").as_string();
        if (directory.empty())
            return;
        if (!original_mp4_)
            throw std::runtime_error("offline capture requires an original MP4 source manifest");
        capture_every_ = get_parameter("offline_capture_every_n").as_int();
        if (capture_every_ < 1)
            throw std::runtime_error("offline_capture_every_n must be positive");
        capture_ = std::filesystem::canonical(directory);
        if (!std::filesystem::is_directory(capture_) || !std::filesystem::is_empty(capture_))
            throw std::runtime_error(
                "offline_capture_directory must be an existing empty directory");
        capture_valid_ = true;
        if (!std::filesystem::create_directory(capture_ / "left") ||
            !std::filesystem::create_directory(capture_ / "right"))
            throw std::runtime_error("cannot create offline capture eye directories");
        capture_pairs_.exceptions(std::ios::failbit | std::ios::badbit);
        capture_pairs_.open(capture_ / "pairs.csv");
        capture_pairs_ << "source_frame_index,image_timestamp_ns,width,height,"
                            "left_source_stride,right_source_stride,left_path,"
                            "right_path,pixel_format\n";
        capture_pairs_.flush();
    }

    static void cache_read(int fd, bool start)
    {
        dma_buf_sync sync{};
        sync.flags = (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) | DMA_BUF_SYNC_READ;
        int result;
        do
        {
            result = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
        } while (result < 0 && errno == EINTR);
        if (result < 0)
            throw std::runtime_error("offline capture DMA-BUF read sync failed");
    }

    // The producer still owns the VB export. Copy synchronously before advancing
    // to another target; no consumer token is retained beyond its
    // acknowledgement.
    std::pair<VideoFrameInfo, std::vector<std::uint8_t>> capture_eye(UL handle)
    {
        VideoFrameInfo info{};
        void* address = nullptr;
        S32 fd = -1;
        require_ok(VB_GetFrameInfo(handle, &info), "VB_GetFrameInfo(capture)");
        require_ok(VB_GetDmaBufFd(handle, &fd), "VB_GetDmaBufFd(capture)");
        require_ok(VB_GetVirAddr(handle, &address), "VB_GetVirAddr(capture)");
        const auto width = info.stCommFrameInfo.u32Width;
        const auto height = info.stCommFrameInfo.u32Height;
        const auto stride = info.stVFrame.u32PlaneStride[0];
        struct stat allocation{};
        if (!address || fd < 0 || !width || !height || stride < width ||
            fstat(fd, &allocation) != 0 || allocation.st_size <= 0 ||
            static_cast<std::uint64_t>(stride) * height >
                static_cast<std::uint64_t>(allocation.st_size))
            throw std::runtime_error("invalid offline capture luma allocation");
        std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
        try
        {
            cache_read(fd, true);
        }
        catch (...)
        {
            ++capture_cache_start_errors_;
            throw;
        }
        const auto* source = static_cast<const std::uint8_t*>(address);
        for (std::uint32_t row = 0; row < height; ++row)
            std::memcpy(pixels.data() + static_cast<std::size_t>(row) * width,
                source + static_cast<std::size_t>(row) * stride, width);
        try
        {
            cache_read(fd, false);
        }
        catch (...)
        {
            ++capture_cache_end_errors_;
            throw;
        }
        return {info, std::move(pixels)};
    }

    void capture_pair(const replay::Frame& row)
    {
        if (capture_.empty() || row.id % capture_every_ != 0)
            return;
        if (exports_.empty() || !exports_.back().left_handle || !exports_.back().right_handle)
            throw std::runtime_error("offline capture has no retained stereo pair");
        const auto left = capture_eye(exports_.back().left_handle);
        const auto right = capture_eye(exports_.back().right_handle);
        if (left.first.stCommFrameInfo.u32Width != right.first.stCommFrameInfo.u32Width ||
            left.first.stCommFrameInfo.u32Height != right.first.stCommFrameInfo.u32Height ||
            left.first.stCommFrameInfo.ePixelFormat != right.first.stCommFrameInfo.ePixelFormat)
            throw std::runtime_error("offline capture eye layouts disagree");
        const auto filename = std::to_string(row.id) + ".gray8";
        auto write = [&](const char* eye, const std::vector<std::uint8_t>& pixels)
        {
            std::ofstream file;
            file.exceptions(std::ios::failbit | std::ios::badbit);
            file.open(capture_ / eye / filename, std::ios::binary | std::ios::out);
            file.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
            file.close();
        };
        try
        {
            write("left", left.second);
            write("right", right.second);
            capture_pairs_ << row.id << ',' << row.stamp << ','
                            << left.first.stCommFrameInfo.u32Width << ','
                            << left.first.stCommFrameInfo.u32Height << ','
                            << left.first.stVFrame.u32PlaneStride[0] << ','
                            << right.first.stVFrame.u32PlaneStride[0] << ",left/" << filename
                            << ",right/" << filename << ','
                            << left.first.stCommFrameInfo.ePixelFormat << '\n';
            capture_pairs_.flush();
        }
        catch (...)
        {
            ++capture_write_errors_;
            throw;
        }
        ++captured_pairs_;
        captured_bytes_ += left.second.size() + right.second.size();
    }

    void publish_pose(std::size_t index)
    {
        if (!pose_pub_)
            return;
        const auto& pose = poses_.at(index);
        nav_msgs::msg::Odometry message;
        message.header.stamp = rclcpp::Time(pose.stamp);
        message.header.frame_id = pose_frame_id_;
        message.child_frame_id = pose_child_frame_id_;
        message.pose.pose.position.x = pose.values[0];
        message.pose.pose.position.y = pose.values[1];
        message.pose.pose.position.z = pose.values[2];
        message.pose.pose.orientation.x = pose.values[3];
        message.pose.pose.orientation.y = pose.values[4];
        message.pose.pose.orientation.z = pose.values[5];
        message.pose.pose.orientation.w = pose.values[6];
        pose_pub_->publish(message);
    }

    std::filesystem::path session_path(const std::string& relative) const
    {
        const auto path = std::filesystem::canonical(session_ / relative);
        const auto inside = path.lexically_relative(session_);
        if (inside.empty() || *inside.begin() == ".." || !std::filesystem::is_regular_file(path))
            throw std::runtime_error("invalid session file path");
        return path;
    }
    void check() const
    {
        if (stopping_.load() || !rclcpp::ok())
            throw std::runtime_error("replay interrupted");
    }
    void check_locked() const
    {
        check();
        if (!peer_error_.empty())
            throw std::runtime_error(peer_error_);
        if (window_)
            window_->check_timeout(Steady::now());
    }
    template <class Predicate>
    void wait(Predicate predicate, const std::string& what, bool pause_aware = false)
    {
        auto deadline = Steady::now() + timeout_;
        std::unique_lock<std::mutex> lock(mutex_);
        while (!predicate())
        {
            check_locked();
            const auto before = Steady::now();
            const bool suspended = pause_aware && pause_announced_;
            if (!suspended && before >= deadline)
                throw std::runtime_error("timeout: " + what);
            cv_.wait_until(lock,
                suspended ? before + std::chrono::milliseconds(100)
                            : std::min(deadline, before + std::chrono::milliseconds(100)));
            if (suspended)
                deadline += Steady::now() - before;
        }
        check_locked();
    }

    void publish_control_locked()
    {
        std::ostringstream json;
        const auto* target = acknowledged_ ? &targets_[acknowledged_ - 1] : nullptr;
        json << "{\"state\":\"" << control_.name() << "\",\"acknowledged\":" << acknowledged_
                << ",\"total_targets\":" << targets_.size() << ",\"frame_index\":";
        if (target)
            json << target->id;
        else
            json << "null";
        json << ",\"timestamp_ns\":";
        if (target)
            json << target->stamp;
        else
            json << "null";
        json << ",\"position_sec\":"
                << (target ? (target->stamp - frames_.front().stamp) / 1e9 : 0.0)
                << ",\"fast_forward_sec\":" << fast_forward_sec_ << "}";
        std_msgs::msg::String message;
        message.data = json.str();
        control_state_->publish(message);
    }

    // Only the worker announces a safe pause, with no target in flight. The
    // feeder can keep at most its existing bounded compressed prefeed credit.
    void control_boundary()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        control_.boundary();
        for (;;)
        {
            check_locked();
            const bool pause = control_.paused();
            if (!pause_published_ || pause != pause_announced_)
            {
                std_msgs::msg::Bool message;
                message.data = pause;
                paused_->publish(message);
                pause_announced_ = pause;
                pause_published_ = true;
                publish_control_locked();
                cv_.notify_all();
            }
            if (!pause)
                return;
            cv_.wait_for(lock, std::chrono::milliseconds(100));
            control_.boundary();
        }
    }

    void pace(std::int64_t stamp)
    {
        for (;;)
        {
            control_boundary();
            std::unique_lock<std::mutex> lock(mutex_);
            if (reanchor_)
            {
                wall_anchor_ = Steady::now();
                source_anchor_ = stamp;
                reanchor_ = false;
            }
            if (!realtime_ || !control_.paced())
                return;
            const auto due = wall_anchor_ + std::chrono::nanoseconds(stamp - source_anchor_);
            while (Steady::now() < due && control_.paced() && !reanchor_)
            {
                check_locked();
                cv_.wait_until(lock, std::min(due, Steady::now() + std::chrono::milliseconds(100)));
            }
            check_locked();
            if (control_.mode() != replay::Control::Mode::PauseRequested && !control_.paused() &&
                !reanchor_)
                return;
        }
    }
    void receive_ack(const std::string& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        try
        {
            if (message.rfind("error:", 0) == 0)
                throw std::runtime_error(message);
            const auto value = replay::parse_ack(message);
            if (window_)
            {
                window_->acknowledge(value, Steady::now());
                cv_.notify_all();
                return;
            }
            if (!awaiting_ || value.id != awaiting_->id || value.stamp != awaiting_->stamp ||
                ack_received_)
                throw std::runtime_error(
                    "unexpected, duplicate or mismatched VIO acknowledgement: " + message);
            ack_received_ = true;
        }
        catch (const std::exception& error)
        {
            peer_error_ = error.what();
        }
        cv_.notify_all();
    }
    void discover()
    {
        wait([this] { return ready_received_; }, "downstream replay ready");
        if (display_ready_)
            wait([this] { return display_ready_received_; }, "LAS2 warmup");
        const auto deadline = Steady::now() + timeout_;
        auto event = get_graph_event();
        // Frozen-pose reconstruction feeds LAS2 from the packed full image. There
        // is no VIO process or stereo descriptor consumer in that mode. Legacy
        // replay and stereo-only consumers retain both subscription requirements.
        const bool require_stereo = !pose_pub_ || !config_.full_enabled;
        while ((require_stereo &&
                    (left_pub_->get_subscription_count() == 0 ||
                        right_pub_->get_subscription_count() == 0)) ||
            (publish_imu_ && raw_->get_subscription_count() == 0) ||
            ack_->get_publisher_count() == 0 ||
            (completion_sub_ && completion_sub_->get_publisher_count() == 0) ||
            (publish_imu_ && imu_ack_->get_publisher_count() == 0) ||
            (publish_imu_ && paused_->get_subscription_count() == 0) ||
            (pose_pub_ && require_pose_subscriber_ && pose_pub_->get_subscription_count() == 0) ||
            (config_.full_enabled && full_pub_->get_subscription_count() == 0))
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                check_locked();
            }
            if (Steady::now() >= deadline)
                throw std::runtime_error("timeout: replay subscriber discovery");
            wait_for_graph_change(event, std::chrono::milliseconds(100));
            event->check_and_clear();
        }
    }
    void imu_delivered()
    {
        if (!publish_imu_ || acknowledged_ == targets_.size())
            return;
        // An application watermark confirms that the ordered IMU callback actually
        // consumed the batch, without waiting for DDS heartbeat acknowledgements.
        wait([this] { return imu_received_ >= last_imu_timestamp_ns_; },
            "VIO IMU receipt watermark");
    }
    void send(const StreamBufferInfo& stream)
    {
        const auto deadline = Steady::now() + timeout_;
        for (;;)
        {
            check();
            const bool pool = config_.decoder_input_pool.enabled;
            const auto result = pool ? SYS_SendStream(&demux_source_, &stream)
                                        : VDEC_SendStream(vdec_channel_id_, &stream, 100);
            if (result == 0)
                return;
            const bool retry = pool ? result == SYS_ERR_FULL
                                    : result == ERR_VDEC_BUSY || result == ERR_VDEC_TIMEOUT;
            if (!retry)
                throw std::runtime_error("offline decoder send failed: " + std::to_string(result));
            if (Steady::now() >= deadline)
                throw std::runtime_error("timeout: VDEC send");
            if (pool)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(5));
            }
        }
    }
    void feed()
    {
        try
        {
            Input input;
            if (!input.format || !input.packet)
                throw std::bad_alloc();
            input.format->interrupt_callback = {[](void* opaque) -> int
                {
                    auto* self = static_cast<SessionReplayPublisher*>(opaque);
                    return self->stopping_.load() || !rclcpp::ok() ||
                        Steady::now().time_since_epoch().count() > self->io_deadline_.load();
                },
                this};
            io_deadline_ = (Steady::now() + timeout_).time_since_epoch().count();
            if (avformat_open_input(&input.format, video_.c_str(), nullptr, nullptr) < 0 ||
                avformat_find_stream_info(input.format, nullptr) < 0)
                throw std::runtime_error("cannot open/probe source MJPEG container");
            if (input.format->nb_streams != 1 ||
                input.format->streams[0]->codecpar->codec_id != AV_CODEC_ID_MJPEG)
                throw std::runtime_error("expected one original MJPEG stream");
            auto* stream = input.format->streams[0];
            if (stream->codecpar->width != config_.width ||
                stream->codecpar->height != config_.height)
                throw std::runtime_error("MJPEG dimensions disagree with camera snapshot");
            std::size_t index = 0;
            std::int64_t last_pts = -1;
            for (;;)
            {
                wait([this, index]
                    { return index < decoded_count_.load() + static_cast<std::size_t>(prefeed_); },
                    "bounded decoder prefeed credit", true);
                io_deadline_ = (Steady::now() + timeout_).time_since_epoch().count();
                const auto result = av_read_frame(input.format, input.packet);
                if (result == AVERROR_EOF)
                    break;
                if (result < 0)
                    throw std::runtime_error("MJPEG container packet read failed");
                if (index >= frames_.size())
                    throw std::runtime_error("container has more packets than camera index");
                const auto& row = frames_[index];
                const auto nominal_pts = av_rescale_q(row.stamp - frames_.front().stamp,
                    AVRational{1, 1000000000}, stream->time_base);
                const bool adjusted = nominal_pts <= last_pts;
                const auto expected_pts = adjusted ? last_pts + 1 : nominal_pts;
                const auto& packet = *input.packet;
                const bool pts_match = original_mp4_
                    ? packet.pts >= 0 &&
                        av_rescale_q(packet.pts, stream->time_base, AVRational{1, 1000000}) ==
                            static_cast<std::int64_t>(row.pts)
                    : packet.pts == expected_pts && row.adjusted == adjusted;
                if (packet.stream_index != 0 || !pts_match ||
                    (packet.flags & AV_PKT_FLAG_CORRUPT) != 0 ||
                    packet.size != static_cast<int>(row.bytes) || packet.size < 4 ||
                    packet.data[0] != 0xff || packet.data[1] != 0xd8 ||
                    packet.data[packet.size - 2] != 0xff || packet.data[packet.size - 1] != 0xd9)
                    throw std::runtime_error(
                        "container packet/index mismatch at row " + std::to_string(index));
                StreamBufferInfo compressed{};
                compressed.pu8Addr = packet.data;
                compressed.u32Size = row.bytes;
                compressed.bKeyFrame = MPP_TRUE;
                compressed.eCodecType = MPP_STREAM_CODEC_MJPEG;
                compressed.u64PTS = row.pts;
                compressed.u32Width = config_.width;
                compressed.u32Height = config_.height;
                send(compressed); // Original packet bytes; no bitstream filter or
                                  // re-encoding.
                ++index;
                sent_count_ = index;
                last_pts = expected_pts;
                av_packet_unref(input.packet);
            }
            if (index != frames_.size())
                throw std::runtime_error("container ended before camera index");
            StreamBufferInfo eos{};
            eos.bEndOfStream = MPP_TRUE;
            eos.eCodecType = MPP_STREAM_CODEC_MJPEG;
            send(eos);
            input_complete_ = true;
        }
        catch (const std::exception& error)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            peer_error_ = error.what();
        }
        cv_.notify_all();
    }
    std::unique_ptr<Decoded> decode()
    {
        const auto index = decoded_count_.load();
        auto frame = std::make_unique<Decoded>(vdec_channel_id_, index);
        const auto deadline = Steady::now() + timeout_;
        for (;;)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                check_locked();
            }
            const auto result = VDEC_GetFrame(vdec_channel_id_, &frame->frame, 100);
            if (result == ERR_VDEC_EOS)
            {
                if (index != frames_.size())
                    throw std::runtime_error("VDEC dropped frames before EOS");
                return nullptr;
            }
            if (result == ERR_VDEC_OK)
                break;
            if (result != ERR_VDEC_NO_FRAME && result != ERR_VDEC_TIMEOUT &&
                result != ERR_VDEC_RESOLUTION_CHG)
                throw std::runtime_error("VDEC_GetFrame failed: " + std::to_string(result));
            if (Steady::now() >= deadline)
                throw std::runtime_error("timeout: decoded frame/EOS");
        }
        if (index >= frames_.size() || frame->frame.stVFrame.u64PTS != frames_[index].pts)
            throw std::runtime_error(
                "decoded PTS mismatch/drop at camera row " + std::to_string(index));
        const auto& row = frames_[index];
        if (!process_decoded(frame->frame, row.id, row.stamp, false))
            throw std::runtime_error(
                "invalid embedded metadata at camera row " + std::to_string(index));
        imu_delivered();
        ++decoded_count_;
        cv_.notify_all();
        return frame;
    }

    void run_window()
    {
        struct Retained
        {
            std::size_t target_index;
            std::unique_ptr<Decoded> decoded;
        };
        std::deque<Retained> retained;
        auto reap = [&]
        {
            std::lock_guard<std::mutex> lock(mutex_);
            check_locked();
            while (!retained.empty() && retained.front().target_index < window_->prefix())
            {
                const auto index = retained.front().target_index;
                if (index != acknowledged_ || exports_.empty())
                    throw std::runtime_error("frozen replay export retirement mismatch");
                // Revoke this target's full/stereo exports before returning its VDEC
                // output. No later completed target can retire an earlier live token.
                release_oldest_export();
                retained.pop_front();
                ++acknowledged_;
                const auto previous = control_.mode();
                control_.target_complete(targets_[index].stamp, acknowledged_ == targets_.size());
                reanchor_ = reanchor_ || previous != control_.mode();
                publish_control_locked();
            }
        };
        auto wait_progress = [&]
        {
            std::unique_lock<std::mutex> lock(mutex_);
            check_locked();
            if (window_->prefix() > acknowledged_)
                return;
            const auto deadline = window_->next_deadline();
            if (!deadline)
                return;
            cv_.wait_until(
                lock, std::min(*deadline, Steady::now() + std::chrono::milliseconds(100)));
            check_locked();
        };
        bool eos = false;
        while (!eos)
        {
            reap();
            // A requested pause drains the current window before advertising a safe
            // pause. Stepping/fast-forward use one target so they cannot overshoot a
            // source-time boundary. Continuous playing uses the full bounded window.
            if (retained.empty())
                control_boundary();
            bool issue = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                check_locked();
                const auto mode = control_.mode();
                const bool paused = mode == replay::Control::Mode::PauseRequested ||
                    mode == replay::Control::Mode::Paused;
                issue = !paused && window_->can_issue() &&
                    (mode == replay::Control::Mode::Playing || retained.empty());
                // After the last target, decode/validate the complete original tail.
                if (window_->issued() == targets_.size() && retained.empty())
                    issue = true;
            }
            if (!issue)
            {
                wait_progress();
                continue;
            }
            auto decoded = decode();
            if (!decoded)
            {
                eos = true;
                break;
            }
            const auto& row = frames_[decoded->index];
            if (!row.target)
                continue; // No pose/ACK contract exists for non-target full images.
            std::size_t target_index;
            for (;;)
            {
                reap();
                if (retained.empty())
                    control_boundary();
                bool allowed = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    check_locked();
                    const auto mode = control_.mode();
                    allowed = mode == replay::Control::Mode::Playing ||
                        (retained.empty() &&
                            (mode == replay::Control::Mode::Stepping ||
                                mode == replay::Control::Mode::FastForward));
                    if (allowed)
                    {
                        target_index = window_->issue({row.id, row.stamp}, Steady::now());
                        break;
                    }
                }
                wait_progress();
            }
            publish_pose(target_index);
            if (!process_decoded(decoded->frame, row.id, row.stamp, true, false))
                throw std::runtime_error("frozen target image publication failed");
            capture_pair(row);
            retained.push_back({target_index, std::move(decoded)});
        }
        while (!retained.empty())
        {
            reap();
            if (!retained.empty())
                wait_progress();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        check_locked();
        if (window_->issued() != targets_.size() || window_->completed() != targets_.size() ||
            window_->prefix() != targets_.size() || !exports_.empty())
            throw std::runtime_error("incomplete frozen replay window retirement");
    }

    void run() noexcept
    {
        std::string error;
        try
        {
            discover();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                control_.start();
                publish_control_locked();
            }
            feeder_ = std::thread([this] { feed(); });
            if (window_)
            {
                run_window();
            }
            else
            {
                std::deque<std::unique_ptr<Decoded>> pending;
                bool eos = false;
                while (!eos || !pending.empty())
                {
                    control_boundary();
                    if (pending.empty())
                    {
                        auto next = decode();
                        if (!next)
                        {
                            eos = true;
                            continue;
                        }
                        pending.push_back(std::move(next));
                    }
                    const auto& row = frames_[pending.front()->index];
                    pace(row.stamp);
                    if (row.target)
                    {
                        int forward = 0;
                        const auto required_imu = time_alignment_.camera_to_imu(row.stamp);
                        while (publish_imu_ &&
                            last_imu_timestamp_ns_ < static_cast<std::uint64_t>(required_imu))
                        {
                            if (forward++ >= lookahead_ ||
                                pending.size() >= static_cast<std::size_t>(lookahead_ + 1))
                                throw std::runtime_error(
                                    "IMU does not cover target within lookahead bound");
                            auto next = eos ? nullptr : decode();
                            if (!next)
                            {
                                eos = true;
                                restore_imu_tail(static_cast<std::uint64_t>(required_imu));
                                break;
                            }
                            pending.push_back(std::move(next));
                        }
                        // Commands arriving during metadata lookahead take effect before
                        // starting this target. Publication and ACK remain indivisible.
                        pace(row.stamp);
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            check_locked();
                            awaiting_ = replay::Target{row.id, row.stamp};
                            ack_received_ = false;
                        }
                        // Metadata was already committed in capture order, including
                        // lookahead. Reuse only the common V2D/publication stage here.
                        publish_pose(acknowledged_);
                        if (!process_decoded(
                                pending.front()->frame, row.id, row.stamp, true, false))
                            throw std::runtime_error("target image publication failed");
                        capture_pair(row);
                        wait([this] { return ack_received_; }, "downstream target acknowledgement");
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            awaiting_.reset();
                            ++acknowledged_;
                            const auto previous = control_.mode();
                            control_.target_complete(row.stamp, acknowledged_ == targets_.size());
                            reanchor_ = reanchor_ || previous != control_.mode();
                            publish_control_locked();
                        }
                    }
                    if (!row.target && config_.full_enabled && (poses_.empty() || publish_imu_) &&
                        !process_decoded(
                            pending.front()->frame, row.id, row.stamp, true, false, false))
                        throw std::runtime_error("full image publication failed");
                    pending.pop_front();
                }
            }
            wait([this] { return input_complete_.load(); }, "input EOS");
            if (acknowledged_ != targets_.size() || decoded_count_ != frames_.size() ||
                sent_count_ != frames_.size())
                throw std::runtime_error("incomplete replay counts");
            if (completion_gate_)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    check_locked();
                    completion_gate_->start(Steady::now());
                }
                wait([this] { return completion_gate_->ready(Steady::now()); },
                    "downstream output lease release completion");
            }
        }
        catch (const std::exception& failure)
        {
            error = failure.what();
        }
        stopping_ = true;
        cv_.notify_all();
        if (feeder_.joinable())
            feeder_.join();
        finish(error);
    }
    void stop() noexcept
    {
        stopping_ = true;
        cv_.notify_all();
        if (worker_.joinable())
            worker_.join();
        if (feeder_.joinable())
            feeder_.join();
    }
    void restore_imu_tail(std::uint64_t required)
    {
        if (!imu_sidecar_ || imu_sidecar_->verified() == 0)
            throw std::runtime_error("EOS before target IMU coverage; no verified raw IMU tail");
        while (last_imu_timestamp_ns_ < required && imu_sidecar_->available())
        {
            const auto& row = imu_sidecar_->row();
            if (row.stamp <= last_imu_timestamp_ns_)
                throw std::runtime_error("IMU tail overlaps published metadata");
            sensor_msgs::msg::Imu message;
            message.header.frame_id = config_.imu_frame_id;
            message.header.stamp.sec = static_cast<std::int32_t>(row.stamp / 1000000000ULL);
            message.header.stamp.nanosec = static_cast<std::uint32_t>(row.stamp % 1000000000ULL);
            message.orientation_covariance[0] = -1;
            message.angular_velocity.x = row.motion[0];
            message.angular_velocity.y = row.motion[1];
            message.angular_velocity.z = row.motion[2];
            message.linear_acceleration.x = row.motion[3];
            message.linear_acceleration.y = row.motion[4];
            message.linear_acceleration.z = row.motion[5];
            raw_->publish(message);
            last_imu_timestamp_ns_ = row.stamp;
            ++imu_tail_samples_;
            imu_sidecar_->advance();
        }
        if (last_imu_timestamp_ns_ < required)
            throw std::runtime_error("recorded IMU tail does not cover final target");
        imu_delivered();
    }
    void finish(std::string error) noexcept
    {
        if (finished_.exchange(true))
            return;
        try
        {
            if (!output_valid_)
                throw std::runtime_error("summary output_directory unavailable");
            // Write JSON types explicitly: property_tree would quote booleans/counts.
            boost::property_tree::ptree escaped;
            escaped.put("error", error);
            std::ostringstream encoded;
            boost::property_tree::write_json(encoded, escaped, false);
            const auto json = encoded.str();
            const auto value = json.substr(json.find(':') + 1);
            if (capture_valid_)
            {
                if (capture_pairs_.is_open())
                    capture_pairs_.close();
                auto quote = [](const std::string& text)
                {
                    boost::property_tree::ptree field;
                    field.put("value", text);
                    std::ostringstream output;
                    boost::property_tree::write_json(output, field, false);
                    const auto encoded = output.str();
                    return encoded.substr(
                        encoded.find(':') + 1, encoded.rfind('}') - encoded.find(':') - 1);
                };
                std::ofstream capture_summary(capture_ / "capture_summary.json");
                capture_summary
                    << "{\"format\":\"SPACEMIT_ROBOT_OFFLINE_PAIRED_Y_V1\",\"complete\":"
                    << (error.empty() ? "true" : "false")
                    << ",\"written_pairs\":" << captured_pairs_
                    << ",\"accepted_y_bytes\":" << captured_bytes_
                    << ",\"capture_every_n\":" << capture_every_
                    << ",\"cache_start_errors\":" << capture_cache_start_errors_
                    << ",\"cache_end_errors\":" << capture_cache_end_errors_
                    << ",\"write_errors\":" << capture_write_errors_
                    << ",\"source_video\":" << quote(video_.string())
                    << ",\"source_manifest\":" << quote(source_manifest_.string())
                    << ",\"source_video_sha256\":" << quote(source_video_sha256_)
                    << ",\"ownership\":\"synchronous_producer_owned_export\""
                    << ",\"error\":" << value;
                capture_summary.close();
                if (!capture_summary)
                    throw std::runtime_error("cannot write offline capture summary");
            }
            std::ofstream file(output_ / "replay_summary.json", std::ios::out | std::ios::trunc);
            file << "{\"complete\":" << (error.empty() ? "true" : "false")
                    << ",\"camera_frames\":" << decoded_count_.load() << ",\"source_kind\":\""
                    << (original_mp4_ ? "original_mp4" : "recorded_session") << '"'
                    << ",\"publish_imu\":" << (publish_imu_ ? "true" : "false")
                    << ",\"frozen_pose_rows\":" << poses_.size()
                    << ",\"max_inflight\":" << max_inflight_
                    << ",\"submitted_targets\":" << (window_ ? window_->issued() : acknowledged_)
                    << ",\"completed_targets\":" << (window_ ? window_->completed() : acknowledged_)
                    << ",\"max_inflight_observed\":"
                    << (window_ ? window_->high_water() : (acknowledged_ ? 1 : 0))
                    << ",\"completion_required\":" << (completion_gate_ ? "true" : "false")
                    << ",\"completion_received\":"
                    << (completion_gate_ && completion_gate_->completed() ? "true" : "false")
                    << ",\"targets\":" << targets_.size()
                    << ",\"imu_sidecar_verified\":" << (imu_sidecar_ ? imu_sidecar_->verified() : 0)
                    << ",\"imu_tail_samples\":" << imu_tail_samples_
                    << ",\"acknowledged\":" << acknowledged_ << ",\"error\":" << value;
            file.close();
            if (!file)
                throw std::runtime_error("cannot write replay_summary.json");
        }
        catch (const std::exception& failure)
        {
            error += (error.empty() ? "" : "; ") + std::string(failure.what());
        }
        try
        {
            std::lock_guard<std::mutex> lock(mutex_);
            control_.finish(!error.empty());
            publish_control_locked();
        }
        catch (...)
        {
            // Summary/error reporting still needs to run if ROS is already stopped.
        }
        exit_code_ = error.empty() ? 0 : 1;
        try
        {
            std_msgs::msg::String status;
            status.data = error.empty() ? "complete" : "error:" + error;
            status_->publish(status);
            (void)status_->wait_for_all_acked(std::chrono::milliseconds(100));
            RCLCPP_INFO(get_logger(),
                "session replay %s: camera_frames=%zu targets=%zu acknowledged=%zu",
                status.data.c_str(), decoded_count_.load(), targets_.size(), acknowledged_);
        }
        catch (...)
        {
            exit_code_ = 1;
        }
        rclcpp::shutdown();
    }

    std::filesystem::path session_, output_, video_, source_manifest_, capture_;
    std::ofstream capture_pairs_;
    std::string source_video_sha256_, source_timestamp_reference_;
    std::string pose_frame_id_, pose_child_frame_id_;
    std::size_t captured_pairs_{0}, captured_bytes_{0};
    std::size_t capture_cache_start_errors_{0}, capture_cache_end_errors_{0};
    std::size_t capture_write_errors_{0};
    int capture_every_{8};
    std::unique_ptr<replay::EmbeddedImuSidecar> imu_sidecar_;
    std::size_t imu_tail_samples_{0};
    std::vector<replay::Frame> frames_;
    std::vector<replay::Target> targets_;
    std::vector<replay::Pose> poses_;
    std::unique_ptr<replay::AckWindow> window_;
    std::unique_ptr<replay::CompletionGate> completion_gate_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_, fused_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paused_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commands_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ack_, ready_, imu_ack_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr completion_sub_;
    std::thread worker_, feeder_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<replay::Target> awaiting_;
    std::string peer_error_;
    std::uint64_t imu_received_{0};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr display_ready_;
    bool display_ready_received_{false}, realtime_{false};
    bool original_mp4_{false}, publish_imu_{true}, capture_valid_{false};
    bool require_pose_subscriber_{true};
    bool ready_received_{false}, ack_received_{false}, output_valid_{false};
    bool pause_announced_{false}, pause_published_{false}, reanchor_{true};
    replay::Control control_;
    double fast_forward_sec_{5.0};
    std::int64_t fast_forward_ns_{5000000000LL}, source_anchor_{0};
    Steady::time_point wall_anchor_{};
    std::atomic<bool> stopping_{false}, finished_{false}, input_complete_{false};
    std::atomic<int> exit_code_{1};
    std::atomic<std::size_t> sent_count_{0}, decoded_count_{0};
    std::atomic<Steady::duration::rep> io_deadline_{0};
    std::size_t acknowledged_{0};
    int prefeed_{4}, lookahead_{4}, max_inflight_{1};
    camera_sdk::KalibrTimeAlignment time_alignment_;
    Steady::duration timeout_{std::chrono::seconds(30)};
};
} // namespace camera_io

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_io::SessionReplayPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
    // Join before reading the exit code, including an external signal shutdown.
    node->stop_and_join();
    const int result = node->exit_code();
    node.reset();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
