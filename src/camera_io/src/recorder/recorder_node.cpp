/**
 * @file recorder_node.cpp
 * @brief Record camera frames and synchronized metadata with bounded staging.
 */
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

extern "C"
{
#include "sys_api.h"
#include "vb_api.h"
#include <libavcodec/codec_id.h>
#include <libavformat/avformat.h>
#include <libavutil/mathematics.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
}

#include "host_packet_pool.h"
#include "mpp_msgs/msg/mpp_video_frame_desc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

struct CameraConfig
{
    std::string name;
    std::string topic;
    std::string codec;
    int expected_width{0};
    int expected_height{0};
    int nominal_fps{0};
};

struct ImuConfig
{
    std::string name;
    std::string topic;
    bool enabled{true};
    std::size_t queue_depth{64};
};

struct RecorderConfig
{
    std::filesystem::path output_dir;
    std::vector<CameraConfig> cameras;
    std::vector<ImuConfig> imus;
};

RecorderConfig load_config(const std::string& path)
{
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    RecorderConfig config;
    config.output_dir = root.get<std::string>("output_dir");
    for (const auto& entry : root.get_child("cameras"))
    {
        CameraConfig camera;
        camera.name = entry.second.get<std::string>("name");
        camera.topic = entry.second.get<std::string>("topic");
        camera.codec = entry.second.get<std::string>("codec", "mjpeg");
        camera.expected_width =
            entry.second.get<int>("expected_width", entry.second.get<int>("width", 0));
        camera.expected_height =
            entry.second.get<int>("expected_height", entry.second.get<int>("height", 0));
        camera.nominal_fps = entry.second.get<int>("nominal_fps", entry.second.get<int>("fps", 0));
        if (camera.name.empty() || camera.topic.empty() || camera.codec != "mjpeg" ||
            camera.expected_width < 0 || camera.expected_height < 0 || camera.nominal_fps < 0)
        {
            throw std::runtime_error("invalid camera recorder configuration");
        }
        config.cameras.push_back(std::move(camera));
    }
    if (config.cameras.empty())
    {
        throw std::runtime_error("at least one camera is required");
    }
    if (const auto imus = root.get_child_optional("imus"))
    {
        for (const auto& entry : *imus)
        {
            ImuConfig imu;
            imu.name = entry.second.get<std::string>("name");
            imu.topic = entry.second.get<std::string>("topic");
            imu.enabled = entry.second.get<bool>("enabled", true);
            imu.queue_depth = entry.second.get<std::size_t>("queue_depth", 64);
            if (imu.enabled && (imu.name.empty() || imu.topic.empty() || imu.queue_depth == 0))
            {
                throw std::runtime_error("invalid IMU recorder configuration");
            }
            config.imus.push_back(std::move(imu));
        }
    }
    return config;
}

std::string session_name()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream stream;
    stream << "recording_" << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
        static_cast<std::uint64_t>(stamp.nanosec);
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
        if (VB_GetVirAddr(handle_, &address_) != 0 || address_ == nullptr ||
            VB_GetDmaBufFd(handle_, &fd_) != 0 || fd_ < 0)
        {
            (void)VB_ReleaseBuffer(handle_);
            handle_ = 0;
            throw std::runtime_error("failed to resolve imported VB buffer");
        }
        dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
        if (::ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) != 0)
        {
            const std::string error = std::strerror(errno);
            (void)VB_ReleaseBuffer(handle_);
            handle_ = 0;
            throw std::runtime_error("DMA-BUF read sync failed: " + error);
        }
        synced_ = true;
    }

    ~ImportedBuffer()
    {
        if (synced_)
        {
            dma_buf_sync sync{DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ};
            (void)::ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync);
        }
        if (handle_ != 0)
        {
            (void)VB_ReleaseBuffer(handle_);
        }
    }

    ImportedBuffer(const ImportedBuffer&) = delete;
    ImportedBuffer& operator=(const ImportedBuffer&) = delete;

    std::uint8_t* data() const { return static_cast<std::uint8_t*>(address_); }

private:
    UL handle_{0};
    void* address_{nullptr};
    S32 fd_{-1};
    bool synced_{false};
};

class MjpegWriter
{
public:
    MjpegWriter(CameraConfig config, const std::filesystem::path& directory)
        : config_(std::move(config)), video_path_(directory / (config_.name + ".mkv"))
    {
        const auto timestamp_path = directory / (config_.name + "_timestamps.txt");
        timestamps_.open(timestamp_path);
        if (!timestamps_)
        {
            throw std::runtime_error("failed to open " + timestamp_path.string());
        }
    }

    ~MjpegWriter() { cleanup(); }

    MjpegWriter(const MjpegWriter&) = delete;
    MjpegWriter& operator=(const MjpegWriter&) = delete;

    bool write(
        const mpp_msgs::msg::MppVideoFrameDesc& message, const std::uint8_t* retained = nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool initialized_now = format_ == nullptr;
        if (initialized_now)
        {
            try
            {
                initialize(message);
            }
            catch (...)
            {
                cleanup();
                throw;
            }
        }
        else if (message.width != width_ || message.height != height_)
        {
            throw std::runtime_error("compressed stream dimensions changed");
        }
        const std::uint32_t bytes = message.plane_size_valid[0];
        if (bytes < 4 || bytes > message.total_size)
        {
            throw std::runtime_error("invalid compressed frame size");
        }
        std::unique_ptr<ImportedBuffer> imported;
        if (retained == nullptr)
        {
            imported = std::make_unique<ImportedBuffer>(message.token);
            retained = imported->data();
        }
        const std::uint8_t* data = retained;
        if (data[0] != 0xff || data[1] != 0xd8 || data[bytes - 2] != 0xff ||
            data[bytes - 1] != 0xd9)
        {
            throw std::runtime_error("invalid MJPEG SOI/EOI markers");
        }

        const std::uint64_t timestamp = stamp_ns(message.header.stamp);
        if (!first_timestamp_ns_)
        {
            first_timestamp_ns_ = timestamp;
        }
        const std::int64_t relative_ns =
            static_cast<std::int64_t>(timestamp) - static_cast<std::int64_t>(*first_timestamp_ns_);
        std::int64_t pts = av_rescale_q(relative_ns, AVRational{1, 1000000000}, stream_->time_base);
        std::string status = "ok";
        if (pts <= last_pts_)
        {
            pts = last_pts_ + 1;
            status = "pts_adjusted";
        }

        AVPacket packet{};
        packet.data = const_cast<std::uint8_t*>(data);
        packet.size = static_cast<int>(bytes);
        packet.stream_index = stream_->index;
        packet.pts = pts;
        packet.dts = pts;
        packet.duration = 0;
        if (last_timestamp_ns_ && timestamp > *last_timestamp_ns_)
        {
            packet.duration =
                av_rescale_q(static_cast<std::int64_t>(timestamp - *last_timestamp_ns_),
                    AVRational{1, 1000000000}, stream_->time_base);
        }
        packet.flags = AV_PKT_FLAG_KEY;
        const int result = av_write_frame(format_, &packet);
        if (result < 0)
        {
            throw std::runtime_error("Matroska packet write failed");
        }
        last_pts_ = pts;
        last_timestamp_ns_ = timestamp;
        ++frame_count_;
        timestamps_ << message.frame_index << ' ' << timestamp << ' ' << message.pts << ' ' << bytes
                    << ' ' << status << '\n';
        const std::uint64_t flush_interval = config_.nominal_fps > 0 ? config_.nominal_fps : 20;
        if ((frame_count_ % flush_interval) == 0)
        {
            timestamps_.flush();
        }
        return initialized_now;
    }

    const CameraConfig& config() const { return config_; }

    std::pair<std::uint32_t, std::uint32_t> dimensions() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return {width_, height_};
    }

private:
    void cleanup() noexcept
    {
        if (format_ == nullptr)
        {
            return;
        }
        if (header_written_)
        {
            (void)av_write_trailer(format_);
        }
        if ((format_->oformat->flags & AVFMT_NOFILE) == 0 && format_->pb != nullptr)
        {
            avio_closep(&format_->pb);
        }
        avformat_free_context(format_);
        format_ = nullptr;
        stream_ = nullptr;
        header_written_ = false;
    }

    void initialize(const mpp_msgs::msg::MppVideoFrameDesc& message)
    {
        if (message.width == 0 || message.height == 0)
        {
            throw std::runtime_error("compressed descriptor has zero dimensions");
        }
        if ((config_.expected_width > 0 &&
                message.width != static_cast<std::uint32_t>(config_.expected_width)) ||
            (config_.expected_height > 0 &&
                message.height != static_cast<std::uint32_t>(config_.expected_height)))
        {
            throw std::runtime_error("compressed stream dimensions do not match configuration");
        }
        width_ = message.width;
        height_ = message.height;
        timestamps_ << "# codec=mjpeg width=" << width_ << " height=" << height_
                    << " timing=descriptor_timestamp\n";
        timestamps_ << "frame_index timestamp_ns source_pts_us packet_size status\n";

        int result =
            avformat_alloc_output_context2(&format_, nullptr, "matroska", video_path_.c_str());
        if (result < 0 || format_ == nullptr)
        {
            throw std::runtime_error("failed to create Matroska writer");
        }
        stream_ = avformat_new_stream(format_, nullptr);
        if (stream_ == nullptr)
        {
            throw std::runtime_error("failed to create Matroska stream");
        }
        stream_->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        stream_->codecpar->codec_id = AV_CODEC_ID_MJPEG;
        stream_->codecpar->width = width_;
        stream_->codecpar->height = height_;
        stream_->time_base = AVRational{1, 1000000};
        if (config_.nominal_fps > 0)
        {
            stream_->avg_frame_rate = AVRational{config_.nominal_fps, 1};
        }
        if ((format_->oformat->flags & AVFMT_NOFILE) == 0)
        {
            result = avio_open(&format_->pb, video_path_.c_str(), AVIO_FLAG_WRITE);
            if (result < 0)
            {
                throw std::runtime_error("failed to open " + video_path_.string());
            }
        }
        result = avformat_write_header(format_, nullptr);
        if (result < 0)
        {
            throw std::runtime_error("failed to write Matroska header");
        }
        header_written_ = true;
    }

    CameraConfig config_;
    std::filesystem::path video_path_;
    AVFormatContext* format_{nullptr};
    AVStream* stream_{nullptr};
    bool header_written_{false};
    std::ofstream timestamps_;
    mutable std::mutex mutex_;
    std::optional<std::uint64_t> first_timestamp_ns_;
    std::optional<std::uint64_t> last_timestamp_ns_;
    std::int64_t last_pts_{-1};
    std::uint64_t frame_count_{0};
    std::uint32_t width_{0};
    std::uint32_t height_{0};
};

class CameraWriteWorker
{
public:
    CameraWriteWorker(std::shared_ptr<MjpegWriter> writer, std::size_t capacity,
        std::size_t buffer_bytes, std::function<void()> initialized, rclcpp::Logger logger,
        const std::filesystem::path& directory)
        : writer_(std::move(writer)), capacity_(capacity), buffer_bytes_(buffer_bytes),
            pool_(capacity + 2, buffer_bytes), initialized_(std::move(initialized)), logger_(logger),
            summary_path_(directory / (writer_->config().name + "_writer_summary.json"))
    {
        thread_ = std::thread(&CameraWriteWorker::run, this);
    }

    ~CameraWriteWorker() { stop(); }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        if (thread_.joinable())
            thread_.join();
    }

    void enqueue(const mpp_msgs::msg::MppVideoFrameDesc::SharedPtr& message)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++received_;
            if (last_index_)
            {
                if (message->frame_index <= *last_index_)
                    ++nonmonotonic_;
                else
                    source_gaps_ += message->frame_index - *last_index_ - 1;
            }
            last_index_ = message->frame_index;
            if (queue_.size() >= capacity_)
            {
                ++dropped_;
                throw std::runtime_error("video write queue full; recording incomplete");
            }
        }
        const auto start = std::chrono::steady_clock::now();
        std::unique_ptr<camera_io::recording::HostPacketPool::Lease> staged;
        try
        {
            const auto bytes = message->plane_size_valid[0];
            if (bytes < 4 || bytes > message->total_size || bytes > buffer_bytes_)
                throw std::runtime_error("MJPEG packet exceeds configured staging "
                                            "buffer or descriptor size");
            staged = pool_.acquire(bytes);
            ImportedBuffer source(message->token);
            std::memcpy(staged->data(), source.data(), bytes);
            // Source import is released here, before queueing or any file I/O.
        }
        catch (...)
        {
            ++import_errors_;
            throw;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back({message, std::move(staged)});
            max_pending_ = std::max(max_pending_, queue_.size());
            max_stage_ms_ = std::max(max_stage_ms_,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                    .count());
        }
        ready_.notify_one();
    }

private:
    struct Pending
    {
        mpp_msgs::msg::MppVideoFrameDesc::SharedPtr message;
        std::unique_ptr<camera_io::recording::HostPacketPool::Lease> staged;
    };
    void run()
    {
        for (;;)
        {
            Pending frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (queue_.empty() && stopping_)
                    break;
                frame = std::move(queue_.front());
                queue_.pop_front();
            }
            const auto start = std::chrono::steady_clock::now();
            try
            {
                const bool initialized = writer_->write(*frame.message, frame.staged->data());
                frame.staged.reset();
                ++written_;
                if (initialized)
                    initialized_();
            }
            catch (const std::exception& error)
            {
                ++write_errors_;
                RCLCPP_ERROR(
                    logger_, "%s video writer: %s", writer_->config().name.c_str(), error.what());
            }
            max_write_ms_ = std::max(max_write_ms_,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                    .count());
        }
        std::ofstream summary(summary_path_);
        summary << "{\"staging_memory\":\"host\",\"staging_capacity_bytes\":"
                << (capacity_ + 2) * buffer_bytes_ << ",\"received\":" << received_
                << ",\"source_gaps\":" << source_gaps_
                << ",\"source_nonmonotonic\":" << nonmonotonic_ << ",\"written\":" << written_
                << ",\"queue_dropped\":" << dropped_.load()
                << ",\"import_errors\":" << import_errors_.load()
                << ",\"write_errors\":" << write_errors_ << ",\"max_pending\":" << max_pending_
                << ",\"max_write_ms\":" << max_write_ms_ << ",\"max_stage_ms\":" << max_stage_ms_
                << "}\n";
    }
    std::shared_ptr<MjpegWriter> writer_;
    std::size_t capacity_;
    std::size_t buffer_bytes_;
    camera_io::recording::HostPacketPool pool_;
    std::function<void()> initialized_;
    rclcpp::Logger logger_;
    std::filesystem::path summary_path_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Pending> queue_;
    bool stopping_{false};
    std::size_t max_pending_{0};
    std::optional<std::uint64_t> last_index_;
    std::uint64_t received_{0}, source_gaps_{0}, nonmonotonic_{0};
    std::atomic<std::uint64_t> dropped_{0}, import_errors_{0};
    std::uint64_t written_{0}, write_errors_{0};
    double max_write_ms_{0}, max_stage_ms_{0};
    std::thread thread_;
};

class ImuWriter
{
public:
    ImuWriter(ImuConfig config, const std::filesystem::path& directory)
        : config_(std::move(config)), output_(directory / (config_.name + "_imu.txt"))
    {
        if (!output_)
        {
            throw std::runtime_error("failed to open IMU output");
        }
        output_ << "timestamp_ns qx qy qz qw gx gy gz ax ay az\n";
        output_ << std::setprecision(std::numeric_limits<double>::max_digits10);
    }

    void write(const sensor_msgs::msg::Imu& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        output_ << stamp_ns(message.header.stamp) << ' ' << message.orientation.x << ' '
                << message.orientation.y << ' ' << message.orientation.z << ' '
                << message.orientation.w << ' ' << message.angular_velocity.x << ' '
                << message.angular_velocity.y << ' ' << message.angular_velocity.z << ' '
                << message.linear_acceleration.x << ' ' << message.linear_acceleration.y << ' '
                << message.linear_acceleration.z << '\n';
    }

    const ImuConfig& config() const { return config_; }

private:
    ImuConfig config_;
    std::ofstream output_;
    std::mutex mutex_;
};

class RobotRecorder final : public rclcpp::Node
{
public:
    RobotRecorder() : Node("recorder")
    {
        const auto config_file = declare_parameter<std::string>("config_file", "");
        if (config_file.empty())
        {
            throw std::runtime_error("config_file parameter is required");
        }
        config_ = load_config(config_file);
        const auto session_directory = declare_parameter<std::string>("session_directory", "");
        session_directory_ = session_directory.empty() ? config_.output_dir / session_name()
                                                        : std::filesystem::path(session_directory);
        if (std::filesystem::exists(session_directory_ / "manifest.json"))
        {
            throw std::runtime_error("recording session already contains a manifest");
        }
        std::filesystem::create_directories(session_directory_);

        if (SYS_Init() != 0)
        {
            throw std::runtime_error("SYS_Init failed");
        }
        sys_initialized_ = true;
        if (VB_Init() != 0)
        {
            throw std::runtime_error("VB_Init failed");
        }
        vb_initialized_ = true;

        const int video_queue_depth = declare_parameter<int>("camera_write_queue_depth", 0);
        const int video_buffer_bytes = declare_parameter<int>("camera_write_buffer_bytes", 2097152);
        if (video_buffer_bytes < 4 || video_buffer_bytes > 67108864)
            throw std::runtime_error("invalid camera_write_buffer_bytes");
        if (video_queue_depth < 0 || video_queue_depth > 128)
            throw std::runtime_error("invalid camera_write_queue_depth");
        write_manifest();
        for (const auto& camera : config_.cameras)
        {
            auto writer = std::make_shared<MjpegWriter>(camera, session_directory_);
            std::shared_ptr<CameraWriteWorker> worker;
            if (video_queue_depth > 0)
            {
                worker = std::make_shared<CameraWriteWorker>(
                    writer, video_queue_depth, video_buffer_bytes, [this] { write_manifest(); },
                    get_logger(), session_directory_);
                camera_workers_.push_back(worker);
            }
            auto callback_group =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options;
            options.callback_group = callback_group;
            camera_subscriptions_.push_back(create_subscription<mpp_msgs::msg::MppVideoFrameDesc>(
                camera.topic, rclcpp::SensorDataQoS(),
                [this, writer, worker](const mpp_msgs::msg::MppVideoFrameDesc::SharedPtr message)
                {
                    try
                    {
                        if (worker)
                        {
                            worker->enqueue(message);
                        }
                        else if (writer->write(*message))
                        {
                            write_manifest();
                        }
                    }
                    catch (const std::exception& error)
                    {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s recorder: %s",
                            writer->config().name.c_str(), error.what());
                    }
                },
                options));
            callback_groups_.push_back(std::move(callback_group));
            camera_writers_.push_back(std::move(writer));
        }
        for (const auto& imu : config_.imus)
        {
            if (!imu.enabled)
            {
                continue;
            }
            auto writer = std::make_shared<ImuWriter>(imu, session_directory_);
            auto callback_group =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options;
            options.callback_group = callback_group;
            auto qos = rclcpp::SensorDataQoS();
            qos.keep_last(imu.queue_depth);
            imu_subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(
                imu.topic, qos, [writer](const sensor_msgs::msg::Imu::SharedPtr message)
                { writer->write(*message); }, options));
            callback_groups_.push_back(std::move(callback_group));
            imu_writers_.push_back(std::move(writer));
        }
        RCLCPP_INFO(get_logger(), "recording to %s", session_directory_.c_str());
    }

    ~RobotRecorder() override
    {
        camera_subscriptions_.clear();
        imu_subscriptions_.clear();
        for (auto& worker : camera_workers_)
            worker->stop();
        camera_workers_.clear();
        camera_writers_.clear();
        imu_writers_.clear();
        if (vb_initialized_)
        {
            (void)VB_Exit();
        }
        if (sys_initialized_)
        {
            (void)SYS_Exit();
        }
    }

private:
    void write_manifest()
    {
        std::lock_guard<std::mutex> lock(manifest_mutex_);
        boost::property_tree::ptree root;
        root.put("format_version", 1);
        root.put("session", session_directory_.filename().string());
        boost::property_tree::ptree cameras;
        for (const auto& camera : config_.cameras)
        {
            boost::property_tree::ptree entry;
            entry.put("name", camera.name);
            entry.put("topic", camera.topic);
            entry.put("codec", camera.codec);
            entry.put("container", "matroska");
            entry.put("expected_width", camera.expected_width);
            entry.put("expected_height", camera.expected_height);
            entry.put("nominal_fps", camera.nominal_fps);
            for (const auto& writer : camera_writers_)
            {
                if (writer->config().name == camera.name)
                {
                    const auto dimensions = writer->dimensions();
                    entry.put("actual_width", dimensions.first);
                    entry.put("actual_height", dimensions.second);
                    break;
                }
            }
            entry.put("video", camera.name + ".mkv");
            entry.put("timestamps", camera.name + "_timestamps.txt");
            cameras.push_back(std::make_pair("", entry));
        }
        root.add_child("cameras", cameras);
        boost::property_tree::write_json((session_directory_ / "manifest.json").string(), root);
    }

    RecorderConfig config_;
    std::filesystem::path session_directory_;
    bool sys_initialized_{false};
    bool vb_initialized_{false};
    std::vector<std::shared_ptr<CameraWriteWorker>> camera_workers_;
    std::vector<std::shared_ptr<MjpegWriter>> camera_writers_;
    std::vector<std::shared_ptr<ImuWriter>> imu_writers_;
    std::vector<rclcpp::Subscription<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr>
        camera_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
    std::mutex manifest_mutex_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<RobotRecorder>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "recorder: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
