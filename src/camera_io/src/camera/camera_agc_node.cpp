/**
 * @file camera_agc_node.cpp
 * @brief Apply configurable camera gain control from image exposure statistics.
 */
#include "agc_meter.h"
#include "gain_controller.h"

#include <spacemit_camera/embedded_metadata.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sys_api.h>
#include <sys/vb_api.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

using Clock = std::chrono::steady_clock;
using Frame = mpp_msgs::msg::MppVideoFrameDesc;

static void checked_ioctl(int fd, unsigned long request, void* value)
{
    int ret;
    do
    {
        ret = ioctl(fd, request, value);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0)
        throw std::runtime_error(std::string("ioctl: ") + std::strerror(errno));
}

class Controls
{
public:
    explicit Controls(const std::string& device)
    {
        fd_ = open(device.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("Cannot open camera control device");
        try
        {
            if (flock(fd_, LOCK_EX | LOCK_NB))
                throw std::runtime_error("Another AGC client owns this camera");
            mode = get(V4L2_CID_EXPOSURE_AUTO);
            exposure = get(V4L2_CID_EXPOSURE_ABSOLUTE);
            gain = get(V4L2_CID_GAIN);
            range.id = V4L2_CID_GAIN;
            checked_ioctl(fd_, VIDIOC_QUERYCTRL, &range);
            if (range.type != V4L2_CTRL_TYPE_INTEGER ||
                (range.flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY)))
                throw std::runtime_error("Camera gain control is disabled");
        }
        catch (...)
        {
            close(fd_);
            throw;
        }
    }
    Controls(const Controls&) = delete;
    Controls& operator=(const Controls&) = delete;
    void validate_exposure(int value)
    {
        v4l2_queryctrl exposure_range{};
        exposure_range.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        checked_ioctl(fd_, VIDIOC_QUERYCTRL, &exposure_range);
        if (exposure_range.type != V4L2_CTRL_TYPE_INTEGER || exposure_range.step <= 0 ||
            (exposure_range.flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY)) ||
            value < exposure_range.minimum || value > exposure_range.maximum ||
            (std::int64_t(value) - exposure_range.minimum) % exposure_range.step)
            throw std::invalid_argument("exposure_raw is outside the camera control range/step");
    }
    ~Controls()
    {
        // Controls only: never S_FMT/S_PARM/STREAMON or STREAMOFF. The camera
        // publisher owns streaming, and may still be running during cleanup.
        if (modified_)
            for (const auto& item : std::array<std::pair<unsigned, int>, 4>{
                        {{V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL},
                            {V4L2_CID_EXPOSURE_ABSOLUTE, exposure}, {V4L2_CID_GAIN, gain},
                            {V4L2_CID_EXPOSURE_AUTO, mode}}})
            {
                try
                {
                    set(item.first, item.second);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "RESTORE FAILED: " << e.what() << '\n';
                }
            }
        close(fd_);
    }
    int get(unsigned id)
    {
        v4l2_control c{};
        c.id = id;
        checked_ioctl(fd_, VIDIOC_G_CTRL, &c);
        return c.value;
    }
    void set(unsigned id, int value)
    {
        modified_ = true;
        v4l2_control c{};
        c.id = id;
        c.value = value;
        checked_ioctl(fd_, VIDIOC_S_CTRL, &c);
        if (get(id) != value)
            throw std::runtime_error("Control readback mismatch");
    }
    int mode = 0, exposure = 0, gain = 0;
    v4l2_queryctrl range{};

private:
    int fd_ = -1;
    bool modified_ = false;
};

class ImportedRead
{
public:
    explicit ImportedRead(std::uint64_t token)
    {
        if (!token || VB_Import(token, &handle_) != 0 || !handle_)
            throw std::runtime_error("VB_Import failed (expired descriptor?)");
        try
        {
            if (VB_GetDmaBufFd(handle_, &fd_) || fd_ < 0 || VB_GetVirAddr(handle_, &address_) ||
                !address_)
                throw std::runtime_error("Cannot map imported DMA buffer");
            struct stat info{};
            if (fstat(fd_, &info) || info.st_size <= 0)
                throw std::runtime_error("Cannot obtain DMA buffer size");
            size_ = static_cast<std::size_t>(info.st_size);
            dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
            checked_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync);
        }
        catch (...)
        {
            VB_ReleaseBuffer(handle_);
            handle_ = 0;
            throw;
        }
    }
    ~ImportedRead()
    {
        dma_buf_sync sync{DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ};
        try
        {
            checked_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync);
        }
        catch (const std::exception& e)
        {
            std::cerr << "DMA READ END: " << e.what() << '\n';
        }
        (void)VB_ReleaseBuffer(handle_);
    }
    const std::uint8_t* data() const { return static_cast<const std::uint8_t*>(address_); }
    std::size_t size() const { return size_; }
    ImportedRead(const ImportedRead&) = delete;
    ImportedRead& operator=(const ImportedRead&) = delete;

private:
    UL handle_ = 0;
    S32 fd_ = -1;
    void* address_ = nullptr;
    std::size_t size_ = 0;
};

class MppContext
{
public:
    MppContext()
    {
        if (SYS_Init())
            throw std::runtime_error("SYS_Init failed");
        if (VB_Init())
        {
            SYS_Exit();
            throw std::runtime_error("VB_Init failed");
        }
    }
    ~MppContext()
    {
        VB_Exit();
        SYS_Exit();
    }
    MppContext(const MppContext&) = delete;
    MppContext& operator=(const MppContext&) = delete;
};

class AgcNode : public rclcpp::Node
{
public:
    AgcNode() : Node("camera_agc")
    {
        validate_only = declare_parameter<bool>("validate_only", false);
        const auto device = declare_parameter<std::string>("device", "");
        const auto topic = declare_parameter<std::string>("topic", "/camera/full/frame_desc");
        const auto log = declare_parameter<std::string>("log_file", "");
        raw_exposure_ = declare_parameter<int>("exposure_raw", -1);
        metadata_width_ = declare_parameter<int>("metadata_width", 0);
        embedded_metadata_ = declare_parameter<bool>("embedded_metadata", false);
        target_ = declare_parameter<double>("target_y", 80.0);
        if (device.empty() || topic.empty() || log.empty() || raw_exposure_ <= 0 ||
            metadata_width_ < 0 || (embedded_metadata_ && metadata_width_ != 160) ||
            !std::isfinite(target_) || target_ < 1 || target_ > 254)
            throw std::invalid_argument(
                "Set device, topic, log_file, positive exposure_raw and target_y in [1,254]; "
                "embedded parser requires native 160px metadata");
        if (validate_only)
        {
            std::cout << "AGC PARAMETER CHECK PASS (no devices opened)\n";
            return;
        }
        if (std::ifstream(log).good())
            throw std::runtime_error("AGC statistics log already exists; use a new session");
        csv_.open(log);
        if (!csv_)
            throw std::runtime_error("Cannot create AGC statistics log");
        csv_ << "source_frame,elapsed_s,gain,exposure_us,metadata_valid,mean_a,mean_b,clip_a,clip_"
                "b,meter_ms,control_ms\n";
        csv_ << std::setprecision(12);
        mpp_ = std::make_unique<MppContext>();
        sub_ = create_subscription<Frame>(topic, rclcpp::SensorDataQoS().keep_last(1),
            [this, device](Frame::ConstSharedPtr f) { receive(*f, device); });
        RCLCPP_INFO(get_logger(), "AGC waiting for %s; raw exposure=%d, target=%.1f", topic.c_str(),
            raw_exposure_, target_);
    }
    bool validate_only = false;

private:
    void receive(const Frame& f, const std::string& device)
    {
        MeterResult a, b;
        camera_sdk::EmbeddedMetadata metadata;
        const auto begin = Clock::now();
        try
        {
            if (f.pixel_format != MPP_PIXEL_FORMAT_NV12 || f.plane_num < 2 ||
                (embedded_metadata_ && f.height < 768))
                throw std::runtime_error("Expected native packed NV12 with embedded metadata");
            ImportedRead read(f.token);
            a = measure_eye(
                read.data(), read.size(), f.plane_stride[0], f.width, f.height, metadata_width_, 0);
            b = measure_eye(
                read.data(), read.size(), f.plane_stride[0], f.width, f.height, metadata_width_, 1);
            if (embedded_metadata_)
            {
                camera_sdk::LumaView view{read.data(), f.width, f.height, f.plane_stride[0], 1};
                camera_sdk::decode_embedded_metadata(view, metadata);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "AGC skipped frame: %s", e.what());
            return;
        }
        // Buffer release precedes every UVC ioctl. This independent process
        // cannot stall the publisher/VIO callback with a camera control write.
        const auto measured = Clock::now();
        const double elapsed = std::chrono::duration<double>(measured - start_).count();
        const double meter_ms = std::chrono::duration<double, std::milli>(measured - begin).count();
        if (!controls_)
        {
            controls_ = std::make_unique<Controls>(device);
            controls_->validate_exposure(raw_exposure_);
            gain_ = controls_->gain;
            controller_ = std::make_unique<GainController>(
                controls_->range.minimum, controls_->range.maximum, controls_->range.step, gain_);
            std::cout << "ORIGINAL auto=" << controls_->mode << " exposure=" << controls_->exposure
                        << " gain=" << controls_->gain << std::endl;
            controls_->set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
            controls_->set(V4L2_CID_EXPOSURE_ABSOLUTE, raw_exposure_);
            RCLCPP_INFO(get_logger(), "AGC active: fixed raw exposure=%d, initial gain=%d",
                raw_exposure_, gain_);
        }
        ++received_;
        const int next = controller_->update(
            (a.mean + b.mean) * 0.5, std::max(a.clipped, b.clipped), target_, elapsed, received_);
        double control_ms = 0;
        if (next != gain_)
        {
            const auto before = Clock::now();
            controls_->set(V4L2_CID_GAIN, next);
            gain_ = next;
            control_ms = std::chrono::duration<double, std::milli>(Clock::now() - before).count();
        }
        const auto exposure_us =
            std::uint32_t(metadata.exposure_end_raw_us - metadata.exposure_start_raw_us);
        if (received_ % 6 == 0 || control_ms > 0)
            csv_ << f.frame_index << ',' << elapsed << ',' << gain_ << ',' << exposure_us << ','
                    << metadata.valid << ',' << a.mean << ',' << b.mean << ',' << a.clipped << ','
                    << b.clipped << ',' << meter_ms << ',' << control_ms << '\n';
        if (elapsed - last_print_ >= 1)
        {
            RCLCPP_INFO(get_logger(),
                "AGC frame=%u gain=%d metadata_exposure_us=%u valid=%d Y=%.1f/%.1f "
                "meter=%.3fms control=%.3fms",
                f.frame_index, gain_, exposure_us, metadata.valid, a.mean, b.mean, meter_ms,
                control_ms);
            csv_.flush();
            if (!csv_)
                throw std::runtime_error("AGC statistics write failed");
            last_print_ = elapsed;
        }
    }
    int raw_exposure_ = -1, metadata_width_ = 0, gain_ = 0;
    std::uint64_t received_ = 0;
    bool embedded_metadata_ = false;
    double target_ = 80, last_print_ = -1;
    Clock::time_point start_ = Clock::now();
    std::ofstream csv_;
    std::unique_ptr<MppContext> mpp_;
    std::unique_ptr<Controls> controls_;
    std::unique_ptr<GainController> controller_;
    rclcpp::Subscription<Frame>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    int result = 0;
    try
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<AgcNode>();
        if (!node->validate_only)
            rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        std::cerr << "AGC FAILED: " << e.what() << '\n';
        result = 1;
    }
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
