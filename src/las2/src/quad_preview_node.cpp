/**
 * @file quad_preview_node.cpp
 * @brief Publish bounded JPEG previews of four-camera LAS2 descriptor outputs.
 */
#include <sys/sys_api.h>
#include <sys/vb_api.h>
// MPP macros collide with OpenCV utility templates.
#undef MIN
#undef MAX
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <linux/dma-buf.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <common/msg/dma_depth_frame_desc.hpp>

namespace
{
// Each import has one release; no VB ownership extends into JPEG encoding or
// HTTP.
struct ReadBuffer
{
    UL handle = 0;
    S32 fd = -1;
    void* data = nullptr;
    bool synced = false;
    explicit ReadBuffer(U64 token)
    {
        if (VB_Import(token, &handle))
            throw std::runtime_error("VB import failed");
        dma_buf_sync sync{};
        sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        if (VB_GetDmaBufFd(handle, &fd) || VB_GetVirAddr(handle, &data) ||
            ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync))
        {
            VB_ReleaseBuffer(handle);
            throw std::runtime_error("VB map/cache sync failed");
        }
        synced = true;
    }
    ~ReadBuffer()
    {
        if (synced)
        {
            dma_buf_sync sync{};
            sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
            ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
        }
        VB_ReleaseBuffer(handle);
    }
    ReadBuffer(const ReadBuffer&) = delete;
    ReadBuffer& operator=(const ReadBuffer&) = delete;
};

class Preview : public rclcpp::Node
{
public:
    Preview() : Node("quad_preview")
    {
        directory_ = declare_parameter<std::string>("directory", "/dev/shm/quad_preview");
        const auto fps = declare_parameter<double>("fps", 10.0);
        max_depth_ = declare_parameter<double>("max_depth", 10.0);
        if (!std::isfinite(fps) || fps <= 0 || fps > 30 || !std::isfinite(max_depth_) ||
            max_depth_ <= 0)
            throw std::runtime_error("invalid preview FPS/depth range");
        interval_ = std::chrono::duration<double>(1.0 / fps);
        std::filesystem::create_directories(directory_);
        cv::setNumThreads(1);
        const std::array<std::string, 4> names{"front", "right", "rear", "left"};
        for (size_t i = 0; i < names.size(); ++i)
        {
            subscriptions_[i] = create_subscription<Message>("/las2/" + names[i] + "/depth_desc",
                rclcpp::SensorDataQoS().keep_last(1),
                [this, i, name = names[i]](Message::ConstSharedPtr message)
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (now < last_[i])
                        return;
                    const auto step =
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval_);
                    // Advance the deadline, not arrival time: input jitter must not halve FPS.
                    last_[i] = std::max(last_[i] + step, now);
                    try
                    {
                        render(*message, name);
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
                    }
                });
        }
    }

private:
    using Message = common::msg::DmaDepthFrameDesc;
    void render(const Message& d, const std::string& name)
    {
        if (d.version != 2 || d.encoding != 1 || d.rgb_encoding != 1 || !d.width || !d.height ||
            d.width > 8192 || d.height > 8192 || d.rgb_width != d.width ||
            d.rgb_height != d.height || d.stride_bytes < uint64_t(d.width) * 4 ||
            d.rgb_stride_bytes < uint64_t(d.width) * 3 ||
            d.total_size < uint64_t(d.stride_bytes) * d.height ||
            d.rgb_total_size < uint64_t(d.rgb_stride_bytes) * d.height)
            throw std::runtime_error("invalid preview descriptor");
        cv::Mat depth, rgb;
        {
            ReadBuffer depth_buffer(d.token), rgb_buffer(d.rgb_token);
            depth = cv::Mat(d.height, d.width, CV_32FC1, depth_buffer.data, d.stride_bytes).clone();
            rgb = cv::Mat(d.height, d.width, CV_8UC3, rgb_buffer.data, d.rgb_stride_bytes).clone();
        }
        cv::Mat gray(depth.size(), CV_8UC1), invalid(depth.size(), CV_8UC1);
        for (int y = 0; y < depth.rows; ++y)
            for (int x = 0; x < depth.cols; ++x)
            {
                float z = depth.at<float>(y, x);
                bool valid = std::isfinite(z) && z > 0;
                invalid.at<uint8_t>(y, x) = valid ? 0 : 255;
                gray.at<uint8_t>(y, x) = valid
                    ? cv::saturate_cast<uint8_t>(255 * (1 - std::min(double(z) / max_depth_, 1.0)))
                    : 0;
            }
        cv::Mat color, combined;
        cv::applyColorMap(gray, color, cv::COLORMAP_JET);
        color.setTo(cv::Scalar(0, 0, 0), invalid);
        cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
        cv::hconcat(rgb, color, combined);
        const auto path = directory_ + "/" + name;
        if (!cv::imwrite(path + ".tmp.jpg", combined, {cv::IMWRITE_JPEG_QUALITY, 75}))
            throw std::runtime_error("preview JPEG write failed");
        std::filesystem::rename(path + ".tmp.jpg", path + ".jpg");
    }
    std::string directory_;
    double max_depth_;
    std::chrono::duration<double> interval_;
    std::array<std::chrono::steady_clock::time_point, 4> last_{};
    std::array<rclcpp::Subscription<Message>::SharedPtr, 4> subscriptions_;
};
} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (SYS_Init())
        return 1;
    if (VB_Init())
    {
        SYS_Exit();
        return 1;
    }
    int result = 0;
    try
    {
        rclcpp::spin(std::make_shared<Preview>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("quad_preview"), "%s", e.what());
        result = 1;
    }
    VB_Exit();
    SYS_Exit();
    rclcpp::shutdown();
    return result;
}
