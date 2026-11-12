// V4L2 capture buffers and frame leases for four-view input.

#include "quad_capture.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace camera_sdk::detail
{
namespace
{
int call_ioctl(int fd, unsigned long request, void* value)
{
    int result;
    do
    {
        result = ioctl(fd, request, value);
    } while (result < 0 && errno == EINTR);
    return result;
}
[[noreturn]] void fail(const char* operation)
{
    throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
struct Mapping
{
    void* address = MAP_FAILED;
    std::size_t size = 0;
};
class V4l2Capture;
struct V4l2Lease
{
    std::shared_ptr<V4l2Capture> capture;
    std::uint32_t index = std::numeric_limits<std::uint32_t>::max();
    ~V4l2Lease();
};

class V4l2Capture : public QuadCapture, public std::enable_shared_from_this<V4l2Capture>
{
public:
    V4l2Capture(QuadCameraConfig config, const std::array<QuadView, 4>& views)
        : QuadCapture(std::move(config), views)
    {
        try
        {
            open();
        }
        catch (...)
        {
            close();
            throw;
        }
    }
    ~V4l2Capture() override { close(); }
    void release(std::uint32_t index) noexcept
    {
        if (index == std::numeric_limits<std::uint32_t>::max())
            return;
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        if (call_ioctl(fd_, VIDIOC_QBUF, &buffer) < 0)
            release_error();
    }

    std::unique_ptr<QuadFrameState> get_frame(int timeout_ms) override
    {
        std::lock_guard<std::mutex> lock(get_mutex_);
        auto frame = std::make_unique<QuadFrameState>();
        auto lease = std::make_shared<V4l2Lease>();
        lease->capture = shared_from_this();
        frame->lease = lease;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        pollfd descriptor{fd_, POLLIN, 0};
        int remaining = timeout_ms;
        v4l2_buffer selected{};
        for (;;)
        {
            for (;;)
            {
                const int ready = poll(&descriptor, 1, remaining);
                if (ready == 0)
                    return {};
                if (ready > 0)
                    break;
                if (errno != EINTR)
                    fail("poll quad camera");
                const auto now = std::chrono::steady_clock::now();
                if (now >= deadline)
                    return {};
                remaining = static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
            }
            if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                throw std::runtime_error("quad camera disconnected or capture failed");
            // Bound draining by the configured ring size, even if new frames arrive.
            for (std::size_t i = 0; i < buffers_.size(); ++i)
            {
                v4l2_buffer buffer{};
                buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buffer.memory = V4L2_MEMORY_MMAP;
                if (call_ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0)
                {
                    if (errno == EAGAIN)
                        break;
                    fail("VIDIOC_DQBUF quad camera");
                }
                if (buffer.index >= buffers_.size())
                    throw std::runtime_error("quad driver returned an invalid buffer index");
                const auto& mapping = buffers_[buffer.index];
                const auto* data = static_cast<const std::uint8_t*>(mapping.address);
                if ((buffer.flags & V4L2_BUF_FLAG_ERROR) || buffer.bytesused < 4 ||
                    buffer.bytesused > mapping.size || data[0] != 0xff || data[1] != 0xd8)
                {
                    ++invalid_;
                    release(buffer.index);
                    continue;
                }
                if (lease->index != std::numeric_limits<std::uint32_t>::max())
                {
                    release(lease->index);
                    ++stale_;
                }
                lease->index = buffer.index;
                selected = buffer;
            }
            if (lease->index != std::numeric_limits<std::uint32_t>::max())
                break;
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
                return {};
            remaining = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        }
        frame->image = {QuadPixelFormat::mjpeg,
            static_cast<const std::uint8_t*>(buffers_[selected.index].address), selected.bytesused,
            -1, config_.width, config_.height, {}};
        frame->views = views_;
        auto& metadata = frame->metadata;
        metadata.frame_index = ++delivered_;
        metadata.host_received_ns = quad_host_now_ns();
        metadata.source_sequence = selected.sequence;
        metadata.source_sequence_valid = true;
        metadata.driver_flags = selected.flags;
        if (selected.timestamp.tv_sec >= 0 && selected.timestamp.tv_usec >= 0 &&
            selected.timestamp.tv_usec < 1000000 &&
            static_cast<std::uint64_t>(selected.timestamp.tv_sec) <=
                (std::numeric_limits<std::uint64_t>::max() - selected.timestamp.tv_usec) / 1000000)
        {
            metadata.source_timestamp_us =
                static_cast<std::uint64_t>(selected.timestamp.tv_sec) * 1000000 +
                selected.timestamp.tv_usec;
            if ((selected.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                metadata.clock = QuadClock::v4l2_monotonic;
        }
        return frame;
    }

private:
    void open()
    {
        fd_ = ::open(config_.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            fail("open quad UVC device");
        v4l2_capability capability{};
        if (call_ioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0)
            fail("VIDIOC_QUERYCAP");
        const auto caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
            ? capability.device_caps
            : capability.capabilities;
        if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING))
            throw std::runtime_error("quad camera requires single-plane streaming V4L2 capture");
        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = config_.width;
        format.fmt.pix.height = config_.height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        format.fmt.pix.field = V4L2_FIELD_ANY;
        if (call_ioctl(fd_, VIDIOC_S_FMT, &format) < 0)
            fail("VIDIOC_S_FMT quad MJPEG");
        if (format.fmt.pix.width != config_.width || format.fmt.pix.height != config_.height ||
            format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
            throw std::runtime_error("quad device did not accept requested MJPEG geometry");
        v4l2_streamparm parameters{};
        parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parameters.parm.capture.timeperframe = {1, config_.fps};
        if (call_ioctl(fd_, VIDIOC_S_PARM, &parameters) < 0)
            fail("VIDIOC_S_PARM quad FPS");
        const auto period = parameters.parm.capture.timeperframe;
        if (period.numerator == 0 ||
            static_cast<std::uint64_t>(period.numerator) * config_.fps != period.denominator)
            throw std::runtime_error("quad device did not accept requested FPS: period=" +
                std::to_string(period.numerator) + "/" + std::to_string(period.denominator));
        v4l2_requestbuffers request{};
        request.count = config_.queue_depth;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (call_ioctl(fd_, VIDIOC_REQBUFS, &request) < 0)
            fail("VIDIOC_REQBUFS");
        if (request.count < 2 || request.count > 32)
            throw std::runtime_error("invalid V4L2 ring size");
        buffers_.resize(request.count);
        for (std::uint32_t i = 0; i < request.count; ++i)
        {
            v4l2_buffer buffer{};
            buffer.type = request.type;
            buffer.memory = request.memory;
            buffer.index = i;
            if (call_ioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0)
                fail("VIDIOC_QUERYBUF");
            if (buffer.length == 0)
                throw std::runtime_error("empty V4L2 mapping");
            buffers_[i].size = buffer.length;
            buffers_[i].address = mmap(
                nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
            if (buffers_[i].address == MAP_FAILED)
                fail("mmap quad camera");
            if (call_ioctl(fd_, VIDIOC_QBUF, &buffer) < 0)
                fail("initial VIDIOC_QBUF");
        }
        auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (call_ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
            fail("VIDIOC_STREAMON");
        streaming_ = true;
    }
    void close() noexcept
    {
        if (streaming_)
        {
            auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            call_ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
        for (const auto& mapping : buffers_)
            if (mapping.address != MAP_FAILED)
                munmap(mapping.address, mapping.size);
        if (fd_ >= 0)
            ::close(fd_);
        fd_ = -1;
        streaming_ = false;
    }
    int fd_ = -1;
    bool streaming_ = false;
    std::vector<Mapping> buffers_;
    std::mutex get_mutex_;
};

V4l2Lease::~V4l2Lease()
{
    if (capture)
        capture->release(index);
}
} // namespace

std::shared_ptr<QuadCapture> make_v4l2_quad_capture(
    QuadCameraConfig config, const std::array<QuadView, 4>& views)
{
    return std::make_shared<V4l2Capture>(std::move(config), views);
}
} // namespace camera_sdk::detail
