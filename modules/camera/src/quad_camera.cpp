// Construct four-view capture and retain frame-buffer leases.

#include "quad_capture.h"

#include <stdexcept>

namespace camera_sdk
{

const char* quad_position_name(QuadPosition position) noexcept
{
    switch (position)
    {
    case QuadPosition::front_left:
        return "front_left";
    case QuadPosition::front_right:
        return "front_right";
    case QuadPosition::rear_right:
        return "rear_right";
    case QuadPosition::rear_left:
        return "rear_left";
    }
    return "unknown";
}

std::array<QuadView, 4> make_quad_views(
    std::uint32_t width, std::uint32_t height, const std::array<QuadPosition, 4>& positions)
{
    if (width == 0 || height == 0 || width % 2 || height % 8)
        throw std::invalid_argument("quad layout needs even width and height divisible by eight");
    unsigned mask = 0;
    std::array<QuadView, 4> views{};
    for (std::size_t i = 0; i < views.size(); ++i)
    {
        const auto position = static_cast<unsigned>(positions[i]);
        if (position >= 4 || (mask & (1U << position)))
            throw std::invalid_argument("quad positions must contain each physical position once");
        mask |= 1U << position;
        views[i] = {static_cast<std::uint32_t>(i), positions[i], 0,
            static_cast<std::uint32_t>(i) * (height / 4), width, height / 4};
    }
    return views;
}

QuadFrame::QuadFrame() noexcept = default;
QuadFrame::~QuadFrame() = default;
QuadFrame::QuadFrame(QuadFrame&&) noexcept = default;
QuadFrame& QuadFrame::operator=(QuadFrame&&) noexcept = default;
QuadFrame::QuadFrame(std::unique_ptr<detail::QuadFrameState> state) noexcept
    : state_(std::move(state))
{
}
QuadFrame::operator bool() const noexcept { return static_cast<bool>(state_); }
const QuadImage& QuadFrame::image() const
{
    if (!state_)
        throw std::logic_error("empty quad frame");
    return state_->image;
}
const std::array<QuadView, 4>& QuadFrame::views() const
{
    if (!state_)
        throw std::logic_error("empty quad frame");
    return state_->views;
}
const QuadFrameMetadata& QuadFrame::metadata() const
{
    if (!state_)
        throw std::logic_error("empty quad frame");
    return state_->metadata;
}

QuadCamera::QuadCamera(QuadCameraConfig config)
{
    [[maybe_unused]] const auto views =
        make_quad_views(config.width, config.height, config.positions);
    if (config.device.empty() || config.fps == 0 || config.queue_depth < 2 ||
        config.queue_depth > 32 || config.mpp_device_id < 0)
        throw std::invalid_argument(
            "quad camera requires device, FPS, 2-32 buffers and a nonnegative MPP ID");
    if (config.backend == QuadCaptureBackend::platform_default)
    {
#ifdef CAMERA_SDK_WITH_MPP
        config.backend = QuadCaptureBackend::mpp_nv12;
#else
        config.backend = QuadCaptureBackend::v4l2_mjpeg;
#endif
    }
    config.decoder_input_pool.validate();
    if (config.decoder_input_pool.enabled && config.backend != QuadCaptureBackend::mpp_nv12)
        throw std::invalid_argument("decoder input pool requires the MPP NV12 backend");
    switch (config.backend)
    {
    case QuadCaptureBackend::v4l2_mjpeg:
#ifdef CAMERA_SDK_WITH_V4L2
        capture_ = detail::make_v4l2_quad_capture(std::move(config), views);
#else
        throw std::runtime_error("camera_sdk was built without CAMERA_SDK_WITH_V4L2");
#endif
        break;
    case QuadCaptureBackend::mpp_nv12:
#ifdef CAMERA_SDK_WITH_MPP
        capture_ = detail::make_mpp_quad_capture(std::move(config), views);
#else
        throw std::runtime_error("camera_sdk was built without CAMERA_SDK_WITH_MPP");
#endif
        break;
    default:
        throw std::invalid_argument("unsupported quad backend");
    }
}
QuadCamera::~QuadCamera() = default;
QuadCamera::QuadCamera(QuadCamera&&) noexcept = default;
QuadCamera& QuadCamera::operator=(QuadCamera&&) noexcept = default;
QuadFrame QuadCamera::get_frame(int timeout_ms)
{
    if (!capture_)
        throw std::logic_error("empty quad camera");
    if (timeout_ms < 0)
        throw std::invalid_argument("negative quad capture timeout");
    return QuadFrame(capture_->get_frame(timeout_ms));
}
const QuadCameraConfig& QuadCamera::config() const
{
    if (!capture_)
        throw std::logic_error("empty quad camera");
    return capture_->config();
}
QuadCaptureStats QuadCamera::stats() const
{
    if (!capture_)
        throw std::logic_error("empty quad camera");
    return capture_->stats();
}

} // namespace camera_sdk
