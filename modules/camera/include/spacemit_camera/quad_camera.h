// Four-camera capture buffers, metadata, and frame ownership.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_QUAD_CAMERA_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_QUAD_CAMERA_H_

#include "spacemit_camera/decoder_input_pool_config.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace camera_sdk
{

enum class QuadPosition
{
    front_left,
    front_right,
    rear_right,
    rear_left
};
const char* quad_position_name(QuadPosition position) noexcept;

enum class QuadCaptureBackend
{
    platform_default,
    v4l2_mjpeg,
    mpp_nv12
};
enum class QuadPixelFormat
{
    mjpeg,
    nv12
};
enum class QuadClock
{
    unknown,
    v4l2_monotonic,
    mpp_pts
};

struct QuadCameraConfig
{
    std::string device; // Required: use the device's /dev/v4l/by-id path when possible.
    std::uint32_t width = 1088;
    std::uint32_t height = 5120;
    std::uint32_t fps = 30; // Requested UVC mode; measure delivered FPS separately.
    std::uint32_t queue_depth = 4;
    int mpp_device_id = 1; // Use a different MPP ID from any simultaneous Nori camera.
    QuadCaptureBackend backend = QuadCaptureBackend::platform_default;
    // Top-to-bottom packed order, confirmed for the user's Omni Depth rig.
    std::array<QuadPosition, 4> positions{QuadPosition::front_left, QuadPosition::front_right,
        QuadPosition::rear_right, QuadPosition::rear_left};
    DecoderInputPoolConfig decoder_input_pool;
};

struct QuadView
{
    std::uint32_t slot = 0; // Calibration array slot, NOT the vendor's camera_id field.
    QuadPosition position = QuadPosition::front_left;
    std::uint32_t x = 0, y = 0, width = 0, height = 0;
};

// Validate the four-view vertical layout, including NV12 chroma alignment.
std::array<QuadView, 4> make_quad_views(
    std::uint32_t width, std::uint32_t height, const std::array<QuadPosition, 4>& positions);

struct QuadPlane
{
    std::size_t offset = 0, stride = 0, size = 0;
};

struct QuadImage
{
    QuadPixelFormat format = QuadPixelFormat::mjpeg;
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    int dma_fd = -1; // Borrowed. -1 for host V4L2 compressed JPEG.
    std::uint32_t width = 0, height = 0;
    std::array<QuadPlane, 2> planes{}; // NV12 only; MJPEG has no pixel row stride.
};

struct QuadFrameMetadata
{
    std::uint64_t frame_index = 0; // Delivery index within this camera instance.
    std::uint64_t source_timestamp_us = 0;
    QuadClock clock = QuadClock::unknown;
    std::uint64_t host_received_ns = 0; // steady_clock, not camera exposure time.
    std::uint32_t source_sequence = 0;
    std::uint32_t driver_flags = 0;
    bool source_sequence_valid = false;
    // This image-only entry does not associate USB timing/IMU events yet.
    bool exposure_time_valid = false;
};

struct QuadCaptureStats
{
    std::uint64_t delivered = 0;
    std::uint64_t stale_dropped = 0;
    std::uint64_t invalid_dropped = 0;
    std::uint64_t release_errors = 0;
};

namespace detail
{
struct QuadFrameState;
class QuadCapture;
} // namespace detail

class QuadFrame
{
public:
    QuadFrame() noexcept;
    ~QuadFrame();
    QuadFrame(QuadFrame&&) noexcept;
    QuadFrame& operator=(QuadFrame&&) noexcept;
    QuadFrame(const QuadFrame&) = delete;
    QuadFrame& operator=(const QuadFrame&) = delete;
    explicit operator bool() const noexcept;
    const QuadImage& image() const;
    const std::array<QuadView, 4>& views() const;
    const QuadFrameMetadata& metadata() const;

private:
    friend class QuadCamera;
    explicit QuadFrame(std::unique_ptr<detail::QuadFrameState> state) noexcept;
    std::unique_ptr<detail::QuadFrameState> state_;
};

// Separate from Camera/Nori. No vendor library dependency and no Nori metadata decoding.
// Frames keep the capture buffers/device alive; release them promptly. Borrowed
// pointers/FDs/ROIs must never outlive their QuadFrame. DMA-BUF CPU access requires
// the caller's normal DMA cache synchronization. Never write shared input pixels.
class QuadCamera
{
public:
    explicit QuadCamera(QuadCameraConfig config);
    ~QuadCamera();
    QuadCamera(QuadCamera&&) noexcept;
    QuadCamera& operator=(QuadCamera&&) noexcept;
    QuadCamera(const QuadCamera&) = delete;
    QuadCamera& operator=(const QuadCamera&) = delete;
    QuadFrame get_frame(int timeout_ms = 1000); // Empty on timeout/no valid frame.
    const QuadCameraConfig& config() const;
    QuadCaptureStats stats() const;

private:
    std::shared_ptr<detail::QuadCapture> capture_;
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_QUAD_CAMERA_H_
