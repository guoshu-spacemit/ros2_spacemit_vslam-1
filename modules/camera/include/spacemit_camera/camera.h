// Native camera capture and frame-lease interfaces.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_CAMERA_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_CAMERA_H_

#include "spacemit_camera/decoder_input_pool_config.h"
#include "spacemit_camera/embedded_metadata.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace camera_sdk
{

enum class PixelFormat
{
    yuyv422,
    nv12,
};

enum class CaptureFormat
{
    yuyv422,
    mjpeg,
};

struct CameraConfig
{
    std::string device = "/dev/video9";
    std::uint32_t width = 4000;
    std::uint32_t height = 1200;
    std::uint32_t fps = 30;
    std::uint32_t metadata_width = 160;
    std::uint32_t queue_depth = 12;
    int mpp_device_id = 0;
    CaptureFormat capture_format = CaptureFormat::yuyv422;
    DecoderInputPoolConfig decoder_input_pool;
};

struct ImageView
{
    int dma_fd = -1;
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::size_t stride = 0;
    PixelFormat format = PixelFormat::yuyv422;

    // The metadata region begins at metadata_x. The packed stereo ROI starts
    // at stereo_x and spans stereo_width pixels within the same DMA-BUF.
    std::uint32_t metadata_x = 0;
    std::uint32_t stereo_x = 160;
    std::uint32_t stereo_width = 3840;
};

namespace detail
{
struct FrameState;
struct CameraState;
} // namespace detail

// Owns a capture buffer lease. The SDK releases the buffer with the lease.
// Keep the lease alive while using its borrowed references, pixels or DMA-BUF
// descriptor. Use DMA-BUF CPU synchronization before reading shared pixels.
class Frame
{
public:
    Frame() noexcept;
    ~Frame();
    Frame(Frame&&) noexcept;
    Frame& operator=(Frame&&) noexcept;
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;

    // Reports whether the frame owns a capture buffer.
    explicit operator bool() const noexcept;
    // Borrowed image view. Throws std::logic_error if the frame is empty.
    const ImageView& image() const;
    // Borrowed metadata. Invalid decoding or clock extension leaves valid false.
    // Check valid before using timestamps or IMU samples.
    // Throws std::logic_error if the frame is empty.
    const EmbeddedMetadata& metadata() const;

private:
    friend class Camera;
    explicit Frame(std::unique_ptr<detail::FrameState> state) noexcept;
    std::unique_ptr<detail::FrameState> state_;
};

class Camera
{
public:
    explicit Camera(CameraConfig config = {});
    ~Camera();
    Camera(Camera&&) noexcept;
    Camera& operator=(Camera&&) noexcept;
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // Waits for capture using the timeout in milliseconds. Calls on the same
    // camera serialize. Throws std::logic_error on a moved-from camera and
    // std::runtime_error on a capture error or if the MPP backend is disabled.
    Frame get_frame(int timeout_ms = 1000);
    // Borrowed configuration. Throws std::logic_error on a moved-from camera.
    const CameraConfig& config() const;

private:
    std::shared_ptr<detail::CameraState> state_;
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_CAMERA_H_
