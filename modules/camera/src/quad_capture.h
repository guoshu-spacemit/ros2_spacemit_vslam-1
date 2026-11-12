// Internal native camera support for quad capture.
#ifndef QUAD_CAPTURE_H
#define QUAD_CAPTURE_H

#include "spacemit_camera/quad_camera.h"
#include <atomic>
#include <chrono>

namespace camera_sdk::detail
{

struct QuadFrameState
{
    QuadImage image;
    std::array<QuadView, 4> views;
    QuadFrameMetadata metadata;
    std::shared_ptr<void> lease;
};

inline std::uint64_t quad_host_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class QuadCapture
{
public:
    QuadCapture(QuadCameraConfig config, const std::array<QuadView, 4>& views)
        : config_(std::move(config)), views_(views)
    {
    }
    virtual ~QuadCapture() = default;
    virtual std::unique_ptr<QuadFrameState> get_frame(int timeout_ms) = 0;
    const QuadCameraConfig& config() const { return config_; }
    QuadCaptureStats stats() const { return {delivered_, stale_, invalid_, release_errors_}; }
    void release_error() noexcept { ++release_errors_; }

protected:
    QuadCameraConfig config_;
    const std::array<QuadView, 4> views_;
    std::atomic<std::uint64_t> delivered_{0}, stale_{0}, invalid_{0}, release_errors_{0};
};

std::shared_ptr<QuadCapture> make_v4l2_quad_capture(
    QuadCameraConfig config, const std::array<QuadView, 4>& views);
std::shared_ptr<QuadCapture> make_mpp_quad_capture(
    QuadCameraConfig config, const std::array<QuadView, 4>& views);

} // namespace camera_sdk::detail

#endif // QUAD_CAPTURE_H
