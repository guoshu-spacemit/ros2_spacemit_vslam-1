#ifndef DRM_RECORDER_H
#define DRM_RECORDER_H
#include <cstdint>
#include <memory>
#include <string>

#include "esdf_drm/map_updates.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace esdf_drm
{

struct DrmRecorderConfig
{
    bool enabled{false};
    bool use_source_timestamps{false};
    bool source_timestamp_origin_zero{false};
    std::string output_pattern;
    int fps{30};
    int bitrate_bps{8000000};
    int gop{30};
    int pool_depth{4};
    int venc_channel{2};
    int mux_channel{2};
};

struct DrmRecorderStats
{
    std::uint64_t captured_frames{0};
    std::uint64_t converted_frames{0};
    std::uint64_t ignored_render_frames{0};
    std::uint64_t submitted_frames{0};
    std::uint64_t encoded_frames{0};
    std::uint64_t dropped_frames{0};
    std::uint64_t encoded_bytes{0};
    double average_v2d_ms{0.0};
    double source_duration_seconds{0.0};
    double output_duration_seconds{0.0};
    std::string current_file;
};

class DrmRecorder
{
public:
    DrmRecorder(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, DrmRecorderConfig config);
    ~DrmRecorder();

    DrmRecorder(const DrmRecorder&) = delete;
    DrmRecorder& operator=(const DrmRecorder&) = delete;

    bool start(int width, int height, std::string* error);
    void submit(const esdf_drm::mapping::RenderFrame& frame, std::int64_t source_timestamp_ns = 0);
    void stop();
    DrmRecorderStats stats() const;
    bool active() const;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace esdf_drm

#endif // DRM_RECORDER_H
