#ifndef GLES_RENDER_PIPELINE_H
#define GLES_RENDER_PIPELINE_H
#include "esdf_drm/performance_stats.h"
#include "esdf_drm/tracking_quality_status.h"
#include "esdf_drm/map_updates.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace esdf_drm
{

struct FeatureOverlayPoint
{
    float x = 0.0F;
    float y = 0.0F;
    float pose_input = 0.0F;
    float track_observation_count = 1.0F;
};

// An exact image/feature pair prepared by the ROS node. The pipeline takes
// ownership of dma_fd and keeps at most one pending pair. source_lease prevents
// the producer recycling the input until its GPU snapshot has completed.
struct FeatureOverlayFrame
{
    int dma_fd = -1;
    int width = 0;
    int height = 0;
    int stride_bytes = 0;
    std::uint32_t source_frame_index = 0;
    std::int64_t timestamp_ns = 0;
    std::uint32_t tracked_feature_count = 0;
    std::uint32_t pose_input_count = 0;
    bool pose_inputs_available = false;
    TrackingQualityStatus quality;
    std::vector<FeatureOverlayPoint> points;
    std::shared_ptr<void> source_lease;
};

struct GlesRenderPipelineConfig
{
    std::shared_ptr<PerformanceStats> performance;
    esdf_drm::mapping::RendererOptions renderer = []
    {
        mapping::RendererOptions options;
        options.width = 1920;
        options.height = 1080;
        return options;
    }();
    bool interpolate_surface = true;
    bool feature_overlay_enabled = false;
    bool preview_overlay_enabled = false;
    std::uint32_t feature_overlay_age_max_frames = 14;
    std::size_t max_block_updates_per_frame = 128;
    double target_fps = 60.0;
    double trajectory_duration_s = 0.0;
    float voxel_size_m = 0.05F;
    float minimum_weight = 1.0F;
    float surface_band_m = 0.05F;
    float slice_min_z_m = -0.25F;
    float slice_max_z_m = 0.25F;
};

struct GlesRenderPipelineStats
{
    std::uint64_t submitted_batches = 0;
    std::uint64_t submitted_blocks = 0;
    std::uint64_t coalesced_blocks = 0;
    std::uint64_t dropped_blocks = 0;
    std::uint64_t rendered_frames = 0;
    std::size_t pending_blocks = 0;
    std::size_t trajectory_points = 0;
    double trajectory_span_s = 0.0;
    esdf_drm::mapping::RenderStats renderer;
};

class GlesRenderPipeline
{
public:
    using LogCallback = std::function<void(const std::string&)>;
    using FrameCallback = std::function<void(const esdf_drm::mapping::RenderFrame&)>;

    explicit GlesRenderPipeline(GlesRenderPipelineConfig config, LogCallback error_callback = {},
        LogCallback info_callback = {}, FrameCallback frame_callback = {});
    ~GlesRenderPipeline();

    GlesRenderPipeline(const GlesRenderPipeline&) = delete;
    GlesRenderPipeline& operator=(const GlesRenderPipeline&) = delete;

    void start();
    void stop();
    void SubmitDualView(const std::array<float, 16>& primary_view_projection,
        const std::array<float, 16>& secondary_view_projection,
        const esdf_drm::mapping::Pose& world_from_camera, std::int64_t source_timestamp_ns = 0);
    void SubmitSecondaryUpdates(std::vector<esdf_drm::MapChange> updates);
    void SubmitTsdfUpdates(std::vector<esdf_drm::MapChange> updates);
    // Takes ownership of frame.dma_fd. A newer frame replaces the queued one.
    void SubmitNv12Overlay(esdf_drm::mapping::Nv12Overlay frame);
    // Takes ownership of frame.dma_fd. A newer image/feature pair replaces the
    // queued one; rendering never waits for this diagnostic overlay.
    void SubmitFeatureOverlay(FeatureOverlayFrame frame);
    GlesRenderPipelineStats stats() const;

private:
    void render_loop();
    void report_stats();
    void queue_tsdf_updates_locked(
        std::vector<esdf_drm::MapChange> updates, std::unique_lock<std::mutex>& lock);

    GlesRenderPipelineConfig config_;
    LogCallback error_callback_;
    LogCallback info_callback_;
    FrameCallback frame_callback_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::unordered_map<esdf_drm::mapping::ChunkKey, esdf_drm::MapChange, esdf_drm::ChunkKeyHash>
        pending_tsdf_updates_;
    std::unordered_map<esdf_drm::mapping::ChunkKey, esdf_drm::MapChange, esdf_drm::ChunkKeyHash>
        pending_secondary_updates_;
    esdf_drm::mapping::Nv12Overlay pending_nv12_overlay_{};
    FeatureOverlayFrame pending_feature_overlay_{};
    std::array<float, 16> view_projection_{};
    std::array<float, 16> secondary_view_projection_{};
    esdf_drm::mapping::Pose world_from_camera_{};
    std::int64_t camera_timestamp_ns_ = 0;
    GlesRenderPipelineStats stats_{};
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    bool started_{false};
    bool view_projection_ready_{false};
    bool camera_pose_ready_{false};
    std::uint64_t last_reported_frames_{0};
    std::uint64_t last_reported_updated_blocks_{0};
    std::uint64_t last_reported_uploaded_bytes_{0};
};

} // namespace esdf_drm

#endif // GLES_RENDER_PIPELINE_H
