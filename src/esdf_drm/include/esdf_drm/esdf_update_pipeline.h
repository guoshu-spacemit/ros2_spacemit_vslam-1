#ifndef ESDF_UPDATE_PIPELINE_H
#define ESDF_UPDATE_PIPELINE_H
#include "esdf_drm/map_updates.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace esdf_drm
{

struct EsdfUpdatePipelineConfig
{
    esdf_drm::mapping::MapOptions tsdf;
    esdf_drm::mapping::DistanceOptions esdf;
    double target_hz = 5.0;
    std::size_t max_pending_blocks = 16384;
    bool follow_slice_height = false;
    float slice_thickness_m = 0.30F;
    float slice_height_offset_m = 0.0F;
    float slice_min_z_m = -0.25F;
    float slice_max_z_m = 0.25F;
    float slice_output_z_m = 0.0F;
};

struct EsdfUpdatePipelineStats
{
    std::uint64_t submitted_batches = 0;
    std::uint64_t submitted_blocks = 0;
    std::uint64_t coalesced_blocks = 0;
    std::uint64_t dropped_blocks = 0;
    std::uint64_t integrated_batches = 0;
    std::size_t pending_blocks = 0;
    std::size_t mirror_blocks = 0;
    std::size_t esdf_blocks = 0;
    std::size_t last_input_blocks = 0;
    std::size_t last_propagated_blocks = 0;
    double last_apply_ms = 0.0;
    double last_integrate_ms = 0.0;
    float slice_center_z_m = 0.0F;
    bool slice_center_ready = false;
};

class EsdfUpdatePipeline
{
public:
    using LogCallback = std::function<void(const std::string&)>;
    using UpdateCallback = std::function<void(std::vector<esdf_drm::MapChange>)>;

    explicit EsdfUpdatePipeline(EsdfUpdatePipelineConfig config, LogCallback error_callback = {},
        LogCallback info_callback = {}, UpdateCallback update_callback = {});
    ~EsdfUpdatePipeline();

    EsdfUpdatePipeline(const EsdfUpdatePipeline&) = delete;
    EsdfUpdatePipeline& operator=(const EsdfUpdatePipeline&) = delete;

    void start();
    void stop();
    void submit(std::vector<esdf_drm::MapChange> updates);
    void setSliceCenter(float center_z_m);
    EsdfUpdatePipelineStats stats() const;

private:
    void update_loop();
    void report_stats() const;

    EsdfUpdatePipelineConfig config_;
    LogCallback error_callback_;
    LogCallback info_callback_;
    UpdateCallback update_callback_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::unordered_map<esdf_drm::mapping::ChunkKey, esdf_drm::MapChange, esdf_drm::ChunkKeyHash>
        pending_updates_;
    EsdfUpdatePipelineStats stats_{};
    float slice_center_z_m_{0.0F};
    bool slice_center_ready_{false};
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    bool started_{false};
};

} // namespace esdf_drm

#endif // ESDF_UPDATE_PIPELINE_H
