/**
 * @file esdf_update_pipeline.cpp
 * @brief Schedule asynchronous ESDF updates and removal propagation.
 */
#include "esdf_drm/esdf_update_pipeline.h"
#include "esdf_drm/block_removal_queue.h"
#include "esdf_drm/local_map_window.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace esdf_drm
{
namespace
{

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point begin)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

} // namespace

EsdfUpdatePipeline::EsdfUpdatePipeline(EsdfUpdatePipelineConfig config, LogCallback error_callback,
    LogCallback info_callback, UpdateCallback update_callback)
    : config_(std::move(config)), error_callback_(std::move(error_callback)),
        info_callback_(std::move(info_callback)), update_callback_(std::move(update_callback))
{
    if (config_.target_hz <= 0.0 || config_.target_hz > 60.0 || config_.max_pending_blocks == 0 ||
        config_.slice_thickness_m <= 0.0F)
    {
        throw std::invalid_argument("invalid asynchronous ESDF configuration");
    }
}

void EsdfUpdatePipeline::setSliceCenter(float center_z_m)
{
    if (!std::isfinite(center_z_m))
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    slice_center_z_m_ = center_z_m;
    slice_center_ready_ = true;
    stats_.slice_center_z_m = center_z_m;
    stats_.slice_center_ready = true;
}

EsdfUpdatePipeline::~EsdfUpdatePipeline() { stop(); }

void EsdfUpdatePipeline::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_)
    {
        return;
    }
    stopping_.store(false);
    started_ = true;
    thread_ = std::thread(&EsdfUpdatePipeline::update_loop, this);
}

void EsdfUpdatePipeline::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_)
        {
            return;
        }
        stopping_.store(true);
    }
    condition_.notify_all();
    if (thread_.joinable())
    {
        thread_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
    pending_updates_.clear();
    stats_.pending_blocks = 0;
}

void EsdfUpdatePipeline::submit(std::vector<esdf_drm::MapChange> updates)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!started_ || stopping_.load())
    {
        return;
    }
    ++stats_.submitted_batches;
    stats_.submitted_blocks += updates.size();
    for (auto& update : updates)
    {
        if (update.removed)
            wait_for_removal_slot(pending_updates_, update.index, config_.max_pending_blocks, lock,
                condition_, stopping_);
        if (stopping_.load())
            return;
        auto existing = pending_updates_.find(update.index);
        if (existing != pending_updates_.end())
        {
            existing->second = std::move(update);
            ++stats_.coalesced_blocks;
            continue;
        }
        if (pending_updates_.size() >= config_.max_pending_blocks)
        {
            ++stats_.dropped_blocks;
            continue;
        }
        pending_updates_.emplace(update.index, std::move(update));
    }
    stats_.pending_blocks = pending_updates_.size();
    condition_.notify_one();
}

EsdfUpdatePipelineStats EsdfUpdatePipeline::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void EsdfUpdatePipeline::update_loop()
{
    const NotifyStopped notify_stopped{stopping_, condition_, mutex_};
    try
    {
        esdf_drm::mapping::Map mirror(config_.tsdf);
        esdf_drm::mapping::DistanceField integrator(config_.tsdf.resolution_m, config_.esdf);
        const auto update_period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / config_.target_hz));
        auto next_update = Clock::now() + update_period;
        auto next_report = Clock::now() + std::chrono::seconds(2);
        while (!stopping_.load())
        {
            std::vector<esdf_drm::MapChange> updates;
            float slice_center_z_m = 0.0F;
            bool slice_center_ready = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait_until(lock, next_update, [this] { return stopping_.load(); });
                if (stopping_.load())
                {
                    break;
                }
                updates.reserve(pending_updates_.size());
                for (auto& entry : pending_updates_)
                {
                    updates.push_back(std::move(entry.second));
                }
                pending_updates_.clear();
                stats_.pending_blocks = 0;
                slice_center_z_m = slice_center_z_m_;
                slice_center_ready = slice_center_ready_;
            }
            condition_.notify_all();
            if (!updates.empty())
            {
                const auto apply_begin = Clock::now();
                mirror.apply(queued_snapshots(updates));
                std::vector<mapping::ChunkKey> changed;
                changed.reserve(updates.size());
                for (const auto& update : updates)
                    changed.push_back(update.index);
                const double apply_time = elapsed_ms(apply_begin);
                const auto integrate_begin = Clock::now();
                mapping::DistanceStats esdf_stats;
                std::vector<esdf_drm::mapping::ChunkKey> removed;
                for (const auto& update : updates)
                    if (update.removed)
                        removed.push_back(update.index);
                float min_z = config_.slice_min_z_m;
                float max_z = config_.slice_max_z_m;
                if (config_.follow_slice_height)
                {
                    if (!slice_center_ready)
                    {
                        continue;
                    }
                    const float center = slice_center_z_m + config_.slice_height_offset_m;
                    const float half_thickness = config_.slice_thickness_m * 0.5F;
                    min_z = center - half_thickness;
                    max_z = center + half_thickness;
                    // Include deleted columns so native ESDF invalidates their
                    // sites and dependent distances, not just surviving TSDF.
                    mapping::DistanceRequest request;
                    request.slice = true;
                    request.all_input = true;
                    request.minimum_height_m = min_z;
                    request.maximum_height_m = max_z;
                    request.output_height_m = config_.slice_output_z_m;
                    esdf_stats = integrator.update(mirror, removed, request);
                }
                else
                {
                    esdf_stats = integrator.update(mirror, changed);
                    mapping::DistanceRequest request;
                    request.slice = true;
                    request.minimum_height_m = min_z;
                    request.maximum_height_m = max_z;
                    request.output_height_m = config_.slice_output_z_m;
                    (void)integrator.update(mirror, changed, request);
                }
                mapping::DistanceRequest retirement;
                retirement.slice = true;
                retirement.minimum_height_m = min_z;
                retirement.maximum_height_m = max_z;
                retirement.output_height_m = config_.slice_output_z_m;
                integrator.retire(mirror, removed, retirement);
                const double integrate_time = elapsed_ms(integrate_begin);
                if (update_callback_)
                {
                    update_callback_(split_changes(integrator.geometry(true)));
                }
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.integrated_batches;
                stats_.mirror_blocks = mirror.chunk_count();
                stats_.esdf_blocks = integrator.chunk_count(true);
                stats_.last_input_blocks = updates.size();
                stats_.last_propagated_blocks = esdf_stats.propagated_chunks;
                stats_.last_apply_ms = apply_time;
                stats_.last_integrate_ms = integrate_time;
            }
            const auto now = Clock::now();
            if (now >= next_report)
            {
                report_stats();
                next_report = now + std::chrono::seconds(2);
            }
            next_update += update_period;
            if (next_update < now)
            {
                next_update = now + update_period;
            }
        }
    }
    catch (const std::exception& error)
    {
        if (error_callback_)
        {
            error_callback_("asynchronous ESDF failed: " + std::string(error.what()));
        }
    }
}

void EsdfUpdatePipeline::report_stats() const
{
    if (!info_callback_)
    {
        return;
    }
    const auto current = stats();
    std::ostringstream message;
    message << std::fixed << std::setprecision(2)
            << "ESDF async: batches=" << current.integrated_batches
            << " mirror=" << current.mirror_blocks << " esdf=" << current.esdf_blocks
            << " input=" << current.last_input_blocks
            << " propagated=" << current.last_propagated_blocks
            << " pending=" << current.pending_blocks << " coalesced=" << current.coalesced_blocks
            << " dropped=" << current.dropped_blocks << " apply=" << current.last_apply_ms << "ms"
            << " integrate=" << current.last_integrate_ms << "ms";
    if (current.slice_center_ready)
    {
        message << " slice_z=" << current.slice_center_z_m << 'm';
    }
    info_callback_(message.str());
}

} // namespace esdf_drm
