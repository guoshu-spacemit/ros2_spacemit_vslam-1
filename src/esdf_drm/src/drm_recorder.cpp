/**
 * @file drm_recorder.cpp
 * @brief Record the renderer output without modifying its shared input buffers.
 */
#include "drm_recorder.h"

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <dlfcn.h>

#include <mux/mux_api.h>
#include <mux/mux_type.h>
#include <sys/vb_api.h>
#include <sys/vb_type.h>
#include <v2d/v2d_api.h>
#include <venc/venc_api.h>
#include <venc/venc_type.h>

namespace esdf_drm
{
namespace
{

void require_mpp(S32 result, const char* operation)
{
    if (result != 0)
    {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
    }
}

std::string expand_time_pattern(const std::string& pattern)
{
    const std::time_t current_time = std::time(nullptr);
    std::tm local_time{};
    if (localtime_r(&current_time, &local_time) == nullptr)
    {
        throw std::runtime_error("localtime_r failed for record output path");
    }
    char expanded[MUX_PATTERN_MAX_LEN]{};
    if (std::strftime(expanded, sizeof(expanded), pattern.c_str(), &local_time) == 0)
    {
        throw std::runtime_error("expanded record output path is too long");
    }
    return expanded;
}

} // namespace

class DrmRecorder::Implementation
{
public:
    Implementation(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, DrmRecorderConfig config)
        : logger_(std::move(logger)), clock_(std::move(clock)), config_(std::move(config))
    {
    }

    ~Implementation() { stop(); }

    bool start(int width, int height, std::string* error)
    {
        if (!config_.enabled)
        {
            return false;
        }
        if (active_.load())
        {
            return true;
        }
        try
        {
            validate(width, height);
            resolved_output_path_ = expand_time_pattern(config_.output_pattern);
            const std::filesystem::path output(resolved_output_path_);
            if (!output.parent_path().empty())
            {
                std::filesystem::create_directories(output.parent_path());
            }
            width_ = width;
            height_ = height;

            mpp_global_handle_ = dlopen("libmpp.so.1", RTLD_NOW | RTLD_GLOBAL);
            if (mpp_global_handle_ == nullptr)
            {
                throw std::runtime_error(std::string("failed to expose MPP symbols: ") + dlerror());
            }

            VbPoolCfg pool{};
            pool.u32BufSize =
                static_cast<U32>(static_cast<std::uint64_t>(width_) * height_ * 3U / 2U);
            pool.u32BufCnt = static_cast<U32>(config_.pool_depth);
            pool.eModId = MPP_ID_V2D;
            pool.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
            pool_ = VB_CreatePool(&pool);
            if (pool_ == 0)
            {
                throw std::runtime_error("VB_CreatePool failed for DRM recorder");
            }

            require_mpp(VENC_Init(), "VENC_Init");
            venc_initialized_ = true;
            require_mpp(MUX_Init(), "MUX_Init");
            mux_initialized_ = true;

            VencChnAttr venc{};
            venc.eCodecType = MPP_STREAM_CODEC_H265;
            venc.eInputPixelFormat = MPP_PIXEL_FORMAT_NV12;
            venc.u32Width = static_cast<U32>(width_);
            venc.u32Height = static_cast<U32>(height_);
            venc.u32Align = 1;
            venc.u32Bitrate = static_cast<U32>(config_.bitrate_bps);
            venc.u32FrameRate = static_cast<U32>(config_.fps);
            venc.u32Gop = static_cast<U32>(config_.gop);
            venc.eFrameBufMode = VENC_FRAME_BUF_DMABUF_EXTERNAL;
            venc.eRcMode = VENC_RC_MODE_CBR;
            require_mpp(VENC_CreateChn(config_.venc_channel, &venc), "VENC_CreateChn");
            venc_created_ = true;
            require_mpp(VENC_EnableChn(config_.venc_channel), "VENC_EnableChn");
            venc_enabled_ = true;

            MuxChnAttr mux{};
            mux.eOutputType = MUX_OUTPUT_FILE;
            mux.stStreamAttr.eCodecType = MUX_CODEC_H265;
            mux.stStreamAttr.u32Width = static_cast<U32>(width_);
            mux.stStreamAttr.u32Height = static_cast<U32>(height_);
            mux.stStreamAttr.u32Fps = static_cast<U32>(config_.fps);
            mux.stStreamAttr.u32BitrateKbps = static_cast<U32>(config_.bitrate_bps / 1000);
            mux.stSegment.eFileFormat = MUX_FILE_FMP4;
            mux.stSegment.u32FragDurationMs = 0;
            mux.stSegment.u32FsyncIntervalMs = 1000;
            std::snprintf(mux.stSegment.szPattern, sizeof(mux.stSegment.szPattern), "%s",
                resolved_output_path_.c_str());
            require_mpp(MUX_CreateChn(config_.mux_channel, &mux), "MUX_CreateChn");
            mux_created_ = true;
            require_mpp(MUX_StartChn(config_.mux_channel), "MUX_StartChn");
            mux_started_ = true;

            stopping_ = false;
            next_capture_ = std::chrono::steady_clock::now();
            active_.store(true);
            worker_ = std::thread(&Implementation::worker_loop, this);
            RCLCPP_INFO(logger_,
                "DRM recording enabled: final composite %dx%d H.265 "
                "timing=%s fps=%d -> %s",
                width_, height_, config_.use_source_timestamps ? "source-cfr" : "wall", config_.fps,
                resolved_output_path_.c_str());
            return true;
        }
        catch (const std::exception& exception)
        {
            active_.store(false);
            shutdown_mpp();
            if (error != nullptr)
            {
                *error = exception.what();
            }
            return false;
        }
    }

    void submit(const esdf_drm::mapping::RenderFrame& source, std::int64_t source_timestamp_ns)
    {
        if (!active_.load() || source.dma_fd < 0 ||
            source.width != static_cast<std::uint32_t>(width_) ||
            source.height != static_cast<std::uint32_t>(height_))
        {
            return;
        }
        std::uint64_t wall_pts_us = 0;
        std::uint64_t source_index = 0;
        if (config_.use_source_timestamps)
        {
            if (source_timestamp_ns < 0 ||
                (!config_.source_timestamp_origin_zero && source_timestamp_ns == 0))
            {
                ++ignored_render_frames_;
                return;
            }
            if (!source_timeline_initialized_)
            {
                first_source_timestamp_ns_ =
                    config_.source_timestamp_origin_zero ? 0 : source_timestamp_ns;
                source_timeline_initialized_ = true;
            }
            if (source_timestamp_ns < first_source_timestamp_ns_ ||
                source_timestamp_ns < last_source_timestamp_ns_)
            {
                ++ignored_render_frames_;
                return;
            }
            last_source_timestamp_ns_ = source_timestamp_ns;
            const std::uint64_t elapsed_ns =
                static_cast<std::uint64_t>(source_timestamp_ns - first_source_timestamp_ns_);
            source_index = (elapsed_ns * static_cast<std::uint64_t>(config_.fps) + 500000000ULL) /
                1000000000ULL;
            if (last_scheduled_source_index_ >= 0 &&
                source_index <= static_cast<std::uint64_t>(last_scheduled_source_index_))
            {
                ++ignored_render_frames_;
                return;
            }
        }
        else
        {
            const auto now = std::chrono::steady_clock::now();
            if (now < next_capture_)
            {
                return;
            }
            const auto capture_period =
                std::chrono::nanoseconds(1000000000LL / static_cast<std::int64_t>(config_.fps));
            next_capture_ += capture_period;
            if (next_capture_ + capture_period < now)
            {
                next_capture_ = now + capture_period;
            }
            wall_pts_us = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch())
                    .count());
        }
        UL output = VB_GetBuffer(pool_, 0);
        if (output == 0)
        {
            ++dropped_frames_;
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000, "DRM recorder buffer pool exhausted; dropping frame");
            return;
        }

        V2DHandle job = 0;
        try
        {
            const auto begin = std::chrono::steady_clock::now();
            VideoFrameInfo input_frame = make_source_frame(source);
            VideoFrameInfo output_frame = make_output_frame(output);
            require_mpp(V2D_BeginJob(&job), "V2D_BeginJob");
            V2DArea area{0, 0, static_cast<U16>(width_), static_cast<U16>(height_)};
            const S32 add_result = V2D_AddBitblitTask(
                job, &input_frame, &area, &output_frame, &area, V2D_CSC_MODE_RGB_2_BT709NARROW);
            if (add_result != 0)
            {
                (void)V2D_CancelJob(job);
                job = 0;
                require_mpp(add_result, "V2D_AddBitblitTask");
            }
            require_mpp(V2D_EndJob(job), "V2D_EndJob");
            job = 0;
            const auto end = std::chrono::steady_clock::now();
            v2d_time_us_.fetch_add(static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()));
            output_frame.eFrameType = FRAME_TYPE_VENC;
            output_frame.eModId = MPP_ID_VENC;
            (void)VB_UpdateBufferFrameInfo(output, &output_frame);

            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load())
            {
                (void)VB_ReleaseBuffer(output);
                return;
            }
            ++converted_frames_;
            QueuedFrame converted{output, output_frame, source_index, wall_pts_us};
            if (config_.use_source_timestamps)
            {
                // Encode one access unit per complete rendered frame and preserve its source PTS.
                // MP4 represents a missing render interval with sample duration. Re-encoding the
                // same DMA-BUF once per CFR tick serializes VENC and turns transient backpressure
                // into an ever-growing queue.
                converted.output_index = source_index;
                pending_frames_.push_back(std::move(converted));
                last_scheduled_source_index_ = static_cast<std::int64_t>(source_index);
                captured_frames_.fetch_add(1);
            }
            else
            {
                pending_frames_.push_back(std::move(converted));
                captured_frames_.fetch_add(1);
            }
            condition_.notify_one();
        }
        catch (const std::exception& exception)
        {
            if (job != 0)
            {
                (void)V2D_CancelJob(job);
            }
            (void)VB_ReleaseBuffer(output);
            ++dropped_frames_;
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000, "DRM recording frame dropped: %s", exception.what());
        }
    }

    void stop()
    {
        if (!active_.exchange(false) && !worker_.joinable())
        {
            shutdown_mpp();
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
        {
            worker_.join();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            while (!pending_frames_.empty())
            {
                (void)VB_ReleaseBuffer(pending_frames_.front().buffer);
                pending_frames_.pop_front();
            }
        }
        update_current_file();
        shutdown_mpp();
        const DrmRecorderStats snapshot = stats();
        if (snapshot.encoded_frames > 0)
        {
            RCLCPP_INFO(logger_,
                "DRM recording finalized: %s frames=%" PRIu64 " dropped=%" PRIu64 " bytes=%" PRIu64
                " source=%.6fs output=%.6fs",
                snapshot.current_file.empty() ? resolved_output_path_.c_str()
                    : snapshot.current_file.c_str(),
                snapshot.encoded_frames, snapshot.dropped_frames, snapshot.encoded_bytes,
                snapshot.source_duration_seconds, snapshot.output_duration_seconds);
        }
    }

    DrmRecorderStats stats() const
    {
        DrmRecorderStats result;
        result.captured_frames = captured_frames_.load();
        result.converted_frames = converted_frames_.load();
        result.ignored_render_frames = ignored_render_frames_.load();
        result.submitted_frames = submitted_frames_.load();
        result.encoded_frames = encoded_frames_.load();
        result.dropped_frames = dropped_frames_.load();
        result.encoded_bytes = encoded_bytes_.load();
        const std::uint64_t converted = result.converted_frames;
        result.average_v2d_ms = converted == 0
            ? 0.0
            : static_cast<double>(v2d_time_us_.load()) / static_cast<double>(converted) / 1000.0;
        if (source_timeline_initialized_ && last_source_timestamp_ns_ >= first_source_timestamp_ns_)
        {
            result.source_duration_seconds =
                static_cast<double>(last_source_timestamp_ns_ - first_source_timestamp_ns_) /
                1000000000.0;
        }
        const std::uint64_t first_encoded_pts_us = first_encoded_pts_us_.load();
        const std::uint64_t last_encoded_pts_us = last_encoded_pts_us_.load();
        if (first_encoded_pts_us != std::numeric_limits<std::uint64_t>::max() &&
            last_encoded_pts_us >= first_encoded_pts_us)
        {
            result.output_duration_seconds =
                static_cast<double>(last_encoded_pts_us - first_encoded_pts_us) / 1000000.0 +
                1.0 / static_cast<double>(config_.fps);
        }
        {
            std::lock_guard<std::mutex> lock(file_mutex_);
            result.current_file = current_file_;
        }
        return result;
    }

    bool active() const { return active_.load(); }

private:
    struct QueuedFrame
    {
        UL buffer{0};
        VideoFrameInfo frame{};
        std::uint64_t output_index{0};
        std::uint64_t wall_pts_us{0};
    };

    void validate(int width, int height) const
    {
        if (width <= 0 || height <= 0 || (width % 2) != 0 || (height % 2) != 0)
        {
            throw std::runtime_error("recording dimensions must be positive and even");
        }
        if (config_.output_pattern.empty() || config_.output_pattern.size() >= MUX_PATTERN_MAX_LEN)
        {
            throw std::runtime_error("record_output_pattern is empty or too long");
        }
        if (config_.fps <= 0 || config_.fps > 240 || config_.bitrate_bps < 1000 ||
            config_.gop <= 0 || config_.pool_depth < 3 || config_.pool_depth > 16 ||
            config_.venc_channel < 0 || config_.venc_channel >= VENC_MAX_CHN ||
            config_.mux_channel < 0 || config_.mux_channel >= MUX_MAX_CHN)
        {
            throw std::runtime_error("invalid DRM recording parameters");
        }
    }

    VideoFrameInfo make_source_frame(const esdf_drm::mapping::RenderFrame& source) const
    {
        VideoFrameInfo frame{};
        frame.eFrameType = FRAME_TYPE_COMMON;
        frame.eModId = MPP_ID_V2D;
        frame.stCommFrameInfo.u32Width = static_cast<U32>(width_);
        frame.stCommFrameInfo.u32Height = static_cast<U32>(height_);
        frame.stCommFrameInfo.u32Align = 1;
        frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_BGRA;
        frame.stCommFrameInfo.eCompressMode = COMPRESS_MODE_NONE;
        frame.stVFrame.u32TotalSize = static_cast<U32>(source.stride_bytes * source.height);
        frame.stVFrame.u32PlaneNum = 1;
        frame.stVFrame.u32PlaneStride[0] = static_cast<U32>(source.stride_bytes);
        frame.stVFrame.u32PlaneSize[0] = frame.stVFrame.u32TotalSize;
        frame.stVFrame.u32PlaneSizeValid[0] = frame.stVFrame.u32TotalSize;
        frame.stVFrame.u32Fd[0] = static_cast<UL>(source.dma_fd);
        return frame;
    }

    VideoFrameInfo make_output_frame(UL output)
    {
        void* address = nullptr;
        S32 dma_fd = -1;
        U64 physical = 0;
        if (VB_GetVirAddr(output, &address) != 0 || address == nullptr ||
            VB_GetDmaBufFd(output, &dma_fd) != 0 || dma_fd < 0 ||
            VB_GetPhyAddr(output, &physical) != 0)
        {
            throw std::runtime_error("failed to resolve recorder NV12 DMA-BUF");
        }
        VideoFrameInfo frame{};
        frame.eFrameType = FRAME_TYPE_COMMON;
        frame.eModId = MPP_ID_V2D;
        frame.ulPoolId = pool_;
        frame.ulBufferId = output;
        frame.stCommFrameInfo.u32Width = static_cast<U32>(width_);
        frame.stCommFrameInfo.u32Height = static_cast<U32>(height_);
        frame.stCommFrameInfo.u32Align = 1;
        frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
        frame.stCommFrameInfo.eCompressMode = COMPRESS_MODE_NONE;
        frame.stVFrame.u32TotalSize = static_cast<U32>(width_ * height_ * 3 / 2);
        frame.stVFrame.u32PlaneNum = 2;
        frame.stVFrame.u32PlaneStride[0] = static_cast<U32>(width_);
        frame.stVFrame.u32PlaneStride[1] = static_cast<U32>(width_);
        frame.stVFrame.u32PlaneSize[0] = static_cast<U32>(width_ * height_);
        frame.stVFrame.u32PlaneSize[1] = frame.stVFrame.u32PlaneSize[0] / 2U;
        frame.stVFrame.u32PlaneSizeValid[0] = frame.stVFrame.u32PlaneSize[0];
        frame.stVFrame.u32PlaneSizeValid[1] = frame.stVFrame.u32PlaneSize[1];
        frame.stVFrame.u32Fd[0] = static_cast<UL>(dma_fd);
        frame.stVFrame.u32Fd[1] = static_cast<UL>(dma_fd);
        frame.stVFrame.u64PlanePhyAddr[0] = physical;
        frame.stVFrame.u64PlanePhyAddr[1] = physical + frame.stVFrame.u32PlaneSize[0];
        const auto base = reinterpret_cast<std::uintptr_t>(address);
        frame.stVFrame.ulPlaneVirAddr[0] = static_cast<UL>(base);
        frame.stVFrame.ulPlaneVirAddr[1] = static_cast<UL>(base + frame.stVFrame.u32PlaneSize[0]);
        return frame;
    }

    void worker_loop()
    {
        for (;;)
        {
            std::optional<QueuedFrame> queued;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return stopping_ || !pending_frames_.empty(); });
                if (!pending_frames_.empty())
                {
                    queued = std::move(pending_frames_.front());
                    pending_frames_.pop_front();
                }
                else if (stopping_)
                {
                    break;
                }
            }
            if (!queued)
            {
                continue;
            }
            queued->frame.u32Idx = frame_index_.fetch_add(1);
            if (config_.use_source_timestamps)
            {
                // MPP MUX converts every adjacent microsecond PTS delta to 90 kHz independently.
                // Keep the nominal period constant so fractional microsecond alternation cannot
                // accumulate truncation error in the MP4 timeline.
                const std::uint64_t frame_period_us =
                    (1000000ULL + static_cast<std::uint64_t>(config_.fps) - 1ULL) /
                    static_cast<std::uint64_t>(config_.fps);
                queued->frame.stVFrame.u64PTS = queued->output_index * frame_period_us;
            }
            else
            {
                queued->frame.stVFrame.u64PTS = queued->wall_pts_us;
            }
            (void)VB_UpdateBufferFrameInfo(queued->buffer, &queued->frame);
            S32 result = ERR_VENC_OK;
            for (int attempt = 0; attempt < 10; ++attempt)
            {
                result = VENC_SendFrame(config_.venc_channel, &queued->frame, 0);
                if (result == ERR_VENC_OK)
                {
                    break;
                }
                // VENC_SendFrame is non-blocking even when a timeout is supplied. Drain an
                // encoded packet to free an input slot, then retry without treating normal
                // encoder backpressure as a dropped source frame.
                (void)drain_streams(1000);
            }
            if (result != ERR_VENC_OK)
            {
                ++dropped_frames_;
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 2000, "DRM VENC_SendFrame dropped a frame: %d", result);
            }
            else
            {
                ++submitted_frames_;
                // Keep several frames in flight so V2D, VENC and MUX run as a pipeline.
                (void)drain_streams(0);
            }
            (void)VB_ReleaseBuffer(queued->buffer);
        }
        VideoFrameInfo eos{};
        eos.eFrameType = FRAME_TYPE_VENC;
        eos.eModId = MPP_ID_VENC;
        eos.stVFrame.u32FrameFlag = MPP_FRAME_FLAG_EOS;
        const S32 eos_result = VENC_SendFrame(config_.venc_channel, &eos, 1000);
        if (eos_result != ERR_VENC_OK)
        {
            ++dropped_frames_;
            RCLCPP_WARN(logger_, "DRM VENC EOS submission failed: %d", eos_result);
            (void)drain_streams(1000);
            return;
        }

        bool reached_eos = false;
        for (int attempt = 0; attempt < 5 && !reached_eos; ++attempt)
        {
            reached_eos = drain_streams(1000);
        }
        if (!reached_eos)
        {
            RCLCPP_WARN(logger_, "DRM VENC did not report EOS within 5 seconds");
        }
    }

    bool drain_streams(U32 first_timeout_ms)
    {
        if (!venc_enabled_ || !mux_started_)
        {
            return false;
        }
        U32 timeout_ms = first_timeout_ms;
        for (;;)
        {
            StreamBufferInfo stream{};
            const S32 result = VENC_GetStream(config_.venc_channel, &stream, timeout_ms);
            timeout_ms = 0;
            if (result == ERR_VENC_EOS)
            {
                return true;
            }
            if (result == ERR_VENC_NO_STREAM || result == ERR_VENC_TIMEOUT)
            {
                break;
            }
            if (result != ERR_VENC_OK)
            {
                ++dropped_frames_;
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 2000, "DRM VENC_GetStream failed: %d", result);
                break;
            }
            MuxPacket packet{};
            packet.pu8Data = stream.pu8Addr;
            packet.u32Size = stream.u32Size;
            packet.bKeyFrame = stream.bKeyFrame;
            packet.eCodecType = MUX_CODEC_H265;
            packet.u64PTS = stream.u64PTS;
            const S32 mux_result = MUX_SendPacket(config_.mux_channel, &packet);
            if (mux_result != ERR_MUX_OK)
            {
                ++dropped_frames_;
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 2000, "DRM MUX_SendPacket failed: %d", mux_result);
            }
            else
            {
                ++encoded_frames_;
                encoded_bytes_.fetch_add(stream.u32Size);
                std::uint64_t expected = std::numeric_limits<std::uint64_t>::max();
                (void)first_encoded_pts_us_.compare_exchange_strong(expected, stream.u64PTS);
                last_encoded_pts_us_.store(stream.u64PTS);
                update_current_file();
            }
            (void)VENC_ReleaseStream(config_.venc_channel, &stream);
        }
        return false;
    }

    void update_current_file()
    {
        if (!mux_started_)
        {
            return;
        }
        MuxChnStat status{};
        if (MUX_GetChnStat(config_.mux_channel, &status) == ERR_MUX_OK &&
            status.szCurFile[0] != '\0')
        {
            std::lock_guard<std::mutex> lock(file_mutex_);
            current_file_ = status.szCurFile;
        }
    }

    void shutdown_mpp()
    {
        if (mux_started_)
        {
            (void)MUX_StopChn(config_.mux_channel);
            mux_started_ = false;
        }
        if (mux_created_)
        {
            (void)MUX_DestroyChn(config_.mux_channel);
            mux_created_ = false;
        }
        if (venc_enabled_)
        {
            (void)VENC_DisableChn(config_.venc_channel);
            venc_enabled_ = false;
        }
        if (venc_created_)
        {
            (void)VENC_DestroyChn(config_.venc_channel);
            venc_created_ = false;
        }
        if (mux_initialized_)
        {
            (void)MUX_Exit();
            mux_initialized_ = false;
        }
        if (venc_initialized_)
        {
            (void)VENC_Exit();
            venc_initialized_ = false;
        }
        if (pool_ != 0)
        {
            (void)VB_DestroyPool(pool_);
            pool_ = 0;
        }
        if (mpp_global_handle_ != nullptr)
        {
            (void)dlclose(mpp_global_handle_);
            mpp_global_handle_ = nullptr;
        }
    }

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    DrmRecorderConfig config_;
    int width_{0};
    int height_{0};
    void* mpp_global_handle_{nullptr};
    UL pool_{0};
    bool venc_initialized_{false};
    bool venc_created_{false};
    bool venc_enabled_{false};
    bool mux_initialized_{false};
    bool mux_created_{false};
    bool mux_started_{false};
    std::atomic<bool> active_{false};
    bool stopping_{false};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<QueuedFrame> pending_frames_;
    std::atomic<std::uint32_t> frame_index_{0};
    std::atomic<std::uint64_t> captured_frames_{0};
    std::atomic<std::uint64_t> converted_frames_{0};
    std::atomic<std::uint64_t> ignored_render_frames_{0};
    std::atomic<std::uint64_t> submitted_frames_{0};
    std::atomic<std::uint64_t> encoded_frames_{0};
    std::atomic<std::uint64_t> dropped_frames_{0};
    std::atomic<std::uint64_t> encoded_bytes_{0};
    std::atomic<std::uint64_t> v2d_time_us_{0};
    std::chrono::steady_clock::time_point next_capture_{};
    std::int64_t first_source_timestamp_ns_{0};
    std::int64_t last_source_timestamp_ns_{0};
    std::int64_t last_scheduled_source_index_{-1};
    bool source_timeline_initialized_{false};
    std::atomic<std::uint64_t> first_encoded_pts_us_{std::numeric_limits<std::uint64_t>::max()};
    std::atomic<std::uint64_t> last_encoded_pts_us_{0};
    mutable std::mutex file_mutex_;
    std::string current_file_;
    std::string resolved_output_path_;
};

DrmRecorder::DrmRecorder(
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, DrmRecorderConfig config)
    : implementation_(
        std::make_unique<Implementation>(std::move(logger), std::move(clock), std::move(config)))
{
}

DrmRecorder::~DrmRecorder() = default;

bool DrmRecorder::start(int width, int height, std::string* error)
{
    return implementation_->start(width, height, error);
}

void DrmRecorder::submit(
    const esdf_drm::mapping::RenderFrame& frame, std::int64_t source_timestamp_ns)
{
    implementation_->submit(frame, source_timestamp_ns);
}

void DrmRecorder::stop() { implementation_->stop(); }

DrmRecorderStats DrmRecorder::stats() const { return implementation_->stats(); }

bool DrmRecorder::active() const { return implementation_->active(); }

} // namespace esdf_drm
