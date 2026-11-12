#include "camera_mpp_publisher.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

#include "decoder_input_pool_mpp.h"
#include "demux/demux_api.h"
#include "mjpeg_packet_reader.h"
#include "mpp_stream_compat.h"
#include "spacemit_camera/gpio_exposure.h"
#include "sys/sys_api.h"
#include "sys/vb_api.h"
#include "uvc/uvc_api.h"
#include "v2d/v2d_api.h"
#include "vdec/vdec_api.h"

namespace camera_io
{

using detail::configure_decoder_input_pool;
using detail::invalidate_legacy_stream_dma_fd;
using detail::set_decoder_input_pool_mode;

bool CameraMppPublisher::is_file_input() const { return config_.input_type == "file"; }

bool CameraMppPublisher::direct_file_packets() const
{
    return file_packet_reader_ == "libavformat";
}

const std::string& CameraMppPublisher::input_description() const
{
    return is_file_input() ? config_.input_file : config_.device;
}

MppStreamCodecType CameraMppPublisher::decoder_codec() const
{
    if (config_.input_format == "mjpeg")
    {
        return MPP_STREAM_CODEC_MJPEG;
    }
    if (config_.input_format == "h264")
    {
        return MPP_STREAM_CODEC_H264;
    }
    if (config_.input_format == "h265")
    {
        return MPP_STREAM_CODEC_H265;
    }
    throw std::runtime_error("unsupported MPP decoder codec");
}

void CameraMppPublisher::open_decoder()
{
    VdecChnAttr decoder_attr{};
    decoder_attr.eCodecType = decoder_codec();
    decoder_attr.eOutputPixelFormat = MPP_PIXEL_FORMAT_NV12;
    // The K3 Linlon decoder requires an explicit output alignment for
    // 4000x1200 streams. Leaving this zero can queue invalid buffers in
    // the hardware down-scale path.
    decoder_attr.u32Align = 16;
    decoder_attr.u32Width = config_.width;
    decoder_attr.u32Height = config_.height;
    // The recording contract prohibits B frames. Keeping decoder output
    // ordered lets each decoded frame retain its embedded camera metadata.
    decoder_attr.bIsFrameReordering = MPP_FALSE;
    decoder_attr.bDispErrorFrame = MPP_FALSE;
    decoder_attr.stScale.bScaleEnable = MPP_FALSE;
    decoder_attr.u32BufCnt = static_cast<U32>(config_.decoder_buffer_count);
    set_decoder_input_pool_mode(decoder_attr, config_.decoder_input_pool);
    require_ok(VDEC_CreateChn(vdec_channel_id_, &decoder_attr), "VDEC_CreateChn");
    vdec_created_ = true;
    require_ok(VDEC_EnableChn(vdec_channel_id_), "VDEC_EnableChn");
    vdec_enabled_ = true;
}

void CameraMppPublisher::configure_input_pool(const MppNode& source, const MppNode& sink)
{
    require_ok(configure_decoder_input_pool(source, sink, config_.decoder_input_pool),
        "SYS_ConfigStreamDmaBufPool");
    if (config_.decoder_input_pool.enabled)
    {
        RCLCPP_INFO(get_logger(), "VDEC input: fixed DMA-BUF pool, %u slots x %u bytes",
            config_.decoder_input_pool.slot_count, config_.decoder_input_pool.slot_bytes);
    }
}

void CameraMppPublisher::open_pipeline()
{
    require_ok(SYS_Init(), "SYS_Init");
    sys_initialized_ = true;
    require_ok(VB_Init(), "VB_Init");
    vb_initialized_ = true;
    if (is_file_input())
    {
        if (direct_file_packets())
        {
            require_ok(VDEC_Init(), "VDEC_Init");
            vdec_initialized_ = true;
            open_decoder();
            if (config_.decoder_input_pool.enabled)
            {
                // SYS owns this logical source and the preallocated input ring.
                // libavformat supplies packets; no MPP DEMUX channel is opened.
                demux_source_ = {MPP_ID_DEMUX, 0, kDemuxChn};
                vdec_sink_ = {MPP_ID_VDEC, 0, vdec_channel_id_};
                require_ok(SYS_Bind(&demux_source_, &vdec_sink_), "SYS_Bind(file packets->VDEC)");
                demux_vdec_bound_ = true;
                configure_input_pool(demux_source_, vdec_sink_);
                RCLCPP_INFO(get_logger(),
                    "MP4 input: libavformat original MJPEG packets -> "
                    "SYS_SendStream -> fixed DMA-BUF pool -> VDEC");
            }
            else
            {
                RCLCPP_INFO(get_logger(),
                    "MP4 input: libavformat original MJPEG packets -> "
                    "VDEC_SendStream; decoder_input_pool disabled in configuration");
            }
        }
        else
        {
            require_ok(DEMUX_Init(), "DEMUX_Init");
            demux_initialized_ = true;
            require_ok(VDEC_Init(), "VDEC_Init");
            vdec_initialized_ = true;

            DemuxChnAttr demux_attr{};
            demux_attr.eInputType = DEMUX_INPUT_FILE;
            std::strncpy(
                demux_attr.szUrl, config_.input_file.c_str(), sizeof(demux_attr.szUrl) - 1);
            demux_attr.u32OpenTimeoutMs = 5000;
            demux_attr.u32RwTimeoutMs = 5000;
            // MPP inserts VPS/SPS/PPS before each IDR for reliable VDEC
            // startup without CPU packet handling.
            demux_attr.bInjectPS = MPP_TRUE;
            require_ok(DEMUX_CreateChn(kDemuxChn, &demux_attr), "DEMUX_CreateChn");
            demux_created_ = true;

            open_decoder();
            require_ok(DEMUX_GetSrcNode(kDemuxChn, &demux_source_), "DEMUX_GetSrcNode");
            vdec_sink_ = {MPP_ID_VDEC, 0, vdec_channel_id_};
            require_ok(SYS_Bind(&demux_source_, &vdec_sink_), "SYS_Bind(DEMUX->VDEC)");
            demux_vdec_bound_ = true;
            configure_input_pool(demux_source_, vdec_sink_);
        }
    }
    else
    {
        require_ok(UVC_Init(), "UVC_Init");
        uvc_initialized_ = true;
        if (config_.decode_enabled)
        {
            require_ok(VDEC_Init(), "VDEC_Init");
            vdec_initialized_ = true;
        }

        UvcDevAttr device_attr{};
        std::strncpy(
            device_attr.acDevNode, config_.device.c_str(), sizeof(device_attr.acDevNode) - 1);
        require_ok(UVC_CreateDev(uvc_device_id_, &device_attr), "UVC_CreateDev");
        uvc_created_ = true;
        require_ok(UVC_EnableDev(uvc_device_id_), "UVC_EnableDev");
        uvc_device_enabled_ = true;

        UvcChnAttr channel_attr{};
        channel_attr.u32Width = config_.width;
        channel_attr.u32Height = config_.height;
        channel_attr.ePixelFormat = MPP_PIXEL_FORMAT_MJPEG;
        channel_attr.u32Fps = config_.fps;
        // A positive depth exposes the original compressed VB block so it
        // can be exported before the same payload is sent to VDEC.
        channel_attr.u32Depth = config_.compressed_enabled ? 6U : 0U;
        require_ok(
            UVC_SetChnAttr(uvc_device_id_, uvc_channel_id_, &channel_attr), "UVC_SetChnAttr");
        if (config_.decode_enabled)
        {
            open_decoder();
        }
        if (config_.decode_enabled)
        {
            MppNode source{MPP_ID_UVC, uvc_device_id_, uvc_channel_id_};
            MppNode destination{MPP_ID_VDEC, 0, vdec_channel_id_};
            require_ok(SYS_Bind(&source, &destination), "SYS_Bind(UVC->VDEC)");
            uvc_vdec_bound_ = true;
            configure_input_pool(source, destination);
        }
        // Finish binding and preallocation before the producer can submit input.
        require_ok(UVC_EnableChn(uvc_device_id_, uvc_channel_id_), "UVC_EnableChn");
        uvc_channel_enabled_ = true;
    }

    open_eye_pool();
}

void CameraMppPublisher::open_eye_pool()
{
    if (config_.decode_enabled && !config_.full_only)
    {
        VbPoolCfg pool_config{};
        pool_config.u32BufSize =
            static_cast<U32>(config_.left.output_width * config_.left.output_height * 3 / 2);
        pool_config.u32BufCnt = static_cast<U32>(2 * (config_.max_exported_pairs + 2));
        pool_config.eModId = MPP_ID_V2D;
        pool_config.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
        eye_pool_ = VB_CreatePool(&pool_config);
        if (eye_pool_ == 0)
        {
            throw std::runtime_error("VB_CreatePool failed");
        }
    }
}

void CameraMppPublisher::close_pipeline() noexcept
{
    while (!exports_.empty())
    {
        release_oldest_export();
    }
    while (!compressed_exports_.empty())
    {
        (void)VB_Unexport(compressed_exports_.front());
        compressed_exports_.pop_front();
    }
    stop_file_input();
    // Return all decoder input leases before unbinding the fixed ring.
    if (uvc_channel_enabled_)
    {
        (void)UVC_DisableChn(uvc_device_id_, uvc_channel_id_);
        uvc_channel_enabled_ = false;
    }
    if (vdec_enabled_)
    {
        (void)VDEC_DisableChn(vdec_channel_id_);
        vdec_enabled_ = false;
    }
    if (demux_vdec_bound_)
    {
        (void)SYS_UnBind(&demux_source_, &vdec_sink_);
        demux_vdec_bound_ = false;
    }
    if (uvc_vdec_bound_)
    {
        MppNode source{MPP_ID_UVC, uvc_device_id_, uvc_channel_id_};
        MppNode destination{MPP_ID_VDEC, 0, vdec_channel_id_};
        (void)SYS_UnBind(&source, &destination);
        uvc_vdec_bound_ = false;
    }
    if (eye_pool_ != 0)
    {
        (void)VB_DestroyPool(eye_pool_);
        eye_pool_ = 0;
    }
    if (vdec_created_)
    {
        (void)VDEC_DestroyChn(vdec_channel_id_);
        vdec_created_ = false;
    }
    if (uvc_device_enabled_)
    {
        (void)UVC_DisableDev(uvc_device_id_);
        uvc_device_enabled_ = false;
    }
    if (uvc_created_)
    {
        (void)UVC_DestroyDev(uvc_device_id_);
        uvc_created_ = false;
    }
    if (vdec_initialized_)
    {
        (void)VDEC_Exit();
        vdec_initialized_ = false;
    }
    if (uvc_initialized_)
    {
        (void)UVC_Exit();
        uvc_initialized_ = false;
    }
    if (demux_created_)
    {
        (void)DEMUX_DestroyChn(kDemuxChn);
        demux_created_ = false;
    }
    if (demux_initialized_)
    {
        (void)DEMUX_Exit();
        demux_initialized_ = false;
    }
    if (vb_initialized_)
    {
        (void)VB_Exit();
        vb_initialized_ = false;
    }
    if (sys_initialized_)
    {
        (void)SYS_Exit();
        sys_initialized_ = false;
    }
}

void CameraMppPublisher::stop_file_input() noexcept
{
    std::lock_guard<std::mutex> lock(file_start_mutex_);
    if (direct_file_packets())
    {
        capture_running_.store(false);
        file_packet_cv_.notify_all();
        if (file_packet_thread_.joinable())
            file_packet_thread_.join();
        file_packet_started_ = false;
    }
    if (demux_started_)
    {
        (void)DEMUX_StopChn(kDemuxChn);
        demux_started_ = false;
    }
}

void CameraMppPublisher::start_file_input_if_ready()
{
    if (!is_file_input() || !file_playback_active_.load() || !file_start_ready_.load())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(file_start_mutex_);
    if (demux_started_ || file_packet_started_)
    {
        return;
    }
    reset_capture_stats();
    capture_running_.store(true);
    if (config_.decode_enabled)
    {
        decode_thread_ = std::thread(&CameraMppPublisher::decode_loop, this);
    }
    if (direct_file_packets())
    {
        file_packets_submitted_.store(0);
        file_packets_read_complete_.store(false);
        file_packet_error_.store(false);
        try
        {
            file_packet_thread_ = std::thread(&CameraMppPublisher::file_packet_loop, this);
            file_packet_started_ = true;
        }
        catch (...)
        {
            capture_running_.store(false);
            if (decode_thread_.joinable())
                decode_thread_.join();
            throw;
        }
        RCLCPP_INFO(get_logger(), "MP4 libavformat playback started after ready event: %s",
            file_start_ready_topic_.empty() ? "immediate" : file_start_ready_topic_.c_str());
        return;
    }
    const S32 start_result = DEMUX_StartChn(kDemuxChn);
    if (start_result != 0)
    {
        capture_running_.store(false);
        if (decode_thread_.joinable())
        {
            decode_thread_.join();
        }
        require_ok(start_result, "DEMUX_StartChn");
    }
    demux_started_ = true;
    RCLCPP_INFO(get_logger(), "MP4 playback started after ready event: %s",
        file_start_ready_topic_.empty() ? "immediate" : file_start_ready_topic_.c_str());
}

bool CameraMppPublisher::send_file_packet(const StreamBufferInfo& stream)
{
    const bool use_pool = config_.decoder_input_pool.enabled;
    const char* operation = use_pool ? "SYS_SendStream" : "VDEC_SendStream";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (capture_running_.load() && rclcpp::ok())
    {
        S32 result;
        if (use_pool)
            result = SYS_SendStream(&demux_source_, &stream);
        else
            result = VDEC_SendStream(vdec_channel_id_, &stream, 100);
        if (result == 0)
            return true;
        // Only queue/slot exhaustion is transient on a configured SYS ring.
        // SYS_ERR_BUSY means owner/cache synchronization failure, not queue pressure.
        const bool decoder_busy = result == ERR_VDEC_BUSY || result == ERR_VDEC_TIMEOUT;
        const bool retry = use_pool ? result == SYS_ERR_FULL : decoder_busy;
        if (!retry)
            throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("timeout submitting original MP4 packet to VDEC");
        std::unique_lock<std::mutex> lock(file_packet_mutex_);
        file_packet_cv_.wait_for(
            lock, std::chrono::milliseconds(5), [this] { return !capture_running_.load(); });
    }
    return false;
}

void CameraMppPublisher::file_packet_loop() noexcept
{
    try
    {
        camera_io::MjpegPacketReader reader(config_.input_file, config_.width, config_.height,
            [this] { return !capture_running_.load() || !rclcpp::ok(); });
        camera_io::MjpegPacketReader::Packet packet;
        std::optional<std::uint64_t> first_pts;
        auto first_wall = std::chrono::steady_clock::now();
        std::uint64_t bytes = 0, large_packets = 0;
        while (capture_running_.load() && rclcpp::ok() && reader.next(&packet))
        {
            if (!first_pts)
            {
                first_pts = packet.pts_us;
                first_wall = std::chrono::steady_clock::now();
            }
            const auto delta = packet.pts_us - *first_pts;
            if (delta > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000))
                throw std::runtime_error("MP4 pacing interval is out of range");
            {
                std::unique_lock<std::mutex> lock(file_packet_mutex_);
                file_packet_cv_.wait_until(lock, first_wall + std::chrono::microseconds(delta),
                    [this] { return !capture_running_.load(); });
            }
            if (!capture_running_.load() || !rclcpp::ok())
                return;
            StreamBufferInfo stream{};
            stream.pu8Addr = const_cast<U8*>(packet.data);
            stream.u32Size = packet.bytes;
            stream.bKeyFrame = MPP_TRUE;
            stream.eCodecType = MPP_STREAM_CODEC_MJPEG;
            stream.u64PTS = packet.pts_us;
            stream.u32Width = config_.width;
            stream.u32Height = config_.height;
            invalidate_legacy_stream_dma_fd(stream);
            if (!send_file_packet(stream))
                return;
            ++file_packets_submitted_;
            bytes += packet.bytes;
            large_packets += packet.bytes > 512U * 1024U ? 1U : 0U;
        }
        if (!capture_running_.load() || !rclcpp::ok())
            return;
        file_packets_read_complete_.store(true);
        StreamBufferInfo eos{};
        eos.bEndOfStream = MPP_TRUE;
        eos.eCodecType = MPP_STREAM_CODEC_MJPEG;
        invalidate_legacy_stream_dma_fd(eos);
        if (!send_file_packet(eos))
            return;
        RCLCPP_INFO(get_logger(),
            "MP4 libavformat source EOF: packets=%" PRIu64 " bytes=%" PRIu64
            " packets_over_512KiB=%" PRIu64 "; EOS submitted",
            file_packets_submitted_.load(), bytes, large_packets);
    }
    catch (const std::exception& error)
    {
        if (capture_running_.load() && rclcpp::ok())
        {
            file_packet_error_.store(true);
            RCLCPP_ERROR(
                get_logger(), "MP4 libavformat playback FAILED (not EOF): %s", error.what());
        }
        capture_running_.store(false);
        file_packet_cv_.notify_all();
    }
    catch (...)
    {
        file_packet_error_.store(true);
        capture_running_.store(false);
        file_packet_cv_.notify_all();
        RCLCPP_ERROR(get_logger(), "MP4 libavformat playback FAILED: unknown exception");
    }
}

void CameraMppPublisher::compressed_capture_loop()
{
    while (capture_running_.load())
    {
        VideoFrameInfo compressed{};
        const S32 uvc_result = UVC_GetFrame(uvc_device_id_, uvc_channel_id_, &compressed, 100);
        if (uvc_result != 0)
        {
            ++compressed_capture_failures_;
            if (!config_.decode_enabled)
            {
                ++capture_failures_;
                report_capture_stats(std::chrono::steady_clock::now());
            }
            continue;
        }

        const std::uint32_t input_index = input_frame_index_++;
        ++compressed_capture_frames_;
        U64 token = 0;
        if (VB_Export(compressed.ulBufferId, &token) != 0)
        {
            ++compressed_capture_failures_;
            if (!config_.decode_enabled)
            {
                ++capture_failures_;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "compressed VB_Export failed");
            }
            (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
            continue;
        }

        if (config_.decode_enabled)
        {
            std::unique_lock<std::mutex> lock(pending_mutex_);
            pending_cv_.wait(lock,
                [this]()
                {
                    return !capture_running_.load() ||
                        pending_compressed_.size() < max_pending_compressed();
                });
            if (!capture_running_.load())
            {
                lock.unlock();
                (void)VB_Unexport(compressed.ulBufferId);
                (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
                break;
            }
            pending_compressed_.push_back(PendingCompressed{compressed, token});
            lock.unlock();
            pending_cv_.notify_all();
        }
        else
        {
            try
            {
                compressed_exports_.push_back(compressed.ulBufferId);
                publish_descriptor(*compressed_pub_, compressed, token, now(),
                    config_.compressed.frame_id, input_index);
                while (static_cast<int>(compressed_exports_.size()) > config_.max_exported_pairs)
                {
                    (void)VB_Unexport(compressed_exports_.front());
                    compressed_exports_.pop_front();
                }
                ++capture_frames_;
            }
            catch (const std::exception& error)
            {
                if (!compressed_exports_.empty() &&
                    compressed_exports_.back() == compressed.ulBufferId)
                {
                    compressed_exports_.pop_back();
                }
                (void)VB_Unexport(compressed.ulBufferId);
                ++capture_failures_;
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "compressed publish failed: %s", error.what());
            }
            report_capture_stats(std::chrono::steady_clock::now());
        }
        (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
    }
}

std::size_t CameraMppPublisher::max_pending_compressed() const
{
    return static_cast<std::size_t>(std::max(2, std::min(4, config_.max_exported_pairs)));
}

std::optional<CameraMppPublisher::PendingCompressed> CameraMppPublisher::take_pending_compressed(
    U64 pts)
{
    std::vector<UL> stale_handles;
    std::optional<PendingCompressed> match;
    {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        pending_cv_.wait_for(lock, std::chrono::milliseconds(50),
            [this, pts]()
            {
                return !capture_running_.load() ||
                    std::any_of(pending_compressed_.begin(), pending_compressed_.end(),
                        [pts](const PendingCompressed& pending)
                        { return pending.frame.stVFrame.u64PTS == pts; });
            });
        const auto position = std::find_if(pending_compressed_.begin(), pending_compressed_.end(),
            [pts](const PendingCompressed& pending)
            { return pending.frame.stVFrame.u64PTS == pts; });
        if (position != pending_compressed_.end())
        {
            for (auto it = pending_compressed_.begin(); it != position; ++it)
            {
                stale_handles.push_back(it->frame.ulBufferId);
            }
            match = *position;
            pending_compressed_.erase(pending_compressed_.begin(), std::next(position));
        }
    }
    for (const UL handle : stale_handles)
    {
        (void)VB_Unexport(handle);
        ++compressed_correlation_drops_;
    }
    pending_cv_.notify_all();
    return match;
}

void CameraMppPublisher::decode_once()
{
    const auto begin = std::chrono::steady_clock::now();
    VideoFrameInfo decoded{};
    const S32 result = VDEC_GetFrame(vdec_channel_id_, &decoded, 100);
    const auto decoded_at = std::chrono::steady_clock::now();
    if (result != ERR_VDEC_OK)
    {
        if (result == ERR_VDEC_EOS && is_file_input())
        {
            if (direct_file_packets())
            {
                const bool complete = file_packets_read_complete_.load() &&
                    !file_packet_error_.load() && capture_frames_ == file_packets_submitted_.load();
                if (complete)
                    RCLCPP_INFO(get_logger(),
                        "MP4 libavformat decode complete: packets=%" PRIu64 " decoded=%" PRIu64,
                        file_packets_submitted_.load(), capture_frames_);
                else
                {
                    file_packet_error_.store(true);
                    RCLCPP_ERROR(get_logger(),
                        "MP4 libavformat decode INCOMPLETE: packets=%" PRIu64 " decoded=%" PRIu64,
                        file_packets_submitted_.load(), capture_frames_);
                    capture_running_.store(false);
                    file_packet_cv_.notify_all();
                    report_capture_stats(decoded_at);
                    return;
                }
            }
            RCLCPP_INFO(get_logger(), "MP4 playback reached end of stream");
            capture_running_.store(false);
            report_capture_stats(decoded_at);
            return;
        }
        ++capture_failures_;
        if (capture_running_.load())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "VDEC_GetFrame failed: %d", result);
        }
        report_capture_stats(decoded_at);
        return;
    }

    std::optional<PendingCompressed> pending;
    if (!is_file_input() && config_.compressed_enabled)
    {
        pending = take_pending_compressed(decoded.stVFrame.u64PTS);
    }
    const std::uint32_t published_index = decoded_frame_index_++;
    const auto image_stamp = process_decoded(decoded, published_index);
    if (pending && image_stamp && compressed_pub_)
    {
        try
        {
            compressed_exports_.push_back(pending->frame.ulBufferId);
            publish_descriptor(*compressed_pub_, pending->frame, pending->token, *image_stamp,
                config_.compressed.frame_id, published_index);
            while (static_cast<int>(compressed_exports_.size()) > config_.max_exported_pairs)
            {
                (void)VB_Unexport(compressed_exports_.front());
                compressed_exports_.pop_front();
            }
        }
        catch (const std::exception& error)
        {
            if (!compressed_exports_.empty() &&
                compressed_exports_.back() == pending->frame.ulBufferId)
            {
                compressed_exports_.pop_back();
            }
            (void)VB_Unexport(pending->frame.ulBufferId);
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "compressed publish failed: %s", error.what());
        }
    }
    else if (pending)
    {
        (void)VB_Unexport(pending->frame.ulBufferId);
    }
    const auto processed_at = std::chrono::steady_clock::now();
    (void)VDEC_ReleaseFrame(vdec_channel_id_, decoded.ulBufferId);
    const auto released_at = std::chrono::steady_clock::now();

    ++capture_frames_;
    vdec_wait_ns_ +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(decoded_at - begin).count();
    process_ns_ +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(processed_at - decoded_at).count();
    release_ns_ +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(released_at - processed_at).count();
    report_capture_stats(released_at);
}

void CameraMppPublisher::decode_loop()
{
    while (capture_running_.load())
    {
        if (gpio_sync_)
        {
            const auto failure = gpio_sync_->failure();
            if (failure)
            {
                RCLCPP_ERROR(get_logger(), "GPIO exposure reader stopped: %s", failure->c_str());
                capture_running_.store(false);
                pending_cv_.notify_all();
                startup_alignment_.reset();
                gpio_sync_.reset();
                return;
            }
        }
        decode_once();
    }
}

void CameraMppPublisher::stop_capture() noexcept
{
    capture_running_.store(false);
    file_packet_cv_.notify_all();
    pending_cv_.notify_all();
    if (file_packet_thread_.joinable())
    {
        file_packet_thread_.join();
    }
    if (compressed_thread_.joinable())
    {
        compressed_thread_.join();
    }
    if (decode_thread_.joinable())
    {
        decode_thread_.join();
    }
    startup_alignment_.reset();
    gpio_sync_.reset();
    std::deque<PendingCompressed> pending;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending.swap(pending_compressed_);
    }
    for (const auto& entry : pending)
    {
        (void)VB_Unexport(entry.frame.ulBufferId);
    }
}

void CameraMppPublisher::reset_capture_stats()
{
    input_frame_index_ = 0;
    decoded_frame_index_ = 0;
    last_imu_timestamp_ns_ = 0;
    gpio_last_imu_raw_.reset();
    imu_duplicate_count_ = 0;
    imu_backward_count_ = 0;
    imu_metadata_sample_count_ = 0;
    imu_invalid_sample_count_ = 0;
    imu_raw_write_successes_ = 0;
    imu_raw_write_failures_ = 0;
    imu_fused_write_failures_ = 0;
    capture_frames_ = 0;
    capture_failures_ = 0;
    compressed_capture_frames_ = 0;
    compressed_capture_failures_ = 0;
    compressed_correlation_drops_ = 0;
    vdec_wait_ns_ = 0;
    process_ns_ = 0;
    release_ns_ = 0;
    last_image_timestamp_ns_ = 0;
    image_timestamp_regression_count_ = 0;
    metadata_timestamp_rejection_count_ = 0;
    clock_.reset();
    capture_stats_start_ = std::chrono::steady_clock::now();
    capture_stats_last_report_ = capture_stats_start_;
}

void CameraMppPublisher::report_capture_stats(const std::chrono::steady_clock::time_point now)
{
    if (now - capture_stats_last_report_ < std::chrono::seconds(2))
    {
        return;
    }

    const double elapsed_s = std::chrono::duration<double>(now - capture_stats_start_).count();
    const double divisor = capture_frames_ == 0 ? 1.0 : static_cast<double>(capture_frames_);
    RCLCPP_INFO(get_logger(),
        "capture frames=%" PRIu64 " failures=%" PRIu64 " fps=%.1f input_frames=%" PRIu64
        " input_fps=%.1f "
        "input_failures=%" PRIu64 " correlation_drops=%" PRIu64 " vdec_wait_avg=%.2fms "
        "process_avg=%.2fms release_avg=%.3fms metadata_reject=%" PRIu64 " imu_metadata=%" PRIu64
        " imu_invalid=%" PRIu64 " imu_duplicate=%" PRIu64 " imu_backward=%" PRIu64
        " imu_written=%" PRIu64 " imu_write_failures=%" PRIu64,
        capture_frames_, capture_failures_,
        elapsed_s > 0.0 ? static_cast<double>(capture_frames_) / elapsed_s : 0.0,
        compressed_capture_frames_.load(),
        elapsed_s > 0.0 ? static_cast<double>(compressed_capture_frames_.load()) / elapsed_s : 0.0,
        compressed_capture_failures_.load(), compressed_correlation_drops_.load(),
        static_cast<double>(vdec_wait_ns_) / divisor / 1.0e6,
        static_cast<double>(process_ns_) / divisor / 1.0e6,
        static_cast<double>(release_ns_) / divisor / 1.0e6, metadata_timestamp_rejection_count_,
        imu_metadata_sample_count_, imu_invalid_sample_count_, imu_duplicate_count_,
        imu_backward_count_, imu_raw_write_successes_, imu_raw_write_failures_);
    capture_stats_last_report_ = now;
}

bool CameraMppPublisher::crop_eyes(
    const VideoFrameInfo& source, VideoFrameInfo& first, VideoFrameInfo& second)
{
    V2DHandle job = 0;
    S32 result = V2D_BeginJob(&job);
    if (result != 0)
    {
        RCLCPP_ERROR(get_logger(), "V2D_BeginJob failed: %d", result);
        return false;
    }
    V2DArea first_source{static_cast<U16>(config_.left.crop.x),
        static_cast<U16>(config_.left.crop.y), static_cast<U16>(config_.left.crop.width),
        static_cast<U16>(config_.left.crop.height)};
    V2DArea second_source{static_cast<U16>(config_.right.crop.x),
        static_cast<U16>(config_.right.crop.y), static_cast<U16>(config_.right.crop.width),
        static_cast<U16>(config_.right.crop.height)};
    V2DArea first_destination{0, 0, static_cast<U16>(config_.left.output_width),
        static_cast<U16>(config_.left.output_height)};
    V2DArea second_destination{0, 0, static_cast<U16>(config_.right.output_width),
        static_cast<U16>(config_.right.output_height)};
    result = V2D_AddBitblitTask(
        job, &source, &first_source, &first, &first_destination, V2D_CSC_MODE_BUTT);
    if (result != 0)
    {
        RCLCPP_ERROR(get_logger(), "first V2D_AddBitblitTask failed: %d", result);
        (void)V2D_CancelJob(job);
        return false;
    }
    result = V2D_AddBitblitTask(
        job, &source, &second_source, &second, &second_destination, V2D_CSC_MODE_BUTT);
    if (result != 0)
    {
        RCLCPP_ERROR(get_logger(), "second V2D_AddBitblitTask failed: %d", result);
        (void)V2D_CancelJob(job);
        return false;
    }
    result = V2D_EndJob(job);
    if (result != 0)
    {
        RCLCPP_ERROR(get_logger(), "V2D_EndJob failed: %d", result);
        return false;
    }
    return true;
}

void CameraMppPublisher::release_oldest_export()
{
    const auto pair = exports_.front();
    exports_.pop_front();
    if (pair.left_handle)
        (void)VB_Unexport(pair.left_handle);
    if (pair.right_handle)
        (void)VB_Unexport(pair.right_handle);
    if (pair.full_handle != 0)
    {
        (void)VB_Unexport(pair.full_handle);
    }
}

} // namespace camera_io
