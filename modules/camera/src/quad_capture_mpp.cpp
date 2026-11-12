// MPP capture and decode for synchronized four-view frames.

#include "mpp_decoder_input_pool.h"
#include "mpp_runtime.h"
#include "quad_capture.h"

#include "sys_api.h"
#include "uvc_api.h"
#include "vb_api.h"
#include "vdec/vdec_api.h"

#include <cstdio>
#include <mutex>
#include <stdexcept>

namespace camera_sdk::detail
{
namespace
{
void check(int result, const char* operation)
{
    if (result != 0)
        throw std::runtime_error(std::string(operation) + ": " + std::to_string(result));
}
class MppQuadCapture;
struct MppLease
{
    std::shared_ptr<MppQuadCapture> capture;
    VideoFrameInfo frame{};
    bool acquired = false;
    ~MppLease();
};

class MppQuadCapture : public QuadCapture, public std::enable_shared_from_this<MppQuadCapture>
{
public:
    MppQuadCapture(QuadCameraConfig config, const std::array<QuadView, 4>& views)
        : QuadCapture(std::move(config), views)
    {
        try
        {
            open();
        }
        catch (...)
        {
            close();
            throw;
        }
    }
    ~MppQuadCapture() override { close(); }
    void release(UL buffer) noexcept
    {
        if (VDEC_ReleaseFrame(config_.mpp_device_id, buffer) != 0)
            release_error();
    }
    std::unique_ptr<QuadFrameState> get_frame(int timeout_ms) override
    {
        std::lock_guard<std::mutex> lock(get_mutex_);
        auto frame = std::make_unique<QuadFrameState>();
        auto lease = std::make_shared<MppLease>();
        lease->capture = shared_from_this();
        frame->lease = lease;
        int result = VDEC_GetFrame(config_.mpp_device_id, &lease->frame, timeout_ms);
        if (result == ERR_VDEC_TIMEOUT || result == ERR_VDEC_NO_FRAME)
            return {};
        check(result, "quad VDEC_GetFrame");
        lease->acquired = true;
        for (std::uint32_t i = 1; i < config_.queue_depth; ++i)
        {
            VideoFrameInfo newer{};
            result = VDEC_GetFrame(config_.mpp_device_id, &newer, 0);
            if (result == ERR_VDEC_TIMEOUT || result == ERR_VDEC_NO_FRAME)
                break;
            check(result, "quad VDEC_GetFrame drain");
            release(lease->frame.ulBufferId);
            lease->frame = newer;
            ++stale_;
        }
        const auto& mpp = lease->frame.stVFrame;
        const auto& geometry = lease->frame.stVdecFrameInfo.stCommFrameInfo;
        if (geometry.u32Width != config_.width || geometry.u32Height != config_.height ||
            geometry.ePixelFormat != MPP_PIXEL_FORMAT_NV12 || mpp.u32PlaneNum < 1 ||
            mpp.u32PlaneNum > 2 || mpp.u32TotalSize == 0)
        {
            ++invalid_;
            throw std::runtime_error("quad decoder returned unexpected NV12 geometry");
        }
        int fd = -1;
        void* base = nullptr;
        check(VB_GetDmaBufFd(lease->frame.ulBufferId, &fd), "quad VB_GetDmaBufFd");
        check(VB_GetVirAddr(lease->frame.ulBufferId, &base), "quad VB_GetVirAddr");
        if (fd < 0 || !base)
            throw std::runtime_error("quad decoded buffer has no mapping/fd");
        const std::size_t y_stride = mpp.u32PlaneStride[0];
        const std::size_t uv_stride = mpp.u32PlaneNum == 2 ? mpp.u32PlaneStride[1] : y_stride;
        const std::size_t y_size = y_stride * config_.height;
        const std::size_t uv_size = uv_stride * (config_.height / 2);
        std::size_t uv_offset = mpp.u32PlaneNum == 2 ? mpp.u32PlaneSize[0] : y_size;
        const auto base_address = reinterpret_cast<std::uintptr_t>(base);
        if (mpp.ulPlaneVirAddr[0] && mpp.ulPlaneVirAddr[0] != base_address)
            throw std::runtime_error("quad NV12 Y plane has unexpected base offset");
        if (mpp.u32PlaneNum == 2 && mpp.ulPlaneVirAddr[1])
        {
            if (mpp.ulPlaneVirAddr[1] < base_address)
                throw std::runtime_error("quad NV12 planes do not share one buffer");
            uv_offset = mpp.ulPlaneVirAddr[1] - base_address;
        }
        if (y_stride < config_.width || uv_stride < config_.width || y_size > uv_offset ||
            uv_offset > mpp.u32TotalSize || uv_size > mpp.u32TotalSize - uv_offset)
            throw std::runtime_error("quad NV12 plane layout exceeds its buffer");
        frame->image = {QuadPixelFormat::nv12, static_cast<const std::uint8_t*>(base),
            mpp.u32TotalSize, fd, config_.width, config_.height,
            {{{0, y_stride, y_size}, {uv_offset, uv_stride, uv_size}}}};
        frame->views = views_;
        frame->metadata.frame_index = ++delivered_;
        frame->metadata.source_timestamp_us = mpp.u64PTS;
        frame->metadata.clock = QuadClock::mpp_pts;
        frame->metadata.driver_flags = mpp.u32FrameFlag;
        frame->metadata.host_received_ns = quad_host_now_ns();
        // u32FrameFlag is not treated as a UVC sequence after decoding.
        return frame;
    }

private:
    void open()
    {
        validate_decoder_input_pool(config_.decoder_input_pool);
        acquire_mpp_runtime();
        runtime_ = true;
        UvcDevAttr device{};
        if (config_.device.size() >= sizeof(device.acDevNode))
            throw std::invalid_argument("quad device path exceeds MPP device-node capacity");
        std::snprintf(device.acDevNode, sizeof(device.acDevNode), "%s", config_.device.c_str());
        check(UVC_CreateDev(config_.mpp_device_id, &device), "quad UVC_CreateDev");
        device_created_ = true;
        check(UVC_EnableDev(config_.mpp_device_id), "quad UVC_EnableDev");
        device_enabled_ = true;
        UvcChnAttr channel{};
        channel.u32Width = config_.width;
        channel.u32Height = config_.height;
        channel.u32Fps = config_.fps;
        channel.ePixelFormat = MPP_PIXEL_FORMAT_MJPEG;
        channel.u32Depth = 0;
        check(UVC_SetChnAttr(config_.mpp_device_id, 0, &channel), "quad UVC_SetChnAttr");
        VdecChnAttr decoder{};
        decoder.eCodecType = MPP_STREAM_CODEC_MJPEG;
        decoder.eOutputPixelFormat = MPP_PIXEL_FORMAT_NV12;
        decoder.u32Width = config_.width;
        decoder.u32Height = config_.height;
        decoder.u32BufCnt = config_.queue_depth;
        decoder.u32Align = 16;
        set_decoder_input_mode(decoder, config_.decoder_input_pool);
        check(VDEC_CreateChn(config_.mpp_device_id, &decoder), "quad VDEC_CreateChn");
        decoder_created_ = true;
        check(VDEC_EnableChn(config_.mpp_device_id), "quad VDEC_EnableChn");
        decoder_enabled_ = true;
        source_ = {MPP_ID_UVC, config_.mpp_device_id, 0};
        destination_ = {MPP_ID_VDEC, 0, config_.mpp_device_id};
        check(SYS_Bind(&source_, &destination_), "quad UVC/VDEC bind");
        bound_ = true;
        check(configure_decoder_input_pool(source_, destination_, config_.decoder_input_pool),
            "quad decoder input pool");
        check(UVC_EnableChn(config_.mpp_device_id, 0), "quad UVC_EnableChn");
        channel_enabled_ = true;
    }
    void close() noexcept
    {
        if (channel_enabled_)
            UVC_DisableChn(config_.mpp_device_id, 0);
        if (decoder_enabled_)
            VDEC_DisableChn(config_.mpp_device_id);
        if (bound_)
            SYS_UnBind(&source_, &destination_);
        if (decoder_created_)
            VDEC_DestroyChn(config_.mpp_device_id);
        if (device_enabled_)
            UVC_DisableDev(config_.mpp_device_id);
        if (device_created_)
            UVC_DestroyDev(config_.mpp_device_id);
        if (runtime_)
            release_mpp_runtime();
        bound_ = decoder_enabled_ = decoder_created_ = channel_enabled_ = false;
        device_enabled_ = device_created_ = runtime_ = false;
    }
    bool runtime_ = false, device_created_ = false, device_enabled_ = false;
    bool channel_enabled_ = false, decoder_created_ = false, decoder_enabled_ = false;
    bool bound_ = false;
    MppNode source_{}, destination_{};
    std::mutex get_mutex_;
};

MppLease::~MppLease()
{
    if (acquired)
        capture->release(frame.ulBufferId);
}
} // namespace
std::shared_ptr<QuadCapture> make_mpp_quad_capture(
    QuadCameraConfig config, const std::array<QuadView, 4>& views)
{
    return std::make_shared<MppQuadCapture>(std::move(config), views);
}
} // namespace camera_sdk::detail
