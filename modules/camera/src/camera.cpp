// Native V4L2/MPP capture with shared DMA-BUF frame ownership.

#include "spacemit_camera/camera.h"
#include "imu_boundary_deduplication.h"
#include "mpp_runtime.h"
#include "timestamp_unwrap.h"

#include <mutex>
#include <stdexcept>
#include <utility>

#ifdef CAMERA_SDK_WITH_MPP
#include "mpp_decoder_input_pool.h"
#include <linux/dma-buf.h>
#include <sys/ioctl.h>

#include <array>
#include <cstdio>
#include <cstring>

#include "sys_api.h"
#include "uvc_api.h"
#include "vb_api.h"
#include "vdec/vdec_api.h"
#endif

namespace camera_sdk
{
namespace detail
{

#ifdef CAMERA_SDK_WITH_MPP
namespace
{

class MppRuntime
{
public:
    static void acquire()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (references_++ != 0)
            return;
        if (SYS_Init() != 0)
            fail("SYS_Init failed");
        sys_initialized_ = true;
        if (VB_Init() != 0)
            fail("VB_Init failed");
        vb_initialized_ = true;
        if (UVC_Init() != 0)
            fail("UVC_Init failed");
        uvc_initialized_ = true;
        if (VDEC_Init() != 0)
            fail("VDEC_Init failed");
        vdec_initialized_ = true;
    }

    static void release() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (references_ == 0 || --references_ != 0)
            return;
        if (vdec_initialized_)
            VDEC_Exit();
        if (uvc_initialized_)
            UVC_Exit();
        if (vb_initialized_)
            VB_Exit();
        if (sys_initialized_)
            SYS_Exit();
        vdec_initialized_ = uvc_initialized_ = vb_initialized_ = sys_initialized_ = false;
    }

private:
    [[noreturn]] static void fail(const char* message)
    {
        if (vdec_initialized_)
            VDEC_Exit();
        if (uvc_initialized_)
            UVC_Exit();
        if (vb_initialized_)
            VB_Exit();
        if (sys_initialized_)
            SYS_Exit();
        vdec_initialized_ = uvc_initialized_ = vb_initialized_ = sys_initialized_ = false;
        references_ = 0;
        throw std::runtime_error(message);
    }

    static std::mutex mutex_;
    static unsigned references_;
    static bool sys_initialized_;
    static bool vb_initialized_;
    static bool uvc_initialized_;
    static bool vdec_initialized_;
};

std::mutex MppRuntime::mutex_;
unsigned MppRuntime::references_ = 0;
bool MppRuntime::sys_initialized_ = false;
bool MppRuntime::vb_initialized_ = false;
bool MppRuntime::uvc_initialized_ = false;
bool MppRuntime::vdec_initialized_ = false;

void dma_sync(int fd, std::uint64_t flags)
{
    if (fd < 0)
        return;
    dma_buf_sync sync{};
    sync.flags = flags;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) != 0)
    {
        throw std::runtime_error("DMA_BUF_IOCTL_SYNC failed");
    }
}

} // namespace
void acquire_mpp_runtime() { MppRuntime::acquire(); }
void release_mpp_runtime() noexcept { MppRuntime::release(); }
#endif

struct CameraState
{
    explicit CameraState(CameraConfig value) : config(std::move(value))
    {
#ifdef CAMERA_SDK_WITH_MPP
        validate_decoder_input_pool(config.decoder_input_pool);
        if (config.decoder_input_pool.enabled && config.capture_format != CaptureFormat::mjpeg)
        {
            throw std::invalid_argument("decoder input pool requires MJPEG capture");
        }
        if (config.width <= config.metadata_width || config.height < 763)
        {
            throw std::invalid_argument("camera geometry cannot contain Nori metadata area");
        }
        MppRuntime::acquire();
        runtime_acquired = true;
        try
        {
            UvcDevAttr device{};
            std::snprintf(device.acDevNode, sizeof(device.acDevNode), "%s", config.device.c_str());
            if (UVC_CreateDev(config.mpp_device_id, &device) != 0)
            {
                throw std::runtime_error("UVC_CreateDev failed");
            }
            device_created = true;
            if (UVC_EnableDev(config.mpp_device_id) != 0)
            {
                throw std::runtime_error("UVC_EnableDev failed");
            }
            device_enabled = true;

            UvcChnAttr channel{};
            channel.u32Width = config.width;
            channel.u32Height = config.height;
            channel.ePixelFormat = config.capture_format == CaptureFormat::mjpeg
                ? MPP_PIXEL_FORMAT_MJPEG
                : MPP_PIXEL_FORMAT_YUYV;
            channel.u32Fps = config.fps;
            channel.u32Depth =
                config.capture_format == CaptureFormat::mjpeg ? 0 : config.queue_depth;
            if (UVC_SetChnAttr(config.mpp_device_id, 0, &channel) != 0)
            {
                throw std::runtime_error("UVC_SetChnAttr failed");
            }
            if (config.capture_format == CaptureFormat::mjpeg)
            {
                VdecChnAttr decoder{};
                decoder.eCodecType = MPP_STREAM_CODEC_MJPEG;
                decoder.eOutputPixelFormat = MPP_PIXEL_FORMAT_NV12;
                decoder.u32Width = config.width;
                decoder.u32Height = config.height;
                decoder.u32Align = 16;
                set_decoder_input_mode(decoder, config.decoder_input_pool);
                if (VDEC_CreateChn(config.mpp_device_id, &decoder) != 0)
                {
                    throw std::runtime_error("VDEC_CreateChn failed");
                }
                decoder_created = true;
                if (VDEC_EnableChn(config.mpp_device_id) != 0)
                {
                    throw std::runtime_error("VDEC_EnableChn failed");
                }
                decoder_enabled = true;

                source_node = {MPP_ID_UVC, config.mpp_device_id, 0};
                decoder_node = {MPP_ID_VDEC, 0, config.mpp_device_id};
                if (SYS_Bind(&source_node, &decoder_node) != 0)
                {
                    throw std::runtime_error("SYS_Bind UVC to VDEC failed");
                }
                decoder_bound = true;
                if (configure_decoder_input_pool(
                        source_node, decoder_node, config.decoder_input_pool) != 0)
                {
                    throw std::runtime_error("SYS_ConfigStreamDmaBufPool failed");
                }
            }
            // Bind and allocate before UVC's producer thread can send a packet.
            if (UVC_EnableChn(config.mpp_device_id, 0) != 0)
            {
                throw std::runtime_error("UVC_EnableChn failed");
            }
            channel_enabled = true;
        }
        catch (...)
        {
            close();
            throw;
        }
#else
        throw std::runtime_error("camera_sdk was built without CAMERA_SDK_WITH_MPP=ON");
#endif
    }

    ~CameraState() { close(); }

    void close() noexcept
    {
#ifdef CAMERA_SDK_WITH_MPP
        if (channel_enabled)
            UVC_DisableChn(config.mpp_device_id, 0);
        if (decoder_enabled)
            VDEC_DisableChn(config.mpp_device_id);
        if (decoder_bound)
            SYS_UnBind(&source_node, &decoder_node);
        if (decoder_created)
            VDEC_DestroyChn(config.mpp_device_id);
        if (device_enabled)
            UVC_DisableDev(config.mpp_device_id);
        if (device_created)
            UVC_DestroyDev(config.mpp_device_id);
        decoder_bound = decoder_enabled = decoder_created = false;
        channel_enabled = device_enabled = device_created = false;
        if (runtime_acquired)
            MppRuntime::release();
        runtime_acquired = false;
#endif
    }

    CameraConfig config;
    std::mutex get_mutex;
    std::uint64_t last_frame_timestamp = 0;
    bool have_timestamp = false;
    ImuBoundaryDeduplicationState imu_boundary_deduplication;
#ifdef CAMERA_SDK_WITH_MPP
    bool runtime_acquired = false;
    bool device_created = false;
    bool device_enabled = false;
    bool channel_enabled = false;
    bool decoder_created = false;
    bool decoder_enabled = false;
    bool decoder_bound = false;
    MppNode source_node{};
    MppNode decoder_node{};
#endif
};

struct FrameState
{
    ImageView image;
    EmbeddedMetadata metadata;
#ifdef CAMERA_SDK_WITH_MPP
    std::shared_ptr<CameraState> camera;
    VideoFrameInfo mpp_frame{};

    ~FrameState()
    {
        if (!camera)
            return;
        if (camera->config.capture_format == CaptureFormat::mjpeg)
        {
            VDEC_ReleaseFrame(camera->config.mpp_device_id, mpp_frame.ulBufferId);
        }
        else
        {
            UVC_ReleaseFrame(camera->config.mpp_device_id, 0, &mpp_frame);
        }
    }
#endif
};

} // namespace detail

Frame::Frame() noexcept = default;
Frame::~Frame() = default;
Frame::Frame(Frame&&) noexcept = default;
Frame& Frame::operator=(Frame&&) noexcept = default;
Frame::Frame(std::unique_ptr<detail::FrameState> state) noexcept : state_(std::move(state)) {}
Frame::operator bool() const noexcept { return static_cast<bool>(state_); }

const ImageView& Frame::image() const
{
    if (!state_)
        throw std::logic_error("empty frame");
    return state_->image;
}

const EmbeddedMetadata& Frame::metadata() const
{
    if (!state_)
        throw std::logic_error("empty frame");
    return state_->metadata;
}

Camera::Camera(CameraConfig config)
    : state_(std::make_shared<detail::CameraState>(std::move(config)))
{
}
Camera::~Camera() = default;
Camera::Camera(Camera&&) noexcept = default;
Camera& Camera::operator=(Camera&&) noexcept = default;

Frame Camera::get_frame(int timeout_ms)
{
    if (!state_)
        throw std::logic_error("empty camera");
#ifdef CAMERA_SDK_WITH_MPP
    std::lock_guard<std::mutex> lock(state_->get_mutex);
    auto frame = std::make_unique<detail::FrameState>();
    frame->camera = state_;
    const bool decoded_mjpeg = state_->config.capture_format == CaptureFormat::mjpeg;
    const S32 result = decoded_mjpeg
        ? VDEC_GetFrame(state_->config.mpp_device_id, &frame->mpp_frame, timeout_ms)
        : UVC_GetFrame(state_->config.mpp_device_id, 0, &frame->mpp_frame, timeout_ms);
    if (result != 0)
    {
        frame->camera.reset();
        throw std::runtime_error(
            std::string(decoded_mjpeg ? "VDEC_GetFrame failed: " : "UVC_GetFrame failed: ") +
            std::to_string(result));
    }

    auto& mpp = frame->mpp_frame;
    int dma_fd = -1;
    if (VB_GetDmaBufFd(mpp.ulBufferId, &dma_fd) != 0 || dma_fd < 0)
    {
        dma_fd = static_cast<int>(mpp.stVFrame.u32Fd[0]);
    }
    void* address = reinterpret_cast<void*>(mpp.stVFrame.ulPlaneVirAddr[0]);
    if (!address && VB_GetVirAddr(mpp.ulBufferId, &address) != 0)
    {
        throw std::runtime_error("VB_GetVirAddr failed");
    }
    const std::size_t stride = mpp.stVFrame.u32PlaneStride[0]
        ? mpp.stVFrame.u32PlaneStride[0]
        : static_cast<std::size_t>(state_->config.width) * (decoded_mjpeg ? 1 : 2);

    frame->image.dma_fd = dma_fd;
    frame->image.data = static_cast<const std::uint8_t*>(address);
    frame->image.size = mpp.stVFrame.u32TotalSize;
    frame->image.width = state_->config.width;
    frame->image.height = state_->config.height;
    frame->image.stride = stride;
    frame->image.format = decoded_mjpeg ? PixelFormat::nv12 : PixelFormat::yuyv422;
    frame->image.metadata_x = 0;
    frame->image.stereo_x = state_->config.metadata_width;
    frame->image.stereo_width = state_->config.width - state_->config.metadata_width;

    std::string decode_error;
    detail::dma_sync(dma_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    const LumaView metadata_image{frame->image.data, frame->image.width, frame->image.height,
        stride, decoded_mjpeg ? 1U : 2U};
    const bool decoded = decode_embedded_metadata(metadata_image, frame->metadata, &decode_error);
    detail::dma_sync(dma_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
    if (decoded &&
        detail::unwrap_metadata_timestamps(
            frame->metadata, state_->last_frame_timestamp, state_->have_timestamp))
    {
        detail::remove_repeated_boundary_imu(frame->metadata, state_->imu_boundary_deduplication);
        state_->last_frame_timestamp = frame->metadata.exposure_end_us;
        state_->have_timestamp = true;
    }
    else
    {
        frame->metadata = {};
    }
    return Frame(std::move(frame));
#else
    (void)timeout_ms;
    throw std::runtime_error("spacemit_camera was built without CAMERA_SDK_WITH_MPP=ON");
#endif
}

const CameraConfig& Camera::config() const
{
    if (!state_)
        throw std::logic_error("empty camera");
    return state_->config;
}

} // namespace camera_sdk
