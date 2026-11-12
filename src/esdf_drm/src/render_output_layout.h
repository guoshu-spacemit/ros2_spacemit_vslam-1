#ifndef RENDER_OUTPUT_LAYOUT_H
#define RENDER_OUTPUT_LAYOUT_H
#include <cstdint>
#include <stdexcept>

#include <sys/vb_api.h>

namespace esdf_drm
{

// Geometry only: no allocation or hardware access. Pool size and exported
// metadata must use the same calculation; capacity is not visible image size.
inline VideoFrameInfo make_render_output_layout(int width, int height, bool bgra)
{
    if (width <= 0 || height <= 0 || width > 16384 || height > 16384 || (width & 1) != 0 ||
        (height & 1) != 0 || (bgra && (width & 15) != 0))
    {
        throw std::invalid_argument(
            "render output needs even dimensions in 1..16384; BGRA width must align to 16");
    }
    VideoFrameInfo frame{};
    frame.eFrameType = FRAME_TYPE_COMMON;
    frame.eModId = MPP_ID_V2D;
    frame.stCommFrameInfo.u32Width = static_cast<U32>(width);
    frame.stCommFrameInfo.u32Height = static_cast<U32>(height);
    frame.stCommFrameInfo.u32Align = 1;
    frame.stCommFrameInfo.ePixelFormat = bgra ? MPP_PIXEL_FORMAT_BGRA : MPP_PIXEL_FORMAT_NV12;
    frame.stCommFrameInfo.eCompressMode = COMPRESS_MODE_NONE;
    auto& planes = frame.stVFrame;
    planes.u32PlaneNum = bgra ? 1U : 2U;
    const auto rows = static_cast<U32>(height);
    const auto stride = static_cast<U32>(width) * (bgra ? 4U : 1U);
    planes.u32PlaneStride[0] = stride;
    planes.u32PlaneSize[0] = stride * (bgra ? ((rows + 15U) & ~15U) : rows);
    planes.u32PlaneSizeValid[0] = stride * rows;
    planes.u32TotalSize = planes.u32PlaneSize[0];
    if (!bgra)
    {
        planes.u32PlaneStride[1] = stride;
        planes.u32PlaneSize[1] = stride * rows / 2U;
        planes.u32PlaneSizeValid[1] = planes.u32PlaneSize[1];
        planes.u32TotalSize += planes.u32PlaneSize[1];
    }
    return frame;
}

} // namespace esdf_drm

#endif // RENDER_OUTPUT_LAYOUT_H
