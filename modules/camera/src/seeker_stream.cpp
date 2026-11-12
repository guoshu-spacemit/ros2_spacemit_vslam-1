// Associate Seeker events with bounded image sequences.

#include "spacemit_camera/seeker_stream.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace camera_sdk
{
namespace
{
bool fail(std::string* error, const char* message)
{
    if (error)
        *error = message;
    return false;
}

bool adjust(const SeekerEventHeader& source, double offset, SeekerEventHeader& output)
{
    std::uint64_t timestamp;
    if (!seeker_timestamp_ns(source, timestamp) || !std::isfinite(offset) ||
        offset >= std::ldexp(1.0, 64) || offset <= -std::ldexp(1.0, 64))
        return false;
    // Match the vendor's truncation of the floating offset; keep the integer
    // source timestamp out of floating point to retain its nanosecond precision.
    if (offset >= 0)
    {
        const auto delta = static_cast<std::uint64_t>(offset);
        if (timestamp > std::numeric_limits<std::uint64_t>::max() - delta)
            return false;
        timestamp += delta;
    }
    else
    {
        const auto delta = static_cast<std::uint64_t>(-offset);
        if (timestamp < delta)
            return false;
        timestamp -= delta;
    }
    output = source;
    output.seconds = timestamp / 1000000000;
    output.nanoseconds = timestamp % 1000000000;
    return true;
}
} // namespace

SeekerStream::SeekerStream(SeekerStreamConfig config) : config_(std::move(config))
{
    if (!std::isfinite(config_.clock_alpha) || config_.clock_alpha <= 0 ||
        config_.clock_alpha > 1 || config_.clock_reset_threshold_ns == 0)
        throw std::invalid_argument("invalid Seeker clock filter configuration");
    make_quad_views(2, 8, config_.positions);
}

bool SeekerStream::process_event(const std::uint8_t* data, std::size_t size,
    SeekerEventFrame& output, const SeekerEventHeader* host_reference, std::string* error)
{
    SeekerEventFrame next;
    if (!decode_seeker_event(data, size, next.event, error, config_.envelope_policy))
        return false;
    std::uint64_t device_ns;
    if (!seeker_timestamp_ns(next.event.header, device_ns))
        return fail(error, "invalid Seeker device timestamp");
    std::lock_guard<std::mutex> lock(mutex_);
    auto offset = offset_ns_;
    auto clock_ready = clock_ready_;
    if (next.event.type == SeekerEventType::sensor_custom)
    {
        if (config_.map_host_time && host_reference)
        {
            std::uint64_t host_ns;
            if (!seeker_timestamp_ns(*host_reference, host_ns))
                return fail(error, "invalid Seeker host reference");
            const double measured = host_ns >= device_ns
                ? static_cast<double>(host_ns - device_ns)
                : -static_cast<double>(device_ns - host_ns);
            if (!clock_ready ||
                std::abs(measured - offset) > static_cast<double>(config_.clock_reset_threshold_ns))
                offset = measured;
            // Disable floating-point contraction for this source in CMake:
            // RISC-V FMA must not change official x86 rounding in replay.
            offset = config_.clock_alpha * measured + (1.0 - config_.clock_alpha) * offset;
            clock_ready = true;
        }
        if (!adjust(next.event.header, offset, next.output_header))
            return fail(error, "Seeker mapped timestamp overflow");
        next.output_header.sequence = 1;
        next.time_mapped = clock_ready;
    }
    // The official event callback header is zero for UVC/calibration events;
    // their device headers remain available in next.event.header.
    if (next.event.type == SeekerEventType::uvc_info)
    {
        uvc_header_ = next.event.header;
        uvc_ready_ = true;
    }
    offset_ns_ = offset;
    clock_ready_ = clock_ready;
    output = std::move(next);
    if (error)
        error->clear();
    return true;
}

bool SeekerStream::process_image(
    const QuadImage& image, SeekerImageFrame& output, std::string* error)
{
    SeekerImageFrame next;
    if (image.format != QuadPixelFormat::mjpeg && image.format != QuadPixelFormat::nv12)
        return fail(error, "unsupported Seeker image format");
    if ((image.format == QuadPixelFormat::mjpeg && (!image.data || image.size == 0)) ||
        (image.format == QuadPixelFormat::nv12 && image.dma_fd < 0 && !image.data))
        return fail(error, "missing Seeker image buffer");
    try
    {
        next.views = make_quad_views(image.width, image.height, config_.positions);
    }
    catch (const std::invalid_argument&)
    {
        return fail(error, "invalid Seeker quad image layout");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    next.image = image;
    next.device_header = uvc_header_;
    if (!adjust(uvc_header_, offset_ns_, next.output_header))
        return fail(error, "Seeker image timestamp overflow");
    next.output_header.sequence = image_sequence_ + 1U;
    next.timestamp_valid = uvc_ready_;
    next.time_mapped = clock_ready_;
    image_sequence_ = next.output_header.sequence;
    output = std::move(next);
    if (error)
        error->clear();
    return true;
}

void SeekerStream::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    offset_ns_ = 0;
    clock_ready_ = false;
    uvc_ready_ = false;
    uvc_header_ = {};
    image_sequence_ = 0;
}

} // namespace camera_sdk
