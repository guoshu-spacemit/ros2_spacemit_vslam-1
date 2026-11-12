// Extend exposure-clock counters and align timestamps to the caller's epoch.

#include "spacemit_camera/camera_clock.h"

#include <limits>

namespace camera_sdk
{
namespace
{

constexpr std::int32_t kMaxExposureDurationUs = 1000000;

} // namespace

void CameraClock::reset()
{
    initialized_ = false;
    anchor_raw_us_ = 0;
    anchor_extended_us_ = 0;
    epoch_offset_ns_ = 0;
}

bool CameraClock::preview(std::uint32_t exposure_start_raw_us, std::uint32_t exposure_end_raw_us,
    std::int64_t ros_now_ns, FrameTimestamps* timestamps, std::string* error) const
{
    if (timestamps == nullptr)
    {
        if (error != nullptr)
        {
            *error = "timestamp output is null";
        }
        return false;
    }
    if (ros_now_ns <= 0)
    {
        if (error != nullptr)
        {
            *error = "ROS clock is not positive";
        }
        return false;
    }

    const std::int32_t exposure_duration_us =
        signed_delta(exposure_end_raw_us, exposure_start_raw_us);
    if (exposure_duration_us <= 0 || exposure_duration_us > kMaxExposureDurationUs)
    {
        if (error != nullptr)
        {
            *error = "invalid exposure duration";
        }
        return false;
    }

    FrameTimestamps candidate;
    candidate.exposure_end_raw_us = exposure_end_raw_us;
    if (!initialized_)
    {
        candidate.exposure_end_extended_us = static_cast<std::int64_t>(exposure_end_raw_us);
        candidate.epoch_offset_ns = ros_now_ns - candidate.exposure_end_extended_us * 1000LL;
    }
    else
    {
        const std::int32_t frame_delta_us = signed_delta(exposure_end_raw_us, anchor_raw_us_);
        if (frame_delta_us <= 0)
        {
            if (error != nullptr)
            {
                *error = "exposure-end timestamp regressed";
            }
            return false;
        }
        candidate.exposure_end_extended_us =
            anchor_extended_us_ + static_cast<std::int64_t>(frame_delta_us);
        candidate.epoch_offset_ns = epoch_offset_ns_;
    }

    const std::int64_t exposure_start_extended_us =
        candidate.exposure_end_extended_us - exposure_duration_us;
    candidate.exposure_start_ns = candidate.epoch_offset_ns + exposure_start_extended_us * 1000LL;
    candidate.exposure_end_ns =
        candidate.epoch_offset_ns + candidate.exposure_end_extended_us * 1000LL;
    if (candidate.exposure_start_ns <= 0 ||
        candidate.exposure_end_ns <= candidate.exposure_start_ns)
    {
        if (error != nullptr)
        {
            *error = "invalid mapped exposure timestamps";
        }
        return false;
    }

    *timestamps = candidate;
    return true;
}

bool CameraClock::preview_recorded(std::uint32_t exposure_start_raw_us,
    std::uint32_t exposure_end_raw_us, std::int64_t recorded_image_ns, ExposureReference reference,
    FrameTimestamps* timestamps, std::string* error) const
{
    const auto duration_us = signed_delta(exposure_end_raw_us, exposure_start_raw_us);
    if (duration_us <= 0 || duration_us > kMaxExposureDurationUs)
    {
        if (error != nullptr)
        {
            *error = "invalid exposure duration";
        }
        return false;
    }
    const auto duration_ns = static_cast<std::int64_t>(duration_us) * 1000LL;
    const auto end_offset_ns =
        reference == ExposureReference::Midpoint ? duration_ns - duration_ns / 2 : duration_ns;
    if (recorded_image_ns <= 0 ||
        recorded_image_ns > std::numeric_limits<std::int64_t>::max() - end_offset_ns)
    {
        if (error != nullptr)
        {
            *error = "invalid recorded image timestamp";
        }
        return false;
    }
    FrameTimestamps candidate;
    if (!preview(exposure_start_raw_us, exposure_end_raw_us, recorded_image_ns + end_offset_ns,
            &candidate, error))
    {
        return false;
    }
    const auto mapped_image_ns = reference == ExposureReference::Midpoint
        ? candidate.exposure_midpoint_ns()
        : candidate.exposure_start_ns;
    if (mapped_image_ns != recorded_image_ns)
    {
        if (error != nullptr)
        {
            *error = "recorded exposure timestamp disagrees with embedded clock";
        }
        return false;
    }
    if (timestamps == nullptr)
    {
        if (error != nullptr)
        {
            *error = "timestamp output is null";
        }
        return false;
    }
    *timestamps = candidate;
    return true;
}

void CameraClock::commit(const FrameTimestamps& timestamps)
{
    anchor_raw_us_ = timestamps.exposure_end_raw_us;
    anchor_extended_us_ = timestamps.exposure_end_extended_us;
    epoch_offset_ns_ = timestamps.epoch_offset_ns;
    initialized_ = true;
}

std::int64_t CameraClock::ros_time_ns(std::uint32_t raw_us) const
{
    if (!initialized_)
    {
        return 0;
    }
    const std::int64_t extended_us = anchor_extended_us_ + signed_delta(raw_us, anchor_raw_us_);
    return epoch_offset_ns_ + extended_us * 1000LL;
}

bool CameraClock::initialized() const { return initialized_; }

std::int32_t CameraClock::signed_delta(std::uint32_t value, std::uint32_t reference)
{
    return static_cast<std::int32_t>(value - reference);
}

} // namespace camera_sdk
