// Extend one hardware clock for exposure and IMU metadata.

#ifndef TIMESTAMP_UNWRAP_H
#define TIMESTAMP_UNWRAP_H

#include "spacemit_camera/embedded_metadata.h"

#include <array>
#include <limits>
#include <optional>

namespace camera_sdk::detail
{

inline constexpr std::uint64_t kTimestampWrapUs = std::uint64_t{1} << 32;

inline std::optional<std::uint64_t> unwrap_near(std::uint64_t reference, std::uint32_t raw) noexcept
{
    constexpr std::uint64_t half = kTimestampWrapUs >> 1;
    std::uint64_t value = (reference & ~(kTimestampWrapUs - 1)) | raw;
    if (value > reference && value - reference > half)
    {
        if (value < kTimestampWrapUs)
        {
            return std::nullopt;
        }
        value -= kTimestampWrapUs;
    }
    else if (reference > value && reference - value > half)
    {
        if (value > std::numeric_limits<std::uint64_t>::max() - kTimestampWrapUs)
        {
            return std::nullopt;
        }
        value += kTimestampWrapUs;
    }
    return value;
}

// Validate the whole frame before writing any extended timestamps.
inline bool unwrap_metadata_timestamps(
    EmbeddedMetadata& metadata, std::uint64_t last_timestamp, bool have_timestamp) noexcept
{
    // Reserve an earlier cycle for samples captured just before the first frame's wrap.
    const auto exposure_end = have_timestamp
        ? unwrap_near(last_timestamp, metadata.exposure_end_raw_us)
        : std::optional<std::uint64_t>{kTimestampWrapUs + metadata.exposure_end_raw_us};
    if (!exposure_end || (have_timestamp && *exposure_end <= last_timestamp) ||
        metadata.imu_count > metadata.imu.size())
    {
        return false;
    }
    const auto exposure_start = unwrap_near(*exposure_end, metadata.exposure_start_raw_us);
    if (!exposure_start || *exposure_start > *exposure_end)
    {
        return false;
    }

    std::array<std::uint64_t, kMaxImuSamples> imu_timestamps{};
    for (std::size_t index = 0; index < metadata.imu_count; ++index)
    {
        const auto timestamp = unwrap_near(*exposure_end, metadata.imu[index].timestamp_raw_us);
        if (!timestamp)
        {
            return false;
        }
        imu_timestamps[index] = *timestamp;
    }
    metadata.exposure_end_us = *exposure_end;
    metadata.exposure_start_us = *exposure_start;
    for (std::size_t index = 0; index < metadata.imu_count; ++index)
    {
        metadata.imu[index].timestamp_us = imu_timestamps[index];
    }
    return true;
}

} // namespace camera_sdk::detail

#endif // TIMESTAMP_UNWRAP_H
