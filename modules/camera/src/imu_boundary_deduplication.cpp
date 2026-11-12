// Remove repeated or non-monotonic IMU samples at frame boundaries.

#include "imu_boundary_deduplication.h"

namespace camera_sdk::detail
{

void remove_repeated_boundary_imu(EmbeddedMetadata& metadata, ImuBoundaryDeduplicationState& state)
{
    if (state.has_last_timestamp)
    {
        for (std::size_t index = 0; index < metadata.imu_count; ++index)
        {
            const ImuSample& sample = metadata.imu[index];
            if (!sample.valid)
            {
                continue;
            }
            if (sample.timestamp_us == state.last_timestamp_us)
            {
                for (std::size_t next = index + 1; next < metadata.imu_count; ++next)
                {
                    metadata.imu[next - 1] = metadata.imu[next];
                }
                --metadata.imu_count;
            }
            break;
        }
    }

    for (std::size_t index = 0; index < metadata.imu_count; ++index)
    {
        const ImuSample& sample = metadata.imu[index];
        if (!sample.valid)
        {
            continue;
        }
        state.last_timestamp_us = sample.timestamp_us;
        state.has_last_timestamp = true;
    }
}

} // namespace camera_sdk::detail
