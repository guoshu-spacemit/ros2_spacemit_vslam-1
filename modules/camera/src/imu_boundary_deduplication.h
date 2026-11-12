// Internal native camera support for imu boundary deduplication.
#ifndef IMU_BOUNDARY_DEDUPLICATION_H
#define IMU_BOUNDARY_DEDUPLICATION_H

#include "spacemit_camera/embedded_metadata.h"

#include <cstdint>

namespace camera_sdk::detail
{

struct ImuBoundaryDeduplicationState
{
    std::uint64_t last_timestamp_us{0};
    bool has_last_timestamp{false};
};

// The camera repeats the final valid IMU sample of one image interval as the
// first valid sample of the next interval.  Remove only that boundary sample;
// a non-contiguous frame is preserved in full.
void remove_repeated_boundary_imu(EmbeddedMetadata& metadata, ImuBoundaryDeduplicationState& state);

} // namespace camera_sdk::detail

#endif // IMU_BOUNDARY_DEDUPLICATION_H
