#ifndef TRACKING_QUALITY_STATUS_H
#define TRACKING_QUALITY_STATUS_H
#include <cstdint>
#include <limits>

namespace esdf_drm
{
// Same-frame functional quality only. Each optional observation has availability.
struct TrackingQualityStatus
{
    bool available = false;
    bool pose_valid = false;
    bool prediction_available = false;
    bool prediction_active = false;
    std::uint32_t prediction_streak = 0;
    float prediction_span_ms = 0.0F;
    bool reprojection_available = false;
    float reprojection_rmse_px = std::numeric_limits<float>::quiet_NaN();
};
} // namespace esdf_drm
#endif // TRACKING_QUALITY_STATUS_H
