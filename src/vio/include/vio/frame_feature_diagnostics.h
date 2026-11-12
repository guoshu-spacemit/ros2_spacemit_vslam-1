#ifndef FRAME_FEATURE_DIAGNOSTICS_H
#define FRAME_FEATURE_DIAGNOSTICS_H
#include <spacemit_vslam.h>
#include <common/msg/feature_point_array.hpp>

#include "vio/feature_track_history.h"
#include "vio/prediction_history.h"

#include <cmath>
#include <limits>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace vio
{
namespace localization = spacemit_vslam::localization;
// Application display decoration of a completed functional tracking result.
class FrameFeatureDiagnostics
{
public:
    void reset()
    {
        feature_track_history_.reset();
        prediction_history_.reset();
    }

    void populate(common::msg::FeaturePointArray& message, std::int64_t timestamp_ns,
        const std::vector<localization::FeatureObservation>& observations,
        const localization::FrameQuality& diagnostics, bool pose_valid)
    {
        // Invalid poses can include an internal reset. Do not inherit recycled IDs.
        if (!pose_valid)
            feature_track_history_.reset();
        feature_track_history_.begin_frame(timestamp_ns);
        message.diagnostics_available = diagnostics.available;
        message.pose_valid = pose_valid;
        const double prediction = metric_value(diagnostics, "tracking.predicted_pose");
        message.prediction_available = std::isfinite(prediction);
        message.prediction_active = pose_valid && message.prediction_available && prediction != 0.0;
        prediction_history_.update(timestamp_ns, message.prediction_active);
        message.prediction_streak = prediction_history_.count();
        message.prediction_span_ms = prediction_history_.span_ms();
        const auto residual = float_metric(diagnostics, "tracking.reprojection_rmse_px");
        message.reprojection_available = std::isfinite(residual) && residual >= 0.0F;
        message.reprojection_rmse_px =
            message.reprojection_available ? residual : std::numeric_limits<float>::quiet_NaN();
        std::unordered_set<std::uint64_t> pose_input_ids;
        for (const auto& selected : diagnostics.pose_inputs)
        {
            if (selected.sensor_id == 0 && std::isfinite(selected.pixel[0]) &&
                std::isfinite(selected.pixel[1]))
                pose_input_ids.insert(selected.track_id);
        }
        const double reported_inputs = metric_value(diagnostics, "tracking.pose_input_count");
        message.pose_inputs_available =
            !pose_input_ids.empty() || (std::isfinite(reported_inputs) && reported_inputs >= 0.0);
        message.pose_input_count = static_cast<std::uint32_t>(pose_input_ids.size());
        message.points.reserve(observations.size());
        for (const auto& observation : observations)
        {
            if (observation.sensor_id != 0 || !std::isfinite(observation.pixel[0]) ||
                !std::isfinite(observation.pixel[1]))
            {
                continue;
            }
            common::msg::FeaturePoint point;
            point.id = observation.track_id;
            point.x = observation.pixel[0];
            point.y = observation.pixel[1];
            point.pose_input = pose_input_ids.count(observation.track_id) != 0;
            const auto history = feature_track_history_.observe(observation.track_id);
            point.track_age_seconds = history.age_seconds;
            point.track_observation_count = history.count;
            message.points.push_back(point);
        }
        message.tracked_feature_count = static_cast<std::uint32_t>(message.points.size());
        // An invalid frame may expose a backend's last observations.
        // They may be displayed, but must not age recycled IDs in the next frame.
        if (!pose_valid)
            feature_track_history_.reset();
    }

private:
    static double metric_value(const localization::FrameQuality& diagnostics, std::string_view name)
    {
        for (const auto& metric : diagnostics.metrics)
            if (metric.name == name)
                return metric.value;
        return std::numeric_limits<double>::quiet_NaN();
    }

    static float float_metric(const localization::FrameQuality& diagnostics, std::string_view name)
    {
        return static_cast<float>(metric_value(diagnostics, name));
    }

    FeatureTrackHistory feature_track_history_;
    PredictionHistory prediction_history_;
};
} // namespace vio

#endif // FRAME_FEATURE_DIAGNOSTICS_H
