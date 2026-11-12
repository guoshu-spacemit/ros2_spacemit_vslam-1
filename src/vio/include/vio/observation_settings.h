#ifndef OBSERVATION_SETTINGS_H
#define OBSERVATION_SETTINGS_H
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vio
{

// Optional minimum camera-forward observation depth, in metres.
// Zero delegates the threshold to the selected localization implementation.
inline float validate_minimum_observation_depth(double min_depth_m)
{
    if (!std::isfinite(min_depth_m) || min_depth_m < 0.0 ||
        min_depth_m > std::numeric_limits<float>::max())
    {
        throw std::invalid_argument(
            "minimum_observation_depth_m must be finite, non-negative and fit in float");
    }
    if (min_depth_m == 0.0)
    {
        return 0.0F;
    }
    const float threshold = static_cast<float>(min_depth_m);
    if (!(threshold > 0.0F))
    {
        throw std::invalid_argument(
            "minimum_observation_depth_m is too small for the localization threshold");
    }
    return threshold;
}

} // namespace vio

#endif // OBSERVATION_SETTINGS_H
