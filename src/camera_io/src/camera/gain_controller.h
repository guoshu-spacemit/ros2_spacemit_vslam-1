#ifndef GAIN_CONTROLLER_H
#define GAIN_CONTROLLER_H
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

// Baseline USB AGC, not a complete libcamera AGC port. Raw control steps do
// not represent linear gain multipliers. Both time and frame-count guards
// prevent reacting repeatedly to images preceding a control change.
class GainController
{
public:
    GainController(int minimum, int maximum, int step, int initial)
        : minimum_(minimum), maximum_(maximum), step_(step), value_(initial)
    {
        if (step <= 0 || minimum > maximum || initial < minimum || initial > maximum ||
            (std::int64_t(initial) - minimum) % step)
            throw std::invalid_argument("invalid gain limits");
    }

    int update(double mean, double clipped, double target, double time, std::uint64_t frame)
    {
        if (!std::isfinite(mean) || !std::isfinite(clipped) || mean < 0 || mean > 255 ||
            clipped < 0 || clipped > 1 || !std::isfinite(target) || target < 1 || target > 254 ||
            !std::isfinite(time) || time < last_change_time_ || frame < last_change_frame_)
            throw std::invalid_argument("invalid luma statistic or target");
        if (frame - last_change_frame_ < 12 || time - last_change_time_ < 0.25)
            return value_;
        if (!have_filtered_)
            filtered_ = mean;
        else
            filtered_ += 0.25 * (mean - filtered_);
        have_filtered_ = true;
        double error = target - filtered_;
        // Highlight constraint prevents increasing gain into major clipping.
        if (clipped > 0.08)
            error = std::min(error, -5.0);
        if (std::abs(error) <= 4.0)
            return value_;
        int delta = std::clamp(static_cast<int>(std::round(error * 0.20)), -8, 8);
        if (delta == 0)
            delta = error > 0 ? 1 : -1;
        // Always advance at least one device step, including non-unit controls.
        const auto steps =
            std::max<std::int64_t>(1, (std::abs(delta) + std::int64_t(step_) - 1) / step_);
        const auto raw = std::clamp<std::int64_t>(
            std::int64_t(value_) + (delta > 0 ? steps : -steps) * step_, minimum_, maximum_);
        const int quantized = static_cast<int>(minimum_ + ((raw - minimum_) / step_) * step_);
        if (quantized != value_)
        {
            value_ = quantized;
            last_change_frame_ = frame;
            last_change_time_ = time;
            have_filtered_ = false;
        }
        return value_;
    }

private:
    int minimum_, maximum_, step_, value_;
    std::uint64_t last_change_frame_ = 0;
    double last_change_time_ = 0;
    double filtered_ = 0;
    bool have_filtered_ = false;
};

#endif // GAIN_CONTROLLER_H
