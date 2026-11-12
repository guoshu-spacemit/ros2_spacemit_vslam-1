// Bounded matching of device and GPIO exposure sequences.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_EXPOSURE_MATCHER_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_EXPOSURE_MATCHER_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace camera_sdk
{
// Pure sequence matcher. No GPIO, ROS, exposure control, or arrival-time
// heuristic.
class ExposureMatcher
{
public:
    struct Exposure
    {
        std::int64_t start_ns{}, end_ns{};
    };
    struct Config
    {
        std::size_t window{32}, capacity{256};
        std::int64_t width_tolerance_ns{100000}, interval_tolerance_ns{200000};
        std::int64_t max_age_ns{500000000}, diversity_ns{500000};
    };
    explicit ExposureMatcher(Config config) : config_(config)
    {
        if (config.window < 4 || config.window > 128 || config.capacity < config.window + 2 ||
            config.capacity > 4096 || config.width_tolerance_ns <= 0 ||
            config.interval_tolerance_ns <= 0 || config.max_age_ns <= 0 ||
            config.diversity_ns <= config.width_tolerance_ns * 2)
            throw std::invalid_argument("invalid GPIO matching thresholds");
    }
    void reset(const std::string& reason)
    {
        device_.clear();
        locked_ = false;
        misses_ = 0;
        last_device_ = {};
        last_gpio_ = {};
        state_ = reason;
    }
    bool locked() const { return locked_; }
    const std::string& state() const { return state_; }
    std::optional<Exposure> match(
        Exposure device, const std::vector<Exposure>& gpio, std::int64_t now_ns)
    {
        if (device.end_ns <= device.start_ns ||
            (!device_.empty() && device.end_ns <= device_.back().end_ns))
        {
            reset("invalid_or_regressed_metadata");
            return {};
        }
        device_.push_back(device);
        while (device_.size() > config_.window)
            device_.pop_front();
        auto fresh = [&](const Exposure& e)
        { return e.end_ns <= now_ns && now_ns - e.end_ns <= config_.max_age_ns; };
        auto width = [](const Exposure& e) { return e.end_ns - e.start_ns; };
        if (locked_)
        {
            std::optional<Exposure> selected;
            for (const auto& e : gpio)
            {
                if (!fresh(e) || e.start_ns <= last_gpio_.start_ns)
                    continue;
                if (std::llabs((e.start_ns - last_gpio_.start_ns) -
                        (device.start_ns - last_device_.start_ns)) >
                        config_.interval_tolerance_ns ||
                    std::llabs(width(e) - width(device)) > config_.width_tolerance_ns)
                    continue;
                if (selected)
                {
                    reset("ambiguous_locked_match");
                    return {};
                }
                selected = e;
            }
            if (selected)
            {
                last_device_ = device;
                last_gpio_ = *selected;
                misses_ = 0;
                state_ = "locked";
                return selected;
            }
            // Do not publish an unverified frame. A single IRQ timestamp outlier or
            // delayed edge need not destroy the last verified anchor; the next frame
            // must still independently satisfy both interval and width constraints.
            if (++misses_ >= 3)
                reset("lost_correspondence");
            else
                state_ = "unverified_frame_dropped";
            return {};
        }
        if (device_.size() < config_.window)
        {
            state_ = "collecting";
            return {};
        }
        auto lo = width(device_.front()), hi = lo;
        unsigned transitions = 0;
        for (std::size_t i = 0; i < device_.size(); ++i)
        {
            lo = std::min(lo, width(device_[i]));
            hi = std::max(hi, width(device_[i]));
            if (i && std::llabs(width(device_[i]) - width(device_[i - 1])) >= config_.diversity_ns)
                ++transitions;
        }
        if (hi - lo < config_.diversity_ns || transitions < 2)
        {
            state_ = "need_exposure_variation";
            return {};
        }
        std::optional<Exposure> selected;
        for (std::size_t base = 0; base + device_.size() <= gpio.size(); ++base)
        {
            const auto& end = gpio[base + device_.size() - 1];
            if (!fresh(end))
                continue;
            bool valid = true;
            for (std::size_t i = 0; i < device_.size(); ++i)
            {
                if (std::llabs(width(device_[i]) - width(gpio[base + i])) >
                        config_.width_tolerance_ns ||
                    (i &&
                        std::llabs((device_[i].start_ns - device_[i - 1].start_ns) -
                            (gpio[base + i].start_ns - gpio[base + i - 1].start_ns)) >
                            config_.interval_tolerance_ns))
                {
                    valid = false;
                    break;
                }
            }
            if (!valid)
                continue;
            if (selected)
            {
                state_ = "ambiguous_acquisition";
                return {};
            }
            selected = end;
        }
        if (!selected)
        {
            state_ = "no_matching_sequence";
            return {};
        }
        last_device_ = device;
        last_gpio_ = *selected;
        locked_ = true;
        state_ = "locked";
        return selected;
    }

private:
    Config config_;
    std::deque<Exposure> device_;
    Exposure last_device_, last_gpio_;
    bool locked_{false};
    unsigned misses_{0};
    std::string state_{"collecting"};
};
} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_EXPOSURE_MATCHER_H_
