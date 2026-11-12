// Camera exposure-clock extension and epoch timestamp alignment.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_CAMERA_CLOCK_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_CAMERA_CLOCK_H_

#include <cstdint>
#include <string>

namespace camera_sdk
{

// Maps the camera's wrapping 32-bit microsecond clock into ROS time. A clock
// update is intentionally a two-step operation: preview() validates a decoded
// metadata block without changing state, then commit() advances the anchor.
// This prevents recycled decoder frames from moving the hardware clock back.
class CameraClock
{
public:
    enum class ExposureReference
    {
        Start,
        Midpoint
    };

    struct FrameTimestamps
    {
        std::uint32_t exposure_end_raw_us{0};
        std::int64_t exposure_end_extended_us{0};
        std::int64_t epoch_offset_ns{0};
        std::int64_t exposure_start_ns{0};
        std::int64_t exposure_end_ns{0};

        std::int64_t exposure_midpoint_ns() const
        {
            return exposure_start_ns + (exposure_end_ns - exposure_start_ns) / 2;
        }
    };

    void reset();

    bool preview(std::uint32_t exposure_start_raw_us, std::uint32_t exposure_end_raw_us,
        std::int64_t ros_now_ns, FrameTimestamps* timestamps, std::string* error = nullptr) const;

    // Recover the original device/ROS clock mapping from a recorded image
    // stamp. The reference must match the recording's timestamp.source.
    bool preview_recorded(std::uint32_t exposure_start_raw_us, std::uint32_t exposure_end_raw_us,
        std::int64_t recorded_image_ns, ExposureReference reference, FrameTimestamps* timestamps,
        std::string* error = nullptr) const;

    void commit(const FrameTimestamps& timestamps);

    std::int64_t ros_time_ns(std::uint32_t raw_us) const;

    bool initialized() const;

private:
    static std::int32_t signed_delta(std::uint32_t value, std::uint32_t reference);

    bool initialized_{false};
    std::uint32_t anchor_raw_us_{0};
    std::int64_t anchor_extended_us_{0};
    std::int64_t epoch_offset_ns_{0};
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_CAMERA_CLOCK_H_
