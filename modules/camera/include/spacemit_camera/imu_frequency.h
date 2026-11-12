// Resolve IMU sampling frequency from immutable camera configuration.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_IMU_FREQUENCY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_IMU_FREQUENCY_H_

#include <optional>
#include <string>

namespace camera_sdk
{

// Returns the IMU source sampling rate in hertz. A configured nominal rate
// takes precedence and the other arguments are ignored. Otherwise the rate is
// the product of positive samples_per_frame and camera_fps. Throws
// std::runtime_error if the inputs cannot produce a rate in (0, 5000] Hz.
double ResolveImuFrequencyHz(
    std::optional<double> nominal_frequency_hz, int samples_per_frame, int camera_fps);

// Reads an enabled IMU's source sampling rate from a camera JSON file.
// A positive camera_fps_override replaces the JSON camera FPS when deriving
// the rate. Throws std::runtime_error if reading, parsing or validation fails.
double LoadImuFrequencyHz(const std::string& path, int camera_fps_override = 0);

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_IMU_FREQUENCY_H_
