// Embedded stereo exposure and IMU metadata contracts.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_EMBEDDED_METADATA_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_EMBEDDED_METADATA_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace camera_sdk
{

constexpr std::size_t kMaxImuSamples = 11;

struct LumaView
{
    const std::uint8_t* data = nullptr;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::size_t row_stride = 0;
    std::size_t pixel_stride = 1;
};

struct ImuSample
{
    std::uint32_t timestamp_raw_us = 0;
    std::uint64_t timestamp_us = 0;
    std::array<std::int16_t, 3> accel_raw{};
    std::array<std::int16_t, 3> gyro_raw{};
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_rad_s{};
    bool valid = false;
};

struct EmbeddedMetadata
{
    std::uint8_t protocol = 0;
    std::uint32_t exposure_start_raw_us = 0;
    std::uint32_t exposure_end_raw_us = 0;
    std::uint64_t exposure_start_us = 0;
    std::uint64_t exposure_end_us = 0;
    std::array<ImuSample, kMaxImuSamples> imu{};
    std::size_t imu_count = 0;
    bool valid = false;
};

bool decode_embedded_metadata(
    const LumaView& image, EmbeddedMetadata& metadata, std::string* error = nullptr);

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_EMBEDDED_METADATA_H_
