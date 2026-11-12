// Decode embedded exposure timestamps and IMU samples from stereo frames.

#include "spacemit_camera/embedded_metadata.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace camera_sdk
{
namespace
{

constexpr std::uint32_t kCodeWidth = 160;
constexpr std::uint32_t kCodeStartY = 2;
constexpr std::uint32_t kCellSize = 8;
constexpr std::uint32_t kGroups = 12;
constexpr std::uint32_t kBytesPerGroup = 16;
constexpr float kGravity = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;

std::uint16_t read_be16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

std::uint32_t read_be32(const std::uint8_t* p)
{
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
        (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

bool decode_line(
    const LumaView& image, std::uint32_t line, std::uint8_t* output, std::string* error)
{
    const std::uint32_t y = kCodeStartY + line * kCellSize;
    const auto pixel = [&](std::uint32_t x) -> std::uint8_t
    {
        return image.data[static_cast<std::size_t>(y) * image.row_stride +
            static_cast<std::size_t>(x) * image.pixel_stride];
    };

    std::uint32_t marker_sum = 0;
    for (std::uint32_t x = 0; x < 16; ++x)
    {
        if (pixel(x) <= 220)
        {
            if (error)
                *error = "metadata line has no white prefix";
            return false;
        }
        marker_sum += pixel(x);
    }
    for (std::uint32_t x = 144; x < 160; ++x)
    {
        if (pixel(x) <= 220)
        {
            if (error)
                *error = "metadata line has no white suffix";
            return false;
        }
        marker_sum += pixel(x);
    }

    // Bright data cells can reach Y=223 in recorded MJPEG. Distinguish
    // them from the line's white markers, including limited-range white.
    const std::uint32_t blank_sum = std::min<std::uint32_t>(480, marker_sum / 16 - 16);

    output[0] = 0;
    output[1] = 0;
    for (std::uint32_t bit = 0; bit < 16; ++bit)
    {
        const std::uint32_t x = 20 + bit * kCellSize;
        const std::uint16_t sum = static_cast<std::uint16_t>(pixel(x)) + pixel(x + 1);
        bool value = false;
        if (sum < 100)
        {
            value = false;
        }
        else if (sum < blank_sum)
        {
            value = true;
        }
        else
        {
            if (error)
                *error = "metadata bit cell is blank";
            return false;
        }
        output[bit / 8] |= static_cast<std::uint8_t>(value) << (bit % 8);
    }
    return true;
}

void decode_imu(const std::uint8_t* group, ImuSample& sample)
{
    sample.timestamp_raw_us = read_be32(group);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        sample.accel_raw[axis] = static_cast<std::int16_t>(read_be16(group + 4 + axis * 2));
        sample.gyro_raw[axis] = static_cast<std::int16_t>(read_be16(group + 10 + axis * 2));
    }
    sample.valid =
        !((sample.accel_raw[0] == -1 && sample.accel_raw[1] == -1 && sample.accel_raw[2] == -1) ||
            sample.gyro_raw[0] == -32768);
    if (!sample.valid)
        return;

    const float accel_scale = 4.0f * kGravity / 32768.0f;
    const float gyro_scale = 1000.0f * kPi / (180.0f * 32768.0f);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        sample.accel_m_s2[axis] = sample.accel_raw[axis] * accel_scale;
        sample.gyro_rad_s[axis] = sample.gyro_raw[axis] * gyro_scale;
    }
}

} // namespace

bool decode_embedded_metadata(const LumaView& image, EmbeddedMetadata& metadata, std::string* error)
{
    metadata = {};
    if (!image.data || image.width < kCodeWidth || image.height < 763 || image.pixel_stride == 0 ||
        image.row_stride < static_cast<std::size_t>(image.width) * image.pixel_stride)
    {
        if (error)
            *error = "invalid luma view for 160x768 metadata area";
        return false;
    }

    std::array<std::uint8_t, kGroups * kBytesPerGroup> bytes{};
    for (std::uint32_t line = 0; line < 8; ++line)
    {
        if (!decode_line(image, line, bytes.data() + line * 2, error))
            return false;
    }

    std::array<std::uint8_t, 5> device_type{};
    std::array<std::uint8_t, 5> device_count{};
    if (std::memcmp(bytes.data(), bytes.data() + 8, 8) == 0)
    {
        metadata.protocol = 0;
        device_type[0] = 1;
        device_count[0] = 11;
        metadata.exposure_start_raw_us = read_be32(bytes.data());
        metadata.exposure_end_raw_us = read_be32(bytes.data() + 4);
    }
    else
    {
        metadata.protocol = bytes[0] >> 4;
        device_type[0] = static_cast<std::uint8_t>(((bytes[0] & 0x0f) << 4) | (bytes[1] >> 4));
        device_count[0] = bytes[1] & 0x0f;
        device_type[1] = bytes[2];
        device_count[1] = bytes[3] >> 4;
        device_type[2] = static_cast<std::uint8_t>(((bytes[3] & 0x0f) << 4) | (bytes[4] >> 4));
        device_count[2] = bytes[4] & 0x0f;
        device_type[3] = bytes[5];
        device_count[3] = bytes[6] >> 4;
        device_type[4] = static_cast<std::uint8_t>(((bytes[6] & 0x0f) << 4) | (bytes[7] >> 4));
        device_count[4] = bytes[7] & 0x0f;
        metadata.exposure_start_raw_us = read_be32(bytes.data() + 8);
        metadata.exposure_end_raw_us = read_be32(bytes.data() + 12);
    }

    std::size_t group_count = 1;
    for (std::uint8_t count : device_count)
        group_count += count;
    if (group_count > kGroups)
    {
        if (error)
            *error = "metadata header requests more than 12 groups";
        return false;
    }
    for (std::size_t line = 8; line < group_count * 8; ++line)
    {
        if (!decode_line(image, static_cast<std::uint32_t>(line), bytes.data() + line * 2, error))
        {
            return false;
        }
    }

    std::size_t group = 1;
    for (std::size_t device = 0; device < device_type.size(); ++device)
    {
        for (std::size_t index = 0; index < device_count[device]; ++index, ++group)
        {
            if (device_type[device] == 1 && metadata.imu_count < metadata.imu.size())
            {
                decode_imu(
                    bytes.data() + group * kBytesPerGroup, metadata.imu[metadata.imu_count++]);
            }
        }
    }
    metadata.valid = true;
    return true;
}

} // namespace camera_sdk
