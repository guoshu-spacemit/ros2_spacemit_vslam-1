// Seeker event wire contracts and validated message decoding.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_SEEKER_PROTOCOL_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_SEEKER_PROTOCOL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace camera_sdk
{

// Public Seeker protocol v1. These are wire values, not host-time estimates.
enum class SeekerEventType : std::uint32_t
{
    sensor_custom = 0x49430104,
    uvc_info = 0x49430110,
    camera_calibration = 0x44430107,
};

struct SeekerEventHeader
{
    std::uint64_t seconds = 0;
    std::uint64_t nanoseconds = 0;
    std::uint32_t sequence = 0;
};

struct SeekerCameraCalibration
{
    std::uint8_t camera_id = 0;
    // Preserve the vendor model ID, including unknown models.
    std::uint32_t model = 0;
    // Vendor T_cam_imu and T_cn_cnm1: qw, qx, qy, qz, tx, ty, tz.
    std::array<double, 7> camera_from_imu{};
    std::array<double, 7> camera_from_previous{};
    // Preserve k1, k2, p1, p2, k3 without reinterpreting model semantics.
    std::array<double, 5> distortion{};
    double xi = 0;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

struct SeekerCalibration
{
    // Marked reserved in the public SDK; this is not a camera-count bitmask.
    std::uint64_t calibration_flag = 0;
    std::uint32_t fx = 0;
    std::uint32_t fy = 0;
    // Protocol slots are not automatically eight physical cameras.
    std::array<SeekerCameraCalibration, 8> cameras{};
};

struct SeekerSensorSample
{
    std::uint64_t valid_flags = 0;
    std::array<double, 4> orientation_wxyz{};
    std::array<double, 9> orientation_covariance{};
    std::array<double, 3> angular_velocity{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 3> linear_acceleration{};
    std::array<double, 9> linear_acceleration_covariance{};
    std::array<double, 3> magnetic_field{};
    std::array<double, 9> magnetic_field_covariance{};
    double pressure = 0;
    double temperature = 0;
};

struct SeekerEvent
{
    std::uint8_t protocol_version = 0;
    std::uint8_t protocol_type = 0;
    SeekerEventType type = SeekerEventType::uvc_info;
    SeekerEventHeader header;
    SeekerCalibration calibration;
    SeekerSensorSample sensor;
};

constexpr std::size_t seeker_event_wire_size = 1696;

enum class SeekerEnvelopePolicy
{
    require_v1,
    // Observed on Omni Depth USB event transfers. Only accepts (0, 0) for
    // sensor_custom/uvc_info; preserves the missing version in the output.
    allow_legacy_zero,
};

// Decode one complete little-endian message, either its defined payload size
// or the full fixed-size event union. No packed host structs or vendor .so.
// This only parses fields: it does not validate physical calibration, convert
// sensor units, associate a UVC image, or establish the meaning of its clock.
// On failure output is unchanged. Optional error is cleared on success.
bool decode_seeker_event(const std::uint8_t* data, std::size_t size, SeekerEvent& output,
    std::string* error = nullptr, SeekerEnvelopePolicy policy = SeekerEnvelopePolicy::require_v1);

std::array<std::uint8_t, seeker_event_wire_size> make_seeker_camera_calibration_request();

// Reject an invalid fractional second or uint64 overflow; no clock mapping.
bool seeker_timestamp_ns(const SeekerEventHeader& header, std::uint64_t& output);

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_SEEKER_PROTOCOL_H_
