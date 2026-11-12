// Decode and validate Seeker event wire messages.

#include "spacemit_camera/seeker_protocol.h"

#include <cstring>
#include <limits>

namespace camera_sdk
{
namespace
{

std::uint64_t read_le(const std::uint8_t* data, std::size_t size)
{
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < size; ++i)
    {
        value |= static_cast<std::uint64_t>(data[i]) << (8 * i);
    }
    return value;
}

double read_double(const std::uint8_t* data)
{
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
        "Seeker requires IEEE 754 binary64");
    const auto bits = read_le(data, 8);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

template <std::size_t N> void read_array(const std::uint8_t* data, std::array<double, N>& values)
{
    for (std::size_t i = 0; i < N; ++i)
        values[i] = read_double(data + i * 8);
}

bool fail(std::string* error, const char* message)
{
    if (error)
        *error = message;
    return false;
}

} // namespace

bool decode_seeker_event(const std::uint8_t* data, std::size_t size, SeekerEvent& output,
    std::string* error, SeekerEnvelopePolicy policy)
{
    if (!data || size < 8)
        return fail(error, "truncated Seeker envelope");
    const auto type = static_cast<SeekerEventType>(read_le(data + 4, 4));
    const bool legacy = policy == SeekerEnvelopePolicy::allow_legacy_zero && data[0] == 0 &&
        data[1] == 0 &&
        (type == SeekerEventType::sensor_custom || type == SeekerEventType::uvc_info);
    if (!legacy && data[0] != 1)
        return fail(error, "unsupported Seeker protocol version");
    // A response can retain the command kind; preserve it for the caller.
    if (!legacy && (data[1] < 1 || data[1] > 3))
        return fail(error, "invalid Seeker protocol type");
    std::size_t minimum = 0;
    switch (type)
    {
    case SeekerEventType::uvc_info:
        minimum = 40;
        break;
    case SeekerEventType::sensor_custom:
        minimum = 456;
        break;
    case SeekerEventType::camera_calibration:
        minimum = seeker_event_wire_size;
        break;
    default:
        return fail(error, "unsupported Seeker event type");
    }
    if (size != minimum && size != seeker_event_wire_size)
    {
        return fail(error, "invalid Seeker message length");
    }

    SeekerEvent decoded;
    decoded.protocol_version = data[0];
    decoded.protocol_type = data[1];
    decoded.type = type;
    const auto* payload = data + 8;
    decoded.header.seconds = read_le(payload + 8, 8);
    decoded.header.nanoseconds = read_le(payload + 16, 8);
    decoded.header.sequence = static_cast<std::uint32_t>(read_le(payload + 24, 4));
    if (type == SeekerEventType::camera_calibration)
    {
        auto& calibration = decoded.calibration;
        calibration.calibration_flag = read_le(payload + 32, 8);
        calibration.fx = static_cast<std::uint32_t>(read_le(payload + 40, 4));
        calibration.fy = static_cast<std::uint32_t>(read_le(payload + 44, 4));
        for (std::size_t i = 0; i < calibration.cameras.size(); ++i)
        {
            const auto* camera = payload + 48 + i * 205;
            auto& value = calibration.cameras[i];
            value.camera_id = camera[0];
            value.model = static_cast<std::uint32_t>(read_le(camera + 1, 4));
            read_array(camera + 5, value.camera_from_imu);
            read_array(camera + 61, value.camera_from_previous);
            read_array(camera + 117, value.distortion);
            value.xi = read_double(camera + 157);
            value.fx = read_double(camera + 165);
            value.fy = read_double(camera + 173);
            value.cx = read_double(camera + 181);
            value.cy = read_double(camera + 189);
            value.width = static_cast<std::uint32_t>(read_le(camera + 197, 4));
            value.height = static_cast<std::uint32_t>(read_le(camera + 201, 4));
        }
    }
    else if (type == SeekerEventType::sensor_custom)
    {
        auto& sensor = decoded.sensor;
        sensor.valid_flags = read_le(payload + 32, 8);
        read_array(payload + 40, sensor.orientation_wxyz);
        read_array(payload + 72, sensor.orientation_covariance);
        read_array(payload + 144, sensor.angular_velocity);
        read_array(payload + 168, sensor.angular_velocity_covariance);
        read_array(payload + 240, sensor.linear_acceleration);
        read_array(payload + 264, sensor.linear_acceleration_covariance);
        read_array(payload + 336, sensor.magnetic_field);
        read_array(payload + 360, sensor.magnetic_field_covariance);
        sensor.pressure = read_double(payload + 432);
        sensor.temperature = read_double(payload + 440);
    }
    output = decoded;
    if (error)
        error->clear();
    return true;
}

std::array<std::uint8_t, seeker_event_wire_size> make_seeker_camera_calibration_request()
{
    std::array<std::uint8_t, seeker_event_wire_size> request{};
    request[0] = 1;
    request[1] = 2;
    const auto type = static_cast<std::uint32_t>(SeekerEventType::camera_calibration);
    for (std::size_t i = 0; i < 4; ++i)
        request[4 + i] = static_cast<std::uint8_t>(type >> (8 * i));
    return request;
}

bool seeker_timestamp_ns(const SeekerEventHeader& header, std::uint64_t& output)
{
    constexpr std::uint64_t billion = 1000000000;
    if (header.nanoseconds >= billion ||
        header.seconds > (std::numeric_limits<std::uint64_t>::max() - header.nanoseconds) / billion)
    {
        return false;
    }
    output = header.seconds * billion + header.nanoseconds;
    return true;
}

} // namespace camera_sdk
