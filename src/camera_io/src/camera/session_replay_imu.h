#ifndef SESSION_REPLAY_IMU_H
#define SESSION_REPLAY_IMU_H
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace camera_io::replay
{

// camera_sdk_embedded's signed 16-bit +/-1000 deg/s, +/-4g wire format.
// This is NOT a generic rounding rule for arbitrary IMU sources.
inline float embedded_imu_scale(bool gyro)
{
    constexpr float pi = 3.14159265358979323846f;
    return gyro ? 1000.0f * pi / (180.0f * 32768.0f) : 4.0f * 9.80665f / 32768.0f;
}

inline float recover_embedded_imu(double saved, bool gyro)
{
    const float scale = embedded_imu_scale(gyro);
    if (!std::isfinite(saved) || saved / scale < -32768.5 || saved / scale >= 32767.5)
        throw std::runtime_error("invalid embedded IMU sidecar value");
    const float restored = static_cast<float>(std::lround(saved / scale)) * scale;
    // Old recorders wrote only six significant digits. Accept exactly that
    // representation or a lossless representation, not an approximate match.
    std::ostringstream legacy;
    legacy.imbue(std::locale::classic());
    legacy << restored;
    if (saved != static_cast<double>(restored) && saved != std::stod(legacy.str()))
        throw std::runtime_error("IMU sidecar does not match embedded wire format");
    return restored;
}

struct ReplayImu
{
    std::uint64_t stamp{};
    std::array<double, 4> orientation{};
    std::array<float, 6> motion{}; // gx gy gz ax ay az
};

// Streaming reader: a long recording does not allocate an unbounded IMU array.
class EmbeddedImuSidecar
{
public:
    explicit EmbeddedImuSidecar(const std::string& path) : input_(path)
    {
        input_.imbue(std::locale::classic());
        std::string header;
        if (!std::getline(input_, header) || header != "timestamp_ns qx qy qz qw gx gy gz ax ay az")
            throw std::runtime_error("invalid IMU sidecar header");
        advance();
    }
    bool available() const { return available_; }
    const ReplayImu& row() const { return row_; }
    void advance()
    {
        std::string line;
        if (!std::getline(input_, line))
        {
            if (!input_.eof())
                throw std::runtime_error("IMU sidecar read failed");
            available_ = false;
            return;
        }
        std::istringstream values(line);
        values.imbue(std::locale::classic());
        std::int64_t stamp{};
        if (!(values >> stamp) || stamp <= 0 ||
            stamp / 1000000000LL > std::numeric_limits<std::int32_t>::max() ||
            static_cast<std::uint64_t>(stamp) <= previous_)
            throw std::runtime_error("invalid/nonmonotonic IMU sidecar timestamp");
        row_.stamp = static_cast<std::uint64_t>(stamp);
        for (auto& q : row_.orientation)
            if (!(values >> q) || !std::isfinite(q))
                throw std::runtime_error("invalid IMU sidecar orientation");
        for (std::size_t i = 0; i < row_.motion.size(); ++i)
        {
            double value{};
            if (!(values >> value))
                throw std::runtime_error("truncated IMU sidecar");
            row_.motion[i] = recover_embedded_imu(value, i < 3);
        }
        std::string extra;
        if (values >> extra)
            throw std::runtime_error("extra IMU sidecar fields");
        previous_ = row_.stamp;
        available_ = true;
    }

    void verify(std::uint64_t stamp, const std::array<float, 6>& motion)
    {
        while (available_ && row_.stamp < stamp)
            advance();
        if (available_ && row_.stamp == stamp)
        {
            if (motion != row_.motion)
                throw std::runtime_error("embedded/sidecar IMU disagreement");
            ++verified_;
            advance();
        }
        else if (verified_ != 0)
        {
            throw std::runtime_error("missing IMU sidecar sample");
        }
    }
    std::size_t verified() const { return verified_; }

private:
    std::ifstream input_;
    ReplayImu row_;
    bool available_{false};
    std::uint64_t previous_{0};
    std::size_t verified_{0};
};
} // namespace camera_io::replay

#endif // SESSION_REPLAY_IMU_H
