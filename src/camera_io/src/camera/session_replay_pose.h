#ifndef SESSION_REPLAY_POSE_H
#define SESSION_REPLAY_POSE_H
#include "session_replay_index.h"

#include <array>
#include <cmath>

namespace camera_io::replay
{
struct Pose
{
    std::int64_t stamp;
    std::array<double, 7> values;
};

// A frozen native-map snapshot is joined by integer exposure time. Neither
// interpolation nor quaternion normalization is appropriate at this boundary.
inline std::vector<Pose> read_poses(const std::string& path, const std::vector<Target>& targets)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open " + path);
    std::vector<Pose> result;
    std::string line;
    bool header = false;
    while (std::getline(input, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty() || line.front() == '#')
            continue;
        if (!header)
        {
            if (line != "timestamp_ns,tx,ty,tz,qx,qy,qz,qw")
                throw std::runtime_error("invalid frozen pose CSV header");
            header = true;
            continue;
        }
        if (result.size() >= targets.size())
            throw std::runtime_error("frozen pose CSV has more rows than targets");
        std::istringstream row(line);
        std::string field;
        if (!std::getline(row, field, ','))
            throw std::runtime_error("missing frozen pose timestamp");
        const auto stamp = number(field);
        if (stamp != static_cast<std::uint64_t>(targets[result.size()].stamp))
            throw std::runtime_error("frozen pose timestamp does not match exact target");
        Pose pose{static_cast<std::int64_t>(stamp), {}};
        for (auto& value : pose.values)
        {
            if (!std::getline(row, field, ','))
                throw std::runtime_error("truncated frozen pose row");
            std::size_t consumed = 0;
            value = std::stod(field, &consumed);
            if (consumed != field.size() || !std::isfinite(value))
                throw std::runtime_error("invalid frozen pose component");
        }
        if (std::getline(row, field, ',') || (!line.empty() && line.back() == ','))
            throw std::runtime_error("extra frozen pose component");
        double norm = 0;
        for (std::size_t i = 3; i < pose.values.size(); ++i)
            norm += pose.values[i] * pose.values[i];
        if (std::abs(norm - 1.0) > 1e-5)
            throw std::runtime_error("frozen pose quaternion is not normalized");
        result.push_back(pose);
    }
    if (input.bad() || result.empty() || result.size() != targets.size())
        throw std::runtime_error("incomplete or unreadable frozen pose CSV");
    return result;
}
} // namespace camera_io::replay

#endif // SESSION_REPLAY_POSE_H
