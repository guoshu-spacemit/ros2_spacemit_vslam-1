#ifndef SESSION_REPLAY_INDEX_H
#define SESSION_REPLAY_INDEX_H
#include <algorithm>
#include <charconv>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace camera_io::replay
{
struct Frame
{
    std::uint32_t id;
    std::int64_t stamp;
    std::uint64_t pts;
    std::uint32_t bytes;
    bool adjusted;
    bool target{false};
};
struct Target
{
    std::uint32_t id;
    std::int64_t stamp;
};

inline std::uint64_t number(const std::string& text)
{
    std::uint64_t value{};
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size())
        throw std::runtime_error("invalid unsigned integer: " + text);
    return value;
}
inline Target target(const std::string& id, const std::string& stamp)
{
    const auto i = number(id), t = number(stamp);
    if (i > std::numeric_limits<std::uint32_t>::max() || t == 0 ||
        t > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() - 1000000000LL))
        throw std::runtime_error("frame ID or timestamp out of range");
    return {static_cast<std::uint32_t>(i), static_cast<std::int64_t>(t)};
}
inline std::vector<Target> read_targets(const std::string& path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open " + path);
    std::vector<Target> result;
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
            if (line.rfind("source_frame_index,image_timestamp_ns,", 0) != 0)
                throw std::runtime_error("invalid VIO CSV header");
            header = true;
            continue;
        }
        const auto first = line.find(','), second = line.find(',', first + 1);
        if (first == std::string::npos || second == std::string::npos)
            throw std::runtime_error("truncated VIO CSV row");
        auto row = target(line.substr(0, first), line.substr(first + 1, second - first - 1));
        if (!result.empty() && (row.id <= result.back().id || row.stamp <= result.back().stamp))
            throw std::runtime_error("duplicate or unordered VIO target");
        result.push_back(row);
    }
    if (input.bad() || result.empty())
        throw std::runtime_error("empty or unreadable VIO targets");
    return result;
}
inline std::vector<Frame> read_camera(const std::string& path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open " + path);
    std::vector<Frame> result;
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
            if (line != "frame_index timestamp_ns source_pts_us packet_size status")
                throw std::runtime_error("invalid camera index header");
            header = true;
            continue;
        }
        std::istringstream fields(line);
        std::string id, stamp, pts, bytes, status, extra;
        if (!(fields >> id >> stamp >> pts >> bytes >> status) || fields >> extra)
            throw std::runtime_error("invalid camera index row");
        const auto key = target(id, stamp);
        const auto size = number(bytes), source_pts = number(pts);
        if (size < 4 || size > static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
            (status != "ok" && status != "pts_adjusted"))
            throw std::runtime_error("invalid camera packet size/status");
        if (!result.empty() &&
            (key.id <= result.back().id || key.stamp <= result.back().stamp ||
                source_pts <= result.back().pts))
            throw std::runtime_error("duplicate or unordered camera index");
        result.push_back({key.id, key.stamp, source_pts, static_cast<std::uint32_t>(size),
            status == "pts_adjusted", false});
    }
    if (input.bad() || result.empty())
        throw std::runtime_error("empty or unreadable camera index");
    return result;
}
inline bool associate(std::vector<Frame>& frames, const std::vector<Target>& targets)
{
    std::size_t next = 0;
    for (auto& frame : frames)
    {
        frame.target = false;
        if (next < targets.size() && frame.id == targets[next].id &&
            frame.stamp == targets[next].stamp)
        {
            frame.target = true;
            ++next;
        }
    }
    return next == targets.size();
}
inline Target parse_ack(const std::string& text)
{
    const auto comma = text.find(',');
    if (comma == std::string::npos)
        throw std::runtime_error("invalid replay acknowledgement");
    return target(text.substr(0, comma), text.substr(comma + 1));
}
} // namespace camera_io::replay

#endif // SESSION_REPLAY_INDEX_H
