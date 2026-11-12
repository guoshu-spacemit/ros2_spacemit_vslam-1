#ifndef REPLAY_MANIFEST_H
#define REPLAY_MANIFEST_H
#include <charconv>
#include <cstdint>
#include <istream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace vio
{
struct ReplayKey
{
    std::uint32_t frame_index;
    std::int64_t timestamp_ns;
    bool operator==(const ReplayKey& other) const
    {
        return frame_index == other.frame_index && timestamp_ns == other.timestamp_ns;
    }
    std::string text() const
    {
        return std::to_string(frame_index) + "," + std::to_string(timestamp_ns);
    }
};

// Parse the original FrameRecorder CSV; pose/status columns do not select input.
inline std::vector<ReplayKey> read_replay_manifest(std::istream& input)
{
    std::vector<ReplayKey> keys;
    std::vector<std::string> header;
    std::size_t index_column = 0, stamp_column = 0, line_number = 0;
    std::string line;
    while (std::getline(input, line))
    {
        ++line_number;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty() || line.front() == '#')
            continue;
        std::vector<std::string> fields;
        std::size_t start = 0;
        for (;;)
        {
            const auto end = line.find(',', start);
            fields.push_back(line.substr(start, end == std::string::npos ? end : end - start));
            if (end == std::string::npos)
                break;
            start = end + 1;
        }
        auto fail = [&]()
        {
            throw std::runtime_error(
                "invalid replay manifest at line " + std::to_string(line_number));
        };
        if (header.empty())
        {
            header = fields;
            unsigned indices = 0, stamps = 0;
            for (std::size_t i = 0; i < fields.size(); ++i)
            {
                if (fields[i] == "source_frame_index")
                {
                    index_column = i;
                    ++indices;
                }
                if (fields[i] == "image_timestamp_ns")
                {
                    stamp_column = i;
                    ++stamps;
                }
            }
            if (indices != 1 || stamps != 1)
                fail();
            continue;
        }
        if (fields.size() != header.size())
            fail();
        auto number = [&](std::size_t column)
        {
            std::uint64_t value = 0;
            const auto& field = fields[column];
            const auto result = std::from_chars(field.data(), field.data() + field.size(), value);
            if (result.ec != std::errc{} || result.ptr != field.data() + field.size())
                fail();
            return value;
        };
        const auto index = number(index_column), stamp = number(stamp_column);
        if (index > std::numeric_limits<std::uint32_t>::max() || stamp == 0 ||
            stamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
            fail();
        ReplayKey key{static_cast<std::uint32_t>(index), static_cast<std::int64_t>(stamp)};
        if (!keys.empty() &&
            (key.frame_index <= keys.back().frame_index ||
                key.timestamp_ns <= keys.back().timestamp_ns))
            fail();
        keys.push_back(key);
    }
    if (input.bad() || keys.empty())
        throw std::runtime_error("empty or unreadable replay manifest");
    return keys;
}
} // namespace vio

#endif // REPLAY_MANIFEST_H
