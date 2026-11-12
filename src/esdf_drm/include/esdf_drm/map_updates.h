#ifndef MAP_UPDATES_H
#define MAP_UPDATES_H

#include <spacemit_mapping.h>
#include <array>
#include <functional>
#include <vector>

namespace esdf_drm
{
namespace mapping = spacemit_mapping::sdk;
using RgbPixel = std::array<std::uint8_t, 3>;
static_assert(sizeof(RgbPixel) == 3);
struct ChunkKeyHash
{
    std::size_t operator()(const mapping::ChunkKey& key) const noexcept
    {
        const auto a = std::hash<std::uint64_t>{}(key.low);
        const auto b = std::hash<std::uint64_t>{}(key.high);
        return a ^ (b + static_cast<std::size_t>(0x9e3779b9U) + (a << 6U) + (a >> 2U));
    }
};
// Queue metadata only: each entry retains exactly one immutable SDK update.
struct MapChange
{
    mapping::ChunkKey index;
    bool removed = false;
    mapping::Snapshot payload;
};
inline std::vector<MapChange> split_changes(const mapping::Snapshot& snapshot)
{
    std::vector<MapChange> result;
    result.reserve(snapshot.size());
    for (std::size_t item = 0; item < snapshot.size(); ++item)
    {
        const auto info = snapshot.info(item);
        result.push_back({info.key, info.removed, snapshot.select(item)});
    }
    return result;
}
inline std::vector<mapping::Snapshot> queued_snapshots(const std::vector<MapChange>& changes)
{
    std::vector<mapping::Snapshot> result;
    result.reserve(changes.size());
    for (const auto& change : changes)
        result.push_back(change.payload);
    return result;
}
} // namespace esdf_drm
#endif
