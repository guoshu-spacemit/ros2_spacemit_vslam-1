#ifndef BLOCK_REMOVAL_QUEUE_H
#define BLOCK_REMOVAL_QUEUE_H
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <utility>
#include <vector>

namespace esdf_drm
{

// A bounded hash map is not a FIFO: repeatedly refreshed entries can starve
// tombstones near the end of its iteration order. Drain ALL removals before
// applying the ordinary upload budget. Deletions do not upload voxel payloads.
template <typename Map>
std::vector<typename Map::mapped_type> take_render_updates(Map& pending, std::size_t budget)
{
    std::vector<typename Map::mapped_type> updates;
    updates.reserve(std::min(pending.size(), budget));
    for (auto entry = pending.begin(); entry != pending.end();)
    {
        if (entry->second.removed)
        {
            updates.push_back(std::move(entry->second));
            entry = pending.erase(entry);
        }
        else
            ++entry;
    }
    auto entry = pending.begin();
    while (entry != pending.end() && budget > 0)
    {
        --budget;
        updates.push_back(std::move(entry->second));
        entry = pending.erase(entry);
    }
    return updates;
}

// Snapshot updates may be dropped under pressure; removals may not. Waiting
// releases the queue mutex and only stalls the mapping worker, never VIO.
template <typename Map, typename Index>
void wait_for_removal_slot(const Map& pending, const Index& index, std::size_t capacity,
    std::unique_lock<std::mutex>& lock, std::condition_variable& condition,
    const std::atomic<bool>& stopping)
{
    condition.wait(lock,
        [&] { return stopping.load() || pending.size() < capacity || pending.count(index) != 0; });
}

// Also wake a producer waiting for a removal slot if the consumer exits early.
struct NotifyStopped
{
    std::atomic<bool>& stopping;
    std::condition_variable& condition;
    std::mutex& mutex;
    ~NotifyStopped()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stopping.store(true);
        }
        condition.notify_all();
    }
};

} // namespace esdf_drm

#endif // BLOCK_REMOVAL_QUEUE_H
