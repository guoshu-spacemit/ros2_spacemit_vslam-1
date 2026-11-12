#ifndef HOST_PACKET_POOL_H
#define HOST_PACKET_POOL_H
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace camera_io::recording
{

// Disk staging needs CPU storage, not a physically contiguous DMA allocation.
// Allocate once before subscribing. Leases return slots even on writer failure.
class HostPacketPool
{
    struct State
    {
        State(std::size_t count, std::size_t bytes)
        {
            if (count == 0 || bytes == 0)
                throw std::invalid_argument("empty packet pool");
            buffers.reserve(count);
            free.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                // Value initialization faults in the storage before camera capture.
                buffers.emplace_back(new std::uint8_t[bytes]());
                free.push_back(i);
            }
        }
        std::mutex mutex;
        std::vector<std::unique_ptr<std::uint8_t[]>> buffers;
        std::vector<std::size_t> free;
    };

public:
    class Lease
    {
    public:
        Lease(std::shared_ptr<State> state, std::size_t index)
            : state_(std::move(state)), index_(index)
        {
        }
        ~Lease()
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->free.push_back(index_);
        }
        Lease(const Lease&) = delete;
        Lease& operator=(const Lease&) = delete;
        std::uint8_t* data() const { return state_->buffers[index_].get(); }

    private:
        std::shared_ptr<State> state_;
        std::size_t index_;
    };

    HostPacketPool(std::size_t count, std::size_t bytes)
        : state_(std::make_shared<State>(count, bytes)), bytes_(bytes)
    {
    }

    std::unique_ptr<Lease> acquire(std::size_t bytes)
    {
        if (bytes == 0 || bytes > bytes_)
            throw std::runtime_error("MJPEG packet exceeds host staging capacity");
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->free.empty())
            throw std::runtime_error("video staging pool exhausted");
        const auto index = state_->free.back();
        auto lease = std::make_unique<Lease>(state_, index);
        state_->free.pop_back();
        return lease;
    }

private:
    std::shared_ptr<State> state_;
    std::size_t bytes_;
};

} // namespace camera_io::recording

#endif // HOST_PACKET_POOL_H
