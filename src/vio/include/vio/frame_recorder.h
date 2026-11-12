#ifndef FRAME_RECORDER_H
#define FRAME_RECORDER_H
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vio
{
struct FrameRecord
{
    std::uint32_t source_frame_index{0};
    std::int64_t image_timestamp_ns{0};
    std::int64_t estimate_timestamp_ns{0};
    enum class Status
    {
        Ok,
        InvalidPose,
        TrackError
    } status{Status::TrackError};
    std::array<double, 3> translation{};
    std::array<double, 4> quaternion_xyzw{};
    double track_duration_ms{0};
};

// One tracking producer; file I/O and formatting belong exclusively to the worker.
class FrameRecorder
{
public:
    FrameRecorder(const std::string& path, std::size_t capacity, const std::string& world_frame,
        const std::string& rig_frame);
    ~FrameRecorder();
    FrameRecorder(const FrameRecorder&) = delete;
    FrameRecorder& operator=(const FrameRecorder&) = delete;
    bool enqueue(const FrameRecord& record) noexcept;
    std::uint64_t dropped() const { return dropped_.load(); }
    bool failed() const { return failed_.load(); }

private:
    void run() noexcept;
    std::string path_;
    std::ofstream output_;
    std::vector<FrameRecord> queue_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::size_t head_{0};
    std::size_t size_{0};
    bool stopping_{false};
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<bool> failed_{false};
    std::uint64_t written_{0};
    std::thread worker_;
};
} // namespace vio

#endif // FRAME_RECORDER_H
