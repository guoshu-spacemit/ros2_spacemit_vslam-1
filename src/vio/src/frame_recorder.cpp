/**
 * @file frame_recorder.cpp
 * @brief Record image-time VIO observations and replay metadata.
 */
#include "vio/frame_recorder.h"

#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <iomanip>
#include <stdexcept>

namespace vio
{
FrameRecorder::FrameRecorder(const std::string& path, std::size_t capacity,
    const std::string& world_frame, const std::string& rig_frame)
    : path_(path), queue_(capacity)
{
    if (capacity == 0 || std::filesystem::exists(path))
        throw std::runtime_error("VIO record path already exists or queue capacity is zero");
    output_.open(path);
    if (!output_)
        throw std::runtime_error("Cannot open VIO record file: " + path);
    output_ << "# world_frame=" << world_frame << " rig_frame=" << rig_frame
            << " pose=world_from_rig quaternion=xyzw\n"
            << "source_frame_index,image_timestamp_ns,estimate_timestamp_ns,status,"
                "tx,ty,tz,qx,qy,qz,qw,track_duration_ms\n"
            << std::setprecision(17);
    output_.flush();
    if (!output_)
        throw std::runtime_error("Cannot write VIO record header: " + path);
    worker_ = std::thread(&FrameRecorder::run, this);
}

FrameRecorder::~FrameRecorder()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    ready_.notify_one();
    worker_.join();
}

bool FrameRecorder::enqueue(const FrameRecord& record) noexcept
{
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || failed_.load() || size_ == queue_.size())
    {
        ++dropped_;
        return false;
    }
    queue_[(head_ + size_) % queue_.size()] = record;
    ++size_;
    lock.unlock();
    ready_.notify_one();
    return true;
}

void FrameRecorder::run() noexcept
{
    try
    {
        for (;;)
        {
            std::array<FrameRecord, 128> batch;
            std::size_t count = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return stopping_ || size_ > 0; });
                if (size_ == 0 && stopping_)
                    break;
                while (size_ > 0 && count < batch.size())
                {
                    batch[count++] = queue_[head_];
                    head_ = (head_ + 1) % queue_.size();
                    --size_;
                }
            }
            for (std::size_t i = 0; i < count; ++i)
            {
                const auto& r = batch[i];
                const char* status = r.status == FrameRecord::Status::Ok ? "ok"
                    : r.status == FrameRecord::Status::InvalidPose       ? "invalid_pose"
                                                                            : "track_error";
                output_ << r.source_frame_index << ',' << r.image_timestamp_ns << ','
                        << r.estimate_timestamp_ns << ',' << status;
                if (r.status == FrameRecord::Status::Ok)
                {
                    for (double value : r.translation)
                        output_ << ',' << value;
                    for (double value : r.quaternion_xyzw)
                        output_ << ',' << value;
                }
                else
                    output_ << ",,,,,,,";
                output_ << ',' << r.track_duration_ms << '\n';
            }
            output_.flush();
            if (!output_)
                throw std::runtime_error("VIO record write failed");
            written_ += count;
        }
        output_.close();
        if (!output_)
            failed_ = true;
    }
    catch (...)
    {
        failed_ = true;
    }
    // A missing summary (e.g. SIGKILL/disk full) also means the run is unverified.
    std::ofstream summary(path_ + ".summary.json");
    summary << "{\"written\":" << written_ << ",\"dropped\":" << dropped_.load()
            << ",\"write_failed\":" << (failed_.load() ? "true" : "false") << "}\n";
}
} // namespace vio
