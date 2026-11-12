// Frame-driven exposure alignment and restoration controls.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_STARTUP_ALIGNMENT_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_STARTUP_ALIGNMENT_H_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace camera_sdk
{

// Controls are deliberately separate from the frame-driven state machine.
// Implementations must save the current mode/value before changing either.
class ExposureControl
{
public:
    virtual ~ExposureControl() = default;
    virtual void set_exposure_us(int value) = 0;
    virtual void restore() = 0;
};

std::unique_ptr<ExposureControl> open_v4l2_exposure(const std::string& device);

class StartupAlignment
{
public:
    struct Config
    {
        bool enabled{false};
        int timeout_ms{10000};
        unsigned frames_per_step{4};
        int tolerance_us{100};
        std::vector<int> exposure_us{
            2000, 7000, 3500, 11000, 5000, 15000, 2500, 9000, 4000, 13000, 6000, 18000};
    };
    using Log = std::function<void(const std::string&)>;
    StartupAlignment(Config config, std::unique_ptr<ExposureControl> control, Log log);
    ~StartupAlignment();
    // Called once per decoded metadata frame, never from an arrival-time timer.
    bool observe(std::uint32_t exposure_us, bool matched);
    static void validate(const Config& config);

private:
    enum class State
    {
        Acquiring,
        Restoring,
        Ready,
        Failed
    };
    void fail(const std::string& reason);
    Config config_;
    std::unique_ptr<ExposureControl> control_;
    Log log_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread watchdog_;
    bool stopping_{false};
    State state_{State::Acquiring};
    std::size_t step_{0};
    unsigned observed_{0};
    std::int64_t stable_duration_{-1};
    std::int64_t previous_duration_{-1};
};
} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_STARTUP_ALIGNMENT_H_
