// Align startup exposures and preserve timestamp-restoration state.

#include "spacemit_camera/startup_alignment.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace camera_sdk
{
namespace
{
class V4l2Exposure final : public ExposureControl
{
public:
    explicit V4l2Exposure(const std::string& device)
    {
        fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("open exposure controls: " + std::string(strerror(errno)));
        try
        {
            mode_ = get(V4L2_CID_EXPOSURE_AUTO);
            value_ = get(V4L2_CID_EXPOSURE_ABSOLUTE);
            range_.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            call(VIDIOC_QUERYCTRL, &range_);
            if (range_.flags & (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY))
                throw std::runtime_error("exposure control is disabled/read-only");
        }
        catch (...)
        {
            close(fd_);
            throw;
        }
    }
    ~V4l2Exposure() override { close(fd_); }
    void set_exposure_us(int us) override
    {
        // Standard V4L2 EXPOSURE_ABSOLUTE units are 100 microseconds.
        const int value = (us + 50) / 100;
        if (value < range_.minimum || value > range_.maximum ||
            (value - range_.minimum) % std::max(1, range_.step) != 0)
            throw std::runtime_error("startup exposure outside V4L2 control range/step");
        touched_ = true;
        set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        set(V4L2_CID_EXPOSURE_ABSOLUTE, value);
    }
    void restore() override
    {
        if (!touched_)
            return;
        // Restore the manual value before restoring auto mode; auto mode may make
        // EXPOSURE_ABSOLUTE inactive. Attempt mode restoration even if value fails.
        std::string error;
        try
        {
            set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
            set(V4L2_CID_EXPOSURE_ABSOLUTE, value_);
        }
        catch (const std::exception& e)
        {
            error = e.what();
        }
        try
        {
            set(V4L2_CID_EXPOSURE_AUTO, mode_);
        }
        catch (const std::exception& e)
        {
            error += e.what();
        }
        if (!error.empty())
            throw std::runtime_error("restore exposure: " + error);
        touched_ = false;
    }

private:
    void call(unsigned long request, void* argument)
    {
        int result;
        do
        {
            result = ioctl(fd_, request, argument);
        } while (result < 0 && errno == EINTR);
        if (result < 0)
            throw std::runtime_error("V4L2 exposure ioctl: " + std::string(strerror(errno)));
    }
    int get(unsigned id)
    {
        v4l2_control c{};
        c.id = id;
        call(VIDIOC_G_CTRL, &c);
        return c.value;
    }
    void set(unsigned id, int value)
    {
        v4l2_control c{};
        c.id = id;
        c.value = value;
        call(VIDIOC_S_CTRL, &c);
    }
    int fd_{-1}, mode_{0}, value_{0};
    bool touched_{false};
    v4l2_queryctrl range_{};
};
} // namespace

std::unique_ptr<ExposureControl> open_v4l2_exposure(const std::string& device)
{
    return std::make_unique<V4l2Exposure>(device);
}

void StartupAlignment::validate(const Config& c)
{
    if (c.timeout_ms < 100 || c.timeout_ms > 120000 || c.frames_per_step < 1 ||
        c.frames_per_step > 64 || c.tolerance_us < 0 || c.tolerance_us > 1000 ||
        c.exposure_us.size() < 3 || c.exposure_us.size() > 64)
        throw std::invalid_argument("invalid startup_alignment configuration");
    for (int us : c.exposure_us)
        if (us < 100 || us > 1000000)
            throw std::invalid_argument("invalid startup exposure_us");
}

StartupAlignment::StartupAlignment(Config config, std::unique_ptr<ExposureControl> control, Log log)
    : config_(std::move(config)), control_(std::move(control)), log_(std::move(log))
{
    validate(config_);
    try
    {
        control_->set_exposure_us(config_.exposure_us.front());
        log_("acquiring: exposure sequence started");
        watchdog_ = std::thread(
            [this]
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!cv_.wait_for(lock, std::chrono::milliseconds(config_.timeout_ms), [this]
                        { return stopping_ || state_ == State::Ready || state_ == State::Failed; }))
                    fail("timeout: publication withheld; reactivate to retry");
            });
    }
    catch (...)
    {
        try
        {
            control_->restore();
        }
        catch (const std::exception& e)
        {
            log_(e.what());
        }
        throw;
    }
}

StartupAlignment::~StartupAlignment()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    if (watchdog_.joinable())
        watchdog_.join();
    try
    {
        control_->restore();
    }
    catch (const std::exception& e)
    {
        log_(e.what());
    }
}

void StartupAlignment::fail(const std::string& reason)
{
    state_ = State::Failed;
    log_(reason);
    try
    {
        control_->restore();
    }
    catch (const std::exception& e)
    {
        log_(e.what());
    }
    cv_.notify_all();
}

bool StartupAlignment::observe(std::uint32_t duration, bool matched)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::Ready)
        return true;
    if (state_ == State::Failed)
        return false;
    try
    {
        if (state_ == State::Restoring)
        {
            observed_ = matched ? observed_ + 1 : 0;
            if (observed_ >= config_.frames_per_step)
            {
                state_ = State::Ready;
                log_("ready: original exposure restored; verified frames may publish");
                cv_.notify_all();
                return true;
            }
        }
        else if (matched)
        {
            control_->restore();
            state_ = State::Restoring;
            observed_ = 0;
            log_("matched: exposure restored; verifying continuity");
        }
        else if (previous_duration_ < 0 ||
            std::abs(static_cast<std::int64_t>(duration) - previous_duration_) >
                config_.tolerance_us)
        {
            // Advance on measured stable changes, not firmware control-unit scaling.
            // GPIO matching continues to use actual widths with its strict tolerance.
            if (stable_duration_ < 0 ||
                std::abs(static_cast<std::int64_t>(duration) - stable_duration_) >
                    config_.tolerance_us)
            {
                stable_duration_ = duration;
                observed_ = 0;
            }
            if (++observed_ >= config_.frames_per_step && step_ + 1 < config_.exposure_us.size())
            {
                previous_duration_ = stable_duration_;
                stable_duration_ = -1;
                ++step_;
                observed_ = 0;
                control_->set_exposure_us(config_.exposure_us[step_]);
                log_("exposure step=" + std::to_string(step_) +
                    " previous_measured_us=" + std::to_string(previous_duration_));
            }
        }
        else
        {
            observed_ = 0;
        }
    }
    catch (const std::exception& e)
    {
        fail(e.what());
    }
    return false;
}
} // namespace camera_sdk
