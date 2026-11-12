// Collect GPIO exposure edges and match bounded camera sequences.

#include "spacemit_camera/gpio_exposure.h"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <system_error>
#include <unistd.h>

namespace camera_sdk
{
namespace
{
std::string describe_failure(const std::exception_ptr& failure)
{
    try
    {
        std::rethrow_exception(failure);
    }
    catch (const std::exception& error)
    {
        return error.what();
    }
    catch (...)
    {
        return "unknown GPIO reader error";
    }
}
} // namespace

std::int64_t GpioExposure::monotonic_ns()
{
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return std::int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
}
GpioExposure::GpioExposure(
    const std::string& chip, unsigned line, bool active_low, ExposureMatcher::Config config)
    : config_(config), matcher_(config)
{
    int chip_fd = open(chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0)
        throw std::runtime_error("GPIO open: " + std::string(strerror(errno)));
    gpio_v2_line_request req{};
    req.offsets[0] = line;
    req.num_lines = 1;
    req.event_buffer_size = config.capacity * 2;
    std::snprintf(req.consumer, sizeof(req.consumer), "camera-exposure-sync");
    req.config.flags =
        GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING;
    if (active_low)
        req.config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    int ret = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
    int error = errno;
    close(chip_fd);
    if (ret < 0)
        throw std::runtime_error(
            "GPIO request (check line ownership): " + std::string(strerror(error)));
    line_fd_ = req.fd;
    stop_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (stop_fd_ < 0)
    {
        close(line_fd_);
        throw std::runtime_error("GPIO stop eventfd failed");
    }
    try
    {
        worker_ = std::thread(&GpioExposure::run, this);
    }
    catch (...)
    {
        close(line_fd_);
        close(stop_fd_);
        throw;
    }
}
GpioExposure::~GpioExposure()
{
    std::uint64_t one = 1;
    ssize_t written;
    do
    {
        written = write(stop_fd_, &one, sizeof(one));
    } while (written < 0 && errno == EINTR);
    // EAGAIN means an existing stop notification already wakes the worker.
    (void)written;
    if (worker_.joinable())
        worker_.join();
    close(line_fd_);
    close(stop_fd_);
}
void GpioExposure::invalidate(const std::string& reason)
{
    pairs_.clear();
    matcher_.reset(reason);
    ++discontinuities_;
}
void GpioExposure::run() noexcept
{
    std::uint32_t seq = 0;
    std::int64_t rise = 0, last = 0;
    try
    {
        while (true)
        {
            pollfd fds[] = {{line_fd_, POLLIN, 0}, {stop_fd_, POLLIN, 0}};
            if (poll(fds, 2, -1) < 0)
            {
                if (errno == EINTR)
                    continue;
                throw std::system_error(
                    errno, std::generic_category(), "poll GPIO exposure events");
            }
            if (fds[1].revents & POLLIN)
                return;
            if (fds[1].revents)
                throw std::runtime_error("GPIO stop event descriptor failed");
            if (fds[0].revents & POLLNVAL)
                throw std::runtime_error("GPIO line event descriptor is invalid");
            if (fds[0].revents & POLLHUP)
                throw std::runtime_error("GPIO line event stream disconnected");
            if (fds[0].revents & POLLERR)
                throw std::runtime_error("GPIO line event descriptor reported an error");
            gpio_v2_line_event events[64];
            const ssize_t bytes = read(line_fd_, events, sizeof(events));
            if (bytes < 0)
            {
                if (errno == EINTR)
                    continue;
                throw std::system_error(
                    errno, std::generic_category(), "read GPIO exposure events");
            }
            if (bytes == 0)
                throw std::runtime_error("GPIO line event stream ended");
            if (bytes % sizeof(events[0]))
                throw std::runtime_error("GPIO line event stream returned an incomplete record");
            std::lock_guard<std::mutex> lock(mutex_);
            for (std::size_t i = 0; i < std::size_t(bytes) / sizeof(events[0]); ++i)
            {
                const auto& e = events[i];
                auto t = std::int64_t(e.timestamp_ns);
                if (e.seqno != seq + 1 || t <= last)
                {
                    invalidate("gpio_sequence_gap");
                    rise = 0;
                }
                seq = e.seqno;
                last = t;
                if (e.id == GPIO_V2_LINE_EVENT_RISING_EDGE)
                {
                    if (rise)
                        invalidate("gpio_missing_end");
                    rise = t;
                }
                else if (e.id == GPIO_V2_LINE_EVENT_FALLING_EDGE)
                {
                    if (!rise)
                    {
                        invalidate("gpio_missing_start");
                        continue;
                    }
                    pairs_.push_back({rise, t});
                    rise = 0;
                    while (pairs_.size() > config_.capacity)
                        pairs_.pop_front();
                }
            }
        }
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        reader_failure_ = std::current_exception();
        pairs_.clear();
        ++discontinuities_;
    }
}
std::optional<ExposureMatcher::Exposure> GpioExposure::match(ExposureMatcher::Exposure device)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (reader_failure_)
        return {};
    return matcher_.match(device, {pairs_.begin(), pairs_.end()}, monotonic_ns());
}
std::string GpioExposure::status() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto discontinuities = " discontinuities=" + std::to_string(discontinuities_);
    if (reader_failure_)
        return "gpio_reader_failed: " + describe_failure(reader_failure_) + discontinuities;
    return matcher_.state() + discontinuities;
}
std::optional<std::string> GpioExposure::failure() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!reader_failure_)
        return std::nullopt;
    return describe_failure(reader_failure_);
}
void GpioExposure::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!reader_failure_)
        invalidate("resynchronizing");
}
} // namespace camera_sdk
