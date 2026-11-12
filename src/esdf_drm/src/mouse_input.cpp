/**
 * @file mouse_input.cpp
 * @brief Translate pointing-device events into map-view camera controls.
 */
#include "esdf_drm/mouse_input.h"

#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace esdf_drm
{
namespace
{

template <std::size_t Size>
bool bit_is_set(const std::array<unsigned long, Size>& bits, unsigned int bit)
{
    constexpr unsigned int kBitsPerWord = sizeof(unsigned long) * 8U;
    const unsigned int word = bit / kBitsPerWord;
    return word < bits.size() && (bits[word] & (1UL << (bit % kBitsPerWord))) != 0UL;
}

bool is_mouse(int file_descriptor)
{
    std::array<unsigned long, (EV_MAX + 8U * sizeof(unsigned long)) / (8U * sizeof(unsigned long))>
        event_bits{};
    std::array<unsigned long, (REL_MAX + 8U * sizeof(unsigned long)) / (8U * sizeof(unsigned long))>
        relative_bits{};
    std::array<unsigned long, (KEY_MAX + 8U * sizeof(unsigned long)) / (8U * sizeof(unsigned long))>
        key_bits{};
    if (ioctl(file_descriptor, EVIOCGBIT(0, sizeof(event_bits)), event_bits.data()) < 0 ||
        !bit_is_set(event_bits, EV_REL) || !bit_is_set(event_bits, EV_KEY) ||
        ioctl(file_descriptor, EVIOCGBIT(EV_REL, sizeof(relative_bits)), relative_bits.data()) <
            0 ||
        ioctl(file_descriptor, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits.data()) < 0)
    {
        return false;
    }
    return bit_is_set(relative_bits, REL_X) && bit_is_set(relative_bits, REL_Y) &&
        bit_is_set(key_bits, BTN_LEFT) && bit_is_set(key_bits, BTN_RIGHT);
}

int open_mouse(const std::string& requested, std::string* selected)
{
    const auto try_open = [&](const std::string& path)
    {
        const int descriptor = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (descriptor >= 0 && is_mouse(descriptor))
        {
            if (selected != nullptr)
            {
                *selected = path;
            }
            return descriptor;
        }
        if (descriptor >= 0)
        {
            close(descriptor);
        }
        return -1;
    };
    if (!requested.empty())
    {
        return try_open(requested);
    }
    for (int index = 0; index < 32; ++index)
    {
        const int descriptor = try_open("/dev/input/event" + std::to_string(index));
        if (descriptor >= 0)
        {
            return descriptor;
        }
    }
    return -1;
}

} // namespace

MouseInput::MouseInput(std::string device, Callback callback)
    : requested_device_(std::move(device)), callback_(std::move(callback))
{
}

MouseInput::~MouseInput() { stop(); }

bool MouseInput::start(std::string* selected_device, std::string* error)
{
    if (thread_.joinable())
    {
        return true;
    }
    file_descriptor_ = open_mouse(requested_device_, selected_device);
    if (file_descriptor_ < 0)
    {
        if (error != nullptr)
        {
            *error = requested_device_.empty()
                ? "no relative mouse input device found"
                : "cannot open mouse input " + requested_device_ + ": " + std::strerror(errno);
        }
        return false;
    }
    stopping_.store(false);
    thread_ = std::thread(&MouseInput::run, this);
    return true;
}

void MouseInput::stop()
{
    stopping_.store(true);
    if (thread_.joinable())
    {
        thread_.join();
    }
    if (file_descriptor_ >= 0)
    {
        close(file_descriptor_);
        file_descriptor_ = -1;
    }
}

void MouseInput::run()
{
    bool left_down = false;
    bool right_down = false;
    MouseInputEvent pending;
    while (!stopping_.load())
    {
        pollfd descriptor{file_descriptor_, POLLIN, 0};
        const int poll_result = poll(&descriptor, 1, 100);
        if (poll_result <= 0 || (descriptor.revents & POLLIN) == 0)
        {
            continue;
        }
        input_event events[32]{};
        const ssize_t bytes = read(file_descriptor_, events, sizeof(events));
        if (bytes <= 0)
        {
            continue;
        }
        const std::size_t count = static_cast<std::size_t>(bytes) / sizeof(events[0]);
        for (std::size_t index = 0; index < count; ++index)
        {
            const input_event& event = events[index];
            if (event.type == EV_REL)
            {
                if (event.code == REL_X)
                    pending.relative_x += event.value;
                else if (event.code == REL_Y)
                    pending.relative_y += event.value;
                else if (event.code == REL_WHEEL)
                    pending.wheel += event.value;
            }
            else if (event.type == EV_KEY)
            {
                if (event.code == BTN_LEFT)
                    left_down = event.value != 0;
                else if (event.code == BTN_RIGHT)
                    right_down = event.value != 0;
                else if (event.code == BTN_MIDDLE && event.value == 1)
                    pending.reset = true;
            }
            else if (event.type == EV_SYN && event.code == SYN_REPORT)
            {
                pending.left_down = left_down;
                pending.right_down = right_down;
                if ((pending.relative_x != 0 || pending.relative_y != 0 || pending.wheel != 0 ||
                        pending.reset) &&
                    callback_)
                {
                    callback_(pending);
                }
                pending = {};
            }
        }
    }
}

} // namespace esdf_drm
