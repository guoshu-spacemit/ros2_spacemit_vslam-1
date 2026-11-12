#ifndef MOUSE_INPUT_H
#define MOUSE_INPUT_H
#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace esdf_drm
{

struct MouseInputEvent
{
    int relative_x{0};
    int relative_y{0};
    int wheel{0};
    bool left_down{false};
    bool right_down{false};
    bool reset{false};
};

class MouseInput
{
public:
    using Callback = std::function<void(const MouseInputEvent&)>;

    MouseInput(std::string device, Callback callback);
    ~MouseInput();

    MouseInput(const MouseInput&) = delete;
    MouseInput& operator=(const MouseInput&) = delete;

    bool start(std::string* selected_device, std::string* error);
    void stop();

private:
    void run();

    std::string requested_device_;
    Callback callback_;
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    int file_descriptor_{-1};
};

} // namespace esdf_drm

#endif // MOUSE_INPUT_H
