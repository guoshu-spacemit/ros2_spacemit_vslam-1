#ifndef SESSION_REPLAY_CONTROL_H
#define SESSION_REPLAY_CONTROL_H
#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>

namespace camera_io::replay
{

// Called under the publisher's mutex. Commands never advance the input cursor;
// only a completed target can consume a step or reach a fast-forward goal.
class Control
{
public:
    enum class Mode
    {
        Initializing,
        Playing,
        PauseRequested,
        Paused,
        Stepping,
        FastForward,
        Finishing,
        Finished,
        Error
    };

    explicit Control(bool start_paused = false) : start_paused_(start_paused) {}

    bool command(const std::string& command, std::int64_t current_stamp, std::int64_t last_stamp,
        std::int64_t advance_ns)
    {
        if (terminal())
            return false;
        if (mode_ == Mode::Initializing)
        {
            if (command == "toggle")
                start_paused_ = !start_paused_;
            else if (command == "pause" || command == "resume")
                start_paused_ = command == "pause";
            else
                return false;
            return true;
        }
        if (command == "toggle" || command == "pause" || command == "resume")
        {
            const bool resume = command == "resume" ||
                (command == "toggle" && (mode_ == Mode::Paused || mode_ == Mode::PauseRequested));
            mode_ = resume ? Mode::Playing : Mode::PauseRequested;
            goal_.reset();
            return true;
        }
        if (command == "step" && mode_ == Mode::Paused)
        {
            mode_ = Mode::Stepping;
            return true;
        }
        if (command == "fast_forward" && (mode_ == Mode::Paused || mode_ == Mode::Playing) &&
            advance_ns > 0 && current_stamp <= last_stamp)
        {
            return_paused_ = mode_ == Mode::Paused;
            // Saturate without overflowing the source timestamp.
            goal_ = current_stamp + std::min(advance_ns, last_stamp - current_stamp);
            mode_ = Mode::FastForward;
            return true;
        }
        return false;
    }

    void start()
    {
        if (mode_ == Mode::Initializing)
            mode_ = start_paused_ ? Mode::Paused : Mode::Playing;
    }

    void boundary()
    {
        if (mode_ == Mode::PauseRequested)
            mode_ = Mode::Paused;
    }

    void target_complete(std::int64_t stamp, bool last)
    {
        if (last)
        {
            mode_ = Mode::Finishing;
            goal_.reset();
        }
        else if (mode_ == Mode::Stepping)
        {
            mode_ = Mode::Paused;
        }
        else if (mode_ == Mode::FastForward && stamp >= *goal_)
        {
            mode_ = return_paused_ ? Mode::Paused : Mode::Playing;
            goal_.reset();
        }
        boundary();
    }

    void finish(bool error)
    {
        mode_ = error ? Mode::Error : Mode::Finished;
        goal_.reset();
    }

    Mode mode() const { return mode_; }
    bool paused() const { return mode_ == Mode::Paused; }
    bool paced() const { return mode_ == Mode::Playing; }
    bool terminal() const
    {
        return mode_ == Mode::Finishing || mode_ == Mode::Finished || mode_ == Mode::Error;
    }
    const char* name() const
    {
        switch (mode_)
        {
        case Mode::Initializing:
            return "initializing";
        case Mode::Playing:
            return "playing";
        case Mode::PauseRequested:
            return "pause_requested";
        case Mode::Paused:
            return "paused";
        case Mode::Stepping:
            return "stepping";
        case Mode::FastForward:
            return "fast_forward";
        case Mode::Finishing:
            return "finishing";
        case Mode::Finished:
            return "finished";
        case Mode::Error:
            return "error";
        }
        return "error";
    }

private:
    Mode mode_{Mode::Initializing};
    bool start_paused_{false}, return_paused_{false};
    std::optional<std::int64_t> goal_;
};

} // namespace camera_io::replay

#endif // SESSION_REPLAY_CONTROL_H
