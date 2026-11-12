// Transport-independent Seeker event and image association.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_SEEKER_STREAM_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_SEEKER_STREAM_H_

#include "spacemit_camera/quad_camera.h"
#include "spacemit_camera/seeker_protocol.h"

#include <mutex>

namespace camera_sdk
{

struct SeekerStreamConfig
{
    SeekerEnvelopePolicy envelope_policy = SeekerEnvelopePolicy::require_v1;
    bool map_host_time = true;
    double clock_alpha = 0.00001;
    std::uint64_t clock_reset_threshold_ns = 50000000;
    std::array<QuadPosition, 4> positions{QuadPosition::front_left, QuadPosition::front_right,
        QuadPosition::rear_right, QuadPosition::rear_left};
};

struct SeekerEventFrame
{
    SeekerEvent event; // All original fields, including device timestamp, sequence and flags.
    SeekerEventHeader output_header; // Official callback convention; IMU sequence is 1.
    bool time_mapped = false;
};

struct SeekerImageFrame
{
    QuadImage image; // Borrowed from the caller, which must retain its owning QuadFrame/buffer.
    std::array<QuadView, 4> views{};
    SeekerEventHeader device_header; // Latest received UVC information, including its sequence.
    SeekerEventHeader output_header; // Image sequence starts at 1, as in the official SDK.
    bool timestamp_valid = false;    // At least one UVC event has been received.
    bool time_mapped = false;
};

// Transport-independent Seeker four-camera processing, shared by live adapters
// and recorded-data replay. No device I/O, clock reads, worker threads or queues.
// Serialize event submissions in receive order. Event and image calls may run
// concurrently; an image snapshots the latest UVC event and clock mapping.
// Compatibility target: official Seeker SDK 4985682 x86 callback behavior.
class SeekerStream
{
public:
    explicit SeekerStream(SeekerStreamConfig config = {});

    // host_reference is an optional sec/nsec pair from the caller's host clock,
    // sampled for this IMU event (or restored from a recording). Sequence ignored.
    // Null keeps the previous mapping; with no mapping, device time is returned.
    // Failure leaves output and stream state unchanged. Sensor flags/units and
    // startup timestamp changes are preserved, not filtered or synthesized.
    bool process_event(const std::uint8_t* data, std::size_t size, SeekerEventFrame& output,
        const SeekerEventHeader* host_reference = nullptr, std::string* error = nullptr);
    bool process_image(
        const QuadImage& image, SeekerImageFrame& output, std::string* error = nullptr);
    void reset(); // New session: forget clock, latest UVC event and image counter.

private:
    SeekerStreamConfig config_;
    std::mutex mutex_;
    double offset_ns_ = 0;
    bool clock_ready_ = false;
    bool uvc_ready_ = false;
    SeekerEventHeader uvc_header_;
    std::uint32_t image_sequence_ = 0;
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_SEEKER_STREAM_H_
