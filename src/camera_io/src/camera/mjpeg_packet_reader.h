#ifndef MJPEG_PACKET_READER_H
#define MJPEG_PACKET_READER_H
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/mathematics.h>
}

namespace camera_io
{

// Owns one compressed packet. Pixels are never decoded or modified on the CPU.
// A returned view is valid until next() or destruction. PTS remains in the
// original source clock; only its unit is converted to the MPP microseconds.
class MjpegPacketReader
{
public:
    struct Packet
    {
        const std::uint8_t* data{nullptr};
        std::uint32_t bytes{0};
        std::uint64_t pts_us{0};
    };

    MjpegPacketReader(
        const std::string& path, int width, int height, std::function<bool()> cancelled = {})
        : cancelled_(std::move(cancelled))
    {
        try
        {
            format_ = avformat_alloc_context();
            packet_ = av_packet_alloc();
            if (!format_ || !packet_)
                throw std::bad_alloc();
            format_->interrupt_callback = {interrupt, this};
            refresh_deadline();
            // Explicit local MP4 demux only. No probe decoding, conversion,
            // network protocols, bitstream filter or recorder-side index.
            AVDictionary* options = nullptr;
            av_dict_set(&options, "protocol_whitelist", "file", 0);
            const int result =
                avformat_open_input(&format_, path.c_str(), av_find_input_format("mov"), &options);
            av_dict_free(&options);
            if (result < 0)
                fail("open original MJPEG MP4", result);
            for (unsigned i = 0; i < format_->nb_streams; ++i)
            {
                const auto* parameters = format_->streams[i]->codecpar;
                if (parameters->codec_type != AVMEDIA_TYPE_VIDEO)
                    continue;
                if (stream_index_ >= 0)
                    throw std::runtime_error("MJPEG MP4 must contain exactly one video stream");
                stream_index_ = static_cast<int>(i);
                if (parameters->width != width || parameters->height != height)
                    throw std::runtime_error("MP4 dimensions disagree with camera configuration");
            }
            if (stream_index_ < 0)
                throw std::runtime_error("MP4 has no video stream");
            const auto* stream = format_->streams[stream_index_];
            time_base_ = stream->time_base;
            if (time_base_.num <= 0 || time_base_.den <= 0)
                throw std::runtime_error("MP4 has invalid source time base");
            expected_packets_ = stream->nb_frames;
        }
        catch (...)
        {
            close();
            throw;
        }
    }

    ~MjpegPacketReader() { close(); }
    MjpegPacketReader(const MjpegPacketReader&) = delete;
    MjpegPacketReader& operator=(const MjpegPacketReader&) = delete;

    bool next(Packet* view)
    {
        if (!view)
            throw std::invalid_argument("null MJPEG packet view");
        *view = {};
        for (;;)
        {
            if (is_cancelled())
                throw std::runtime_error("MJPEG MP4 read cancelled");
            av_packet_unref(packet_);
            refresh_deadline();
            const int result = av_read_frame(format_, packet_);
            if (result == AVERROR_EOF)
            {
                if (format_->pb && format_->pb->error < 0 && format_->pb->error != AVERROR_EOF)
                    fail("read original MJPEG MP4", format_->pb->error);
                if (expected_packets_ > 0 && packets_ != expected_packets_)
                    throw std::runtime_error("MP4 ended before declared video packet count");
                return false;
            }
            if (result < 0)
                fail("read original MJPEG packet", result);
            if (packet_->stream_index != stream_index_)
                continue;
            // Some original recordings use an mp4v sample tag for JPEG. Check
            // the actual payload instead of trusting that container codec tag.
            if ((packet_->flags & AV_PKT_FLAG_CORRUPT) || packet_->size < 4 || !packet_->data ||
                packet_->data[0] != 0xff || packet_->data[1] != 0xd8 ||
                packet_->data[packet_->size - 2] != 0xff ||
                packet_->data[packet_->size - 1] != 0xd9)
                throw std::runtime_error("MP4 packet is corrupt or not a complete original JPEG");
            if (packet_->pts == AV_NOPTS_VALUE || packet_->pts < 0)
                throw std::runtime_error("MJPEG MP4 packet has no nonnegative source PTS");
            const auto pts = av_rescale_q(packet_->pts, time_base_, AVRational{1, 1000000});
            if (pts < 0 || pts == std::numeric_limits<std::int64_t>::max() ||
                (packets_ && pts <= last_pts_us_))
                throw std::runtime_error(
                    "MJPEG MP4 PTS is not strictly increasing in microseconds");
            view->data = packet_->data;
            view->bytes = static_cast<std::uint32_t>(packet_->size);
            view->pts_us = static_cast<std::uint64_t>(pts);
            last_pts_us_ = pts;
            ++packets_;
            return true;
        }
    }

private:
    using Clock = std::chrono::steady_clock;

    bool is_cancelled() const { return cancelled_ && cancelled_(); }

    static int interrupt(void* opaque) noexcept
    {
        auto* self = static_cast<MjpegPacketReader*>(opaque);
        try
        {
            return self->is_cancelled() || Clock::now() >= self->deadline_;
        }
        catch (...)
        {
            return 1;
        }
    }

    void refresh_deadline() { deadline_ = Clock::now() + std::chrono::seconds(5); }

    [[noreturn]] static void fail(const char* operation, int code)
    {
        char message[AV_ERROR_MAX_STRING_SIZE]{};
        av_strerror(code, message, sizeof(message));
        throw std::runtime_error(std::string(operation) + ": " + message);
    }

    void close() noexcept
    {
        av_packet_free(&packet_);
        avformat_close_input(&format_);
    }

    AVFormatContext* format_{nullptr};
    AVPacket* packet_{nullptr};
    int stream_index_{-1};
    AVRational time_base_{};
    std::int64_t expected_packets_{0};
    std::int64_t packets_{0};
    std::int64_t last_pts_us_{-1};
    std::function<bool()> cancelled_;
    Clock::time_point deadline_;
};

} // namespace camera_io

#endif // MJPEG_PACKET_READER_H
