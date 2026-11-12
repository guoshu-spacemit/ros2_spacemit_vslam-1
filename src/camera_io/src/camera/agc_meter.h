#ifndef AGC_METER_H
#define AGC_METER_H
#include <cstddef>
#include <cstdint>
#include <stdexcept>

struct MeterResult
{
    double mean = 0;
    double clipped = 0;
};

inline MeterResult measure_eye(const std::uint8_t* data, std::size_t size, unsigned stride,
    unsigned image_width, unsigned height, unsigned metadata_width, unsigned eye)
{
    if (!data || eye > 1 || image_width <= metadata_width || (image_width - metadata_width) % 2 ||
        stride < image_width || height < 10 || std::uint64_t(stride) * height > size)
        throw std::invalid_argument("invalid packed NV12 metering layout");
    const unsigned width = (image_width - metadata_width) / 2;
    if (width < 10)
        throw std::invalid_argument("empty eye ROI");
    const unsigned base = metadata_width + eye * width;
    std::uint64_t sum = 0, count = 0, clipped = 0;
    for (std::size_t y = height / 10; y < std::uint64_t(height) * 9 / 10; y += 8)
        for (std::size_t x = width / 10; x < std::uint64_t(width) * 9 / 10; x += 8)
        {
            const auto value = data[std::size_t(y) * stride + base + x];
            sum += value;
            clipped += value >= 250;
            ++count;
        }
    if (!count)
        throw std::invalid_argument("empty metering ROI");
    return {double(sum) / count, double(clipped) / count};
}

#endif // AGC_METER_H
