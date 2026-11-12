#ifndef CENTER_DEPTH_OVERLAY_H
#define CENTER_DEPTH_OVERLAY_H
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace las2
{

struct DepthRoi
{
    int x{0};
    int y{0};
    int width{1};
    int height{1};
};

struct CenterDepthSample
{
    int u{0};
    int v{0};
    DepthRoi roi;
    float z_m{std::numeric_limits<float>::quiet_NaN()};
    bool valid{false};
};

inline DepthRoi depth_roi_at(int width, int height, int u, int v, int size)
{
    if (width <= 0 || height <= 0 || size < 1 || size > width || size > height || u < 0 ||
        u >= width || v < 0 || v >= height)
    {
        throw std::invalid_argument("invalid depth ROI geometry");
    }
    return {std::clamp(u - size / 2, 0, width - size), std::clamp(v - size / 2, 0, height - size),
        size, size};
}

// The depth view is rectified axial Z in meters, with a stride in bytes.
// Invalid pixels are skipped; a sparse region is displayed as unavailable.
inline CenterDepthSample sample_depth_roi(const void* data, int width, int height, int stride_bytes,
    int u, int v, int size, double minimum_valid_fraction)
{
    CenterDepthSample sample;
    if (width <= 0 || height <= 0 || size < 1 || size > width || size > height || u < 0 ||
        u >= width || v < 0 || v >= height || !std::isfinite(minimum_valid_fraction) ||
        minimum_valid_fraction < 0.0 || minimum_valid_fraction > 1.0)
    {
        return sample;
    }
    sample.u = u;
    sample.v = v;
    sample.roi = depth_roi_at(width, height, u, v, size);
    if (!data || stride_bytes <= 0 ||
        static_cast<std::size_t>(stride_bytes) < static_cast<std::size_t>(width) * sizeof(float))
    {
        return sample;
    }

    double sum = 0.0;
    int valid_pixels = 0;
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (int row = sample.roi.y; row < sample.roi.y + sample.roi.height; ++row)
    {
        for (int column = sample.roi.x; column < sample.roi.x + sample.roi.width; ++column)
        {
            float depth = 0.0F;
            std::memcpy(&depth,
                bytes + static_cast<std::size_t>(row) * stride_bytes +
                    static_cast<std::size_t>(column) * sizeof(float),
                sizeof(depth));
            if (std::isfinite(depth) && depth > 0.0F)
            {
                sum += depth;
                ++valid_pixels;
            }
        }
    }
    const int required =
        std::max(1, static_cast<int>(std::ceil(size * size * minimum_valid_fraction)));
    if (valid_pixels >= required)
    {
        sample.z_m = static_cast<float>(sum / valid_pixels);
        sample.valid = true;
    }
    return sample;
}

// A small even-aligned mask for one panel of the packed NV12 preview.
// Zero keeps the original pixel; nonzero is a limited-range Y value.
struct CenterDepthMask
{
    int x{0};
    int y{0};
    int width{0};
    int height{0};
    int first_panel{1};
    int panel_count{1};
    std::uint8_t chroma_u{32};
    std::uint8_t chroma_v{160};
    std::vector<std::uint8_t> pixels;
};

namespace center_depth_detail
{

inline std::array<std::uint8_t, 7> glyph(char c)
{
    switch (c)
    {
    case 'm':
        return {0, 0, 26, 21, 21, 21, 21};
    case '0':
        return {14, 17, 19, 21, 25, 17, 14};
    case '1':
        return {4, 12, 4, 4, 4, 4, 14};
    case '2':
        return {14, 17, 1, 2, 4, 8, 31};
    case '3':
        return {30, 1, 1, 14, 1, 1, 30};
    case '4':
        return {2, 6, 10, 18, 31, 2, 2};
    case '5':
        return {31, 16, 16, 30, 1, 1, 30};
    case '6':
        return {14, 16, 16, 30, 17, 17, 14};
    case '7':
        return {31, 1, 2, 4, 8, 8, 8};
    case '8':
        return {14, 17, 17, 14, 17, 17, 14};
    case '9':
        return {14, 17, 17, 15, 1, 1, 14};
    case '.':
        return {0, 0, 0, 0, 0, 6, 6};
    case '-':
        return {0, 0, 0, 31, 0, 0, 0};
    default:
        return {};
    }
}

struct TextGeometry
{
    int characters{0};
    int scale{0};
    int x{0};
    int y{0};
    int width{0};
    int height{0};
};

inline TextGeometry label_geometry(
    int panel_width, int height, const char* label, const CenterDepthSample& sample)
{
    TextGeometry text;
    text.characters = static_cast<int>(std::strlen(label));
    text.scale = std::min({2, (panel_width - 8) / (text.characters * 6 - 1), (height - 8) / 7});
    if (text.scale > 0)
    {
        text.width = (text.characters * 6 - 1) * text.scale;
        text.height = 7 * text.scale;
        text.x = (panel_width - text.width) / 2;
        text.y = height - text.height - 6;
        if (sample.roi.width > 1 && sample.roi.x < text.x + text.width + 4 &&
            sample.roi.x + sample.roi.width > text.x - 4 &&
            sample.roi.y < text.y + text.height + 2 &&
            sample.roi.y + sample.roi.height > text.y - 2)
        {
            text.y = 6;
        }
    }
    return text;
}

inline void initialize_mask(
    CenterDepthMask& mask, int x, int y, int width, int height, int panel_width, int panel_height)
{
    mask.x = std::max(0, x) & ~1;
    mask.y = std::max(0, y) & ~1;
    const int x1 = std::min(panel_width, (x + width + 1) & ~1);
    const int y1 = std::min(panel_height, (y + height + 1) & ~1);
    mask.width = std::max(0, x1 - mask.x);
    mask.height = std::max(0, y1 - mask.y);
    mask.pixels.assign(static_cast<std::size_t>(mask.width) * mask.height, 0);
}

class MaskCanvas
{
public:
    explicit MaskCanvas(CenterDepthMask& mask) : mask_(mask) {}

    void rectangle(int x, int y, int width, int height, std::uint8_t luma)
    {
        const int x0 = std::max(mask_.x, x);
        const int y0 = std::max(mask_.y, y);
        const int x1 = std::min(mask_.x + mask_.width, x + width);
        const int y1 = std::min(mask_.y + mask_.height, y + height);
        for (int row = y0; row < y1 && x0 < x1; ++row)
        {
            std::memset(mask_.pixels.data() +
                    static_cast<std::size_t>(row - mask_.y) * mask_.width + x0 - mask_.x,
                luma, x1 - x0);
        }
    }

private:
    CenterDepthMask& mask_;
};

inline void draw_roi_box(MaskCanvas& canvas, const CenterDepthSample& sample)
{
    const int x = sample.roi.x;
    const int y = sample.roi.y;
    const int width = sample.roi.width;
    const int height = sample.roi.height;
    canvas.rectangle(x - 1, y - 1, width + 2, 1, 16);
    canvas.rectangle(x - 1, y + height, width + 2, 1, 16);
    canvas.rectangle(x - 1, y, 1, height, 16);
    canvas.rectangle(x + width, y, 1, height, 16);
    canvas.rectangle(x, y, width, 1, 210);
    canvas.rectangle(x, y + height - 1, width, 1, 210);
    canvas.rectangle(x, y, 1, height, 210);
    canvas.rectangle(x + width - 1, y, 1, height, 210);
}

inline void draw_label(MaskCanvas& canvas, const TextGeometry& text, const char* label)
{
    if (text.scale < 1)
        return;
    canvas.rectangle(text.x - 4, text.y - 2, text.width + 8, text.height + 4, 16);
    for (int character = 0; character < text.characters; ++character)
    {
        const auto rows = glyph(label[character]);
        for (int row = 0; row < 7; ++row)
        {
            for (int column = 0; column < 5; ++column)
            {
                if ((rows[row] & (1U << (4 - column))) != 0)
                {
                    canvas.rectangle(text.x + (character * 6 + column) * text.scale,
                        text.y + row * text.scale, text.scale, text.scale, 210);
                }
            }
        }
    }
}

} // namespace center_depth_detail

// Both masks target only the depth half. The box exactly matches the sampled
// region, and the metric distance label stays at the bottom of that half.
inline std::array<CenterDepthMask, 2> make_center_depth_masks(
    int panel_width, int height, const CenterDepthSample& sample)
{
    if (panel_width < 2 || height < 2 || panel_width % 2 != 0 || height % 2 != 0 || sample.u < 0 ||
        sample.u >= panel_width || sample.v < 0 || sample.v >= height || sample.roi.width < 1 ||
        sample.roi.width != sample.roi.height || sample.roi.width > panel_width ||
        sample.roi.height > height)
    {
        throw std::invalid_argument("invalid center depth mask geometry");
    }
    const auto expected = depth_roi_at(panel_width, height, sample.u, sample.v, sample.roi.width);
    if (sample.roi.x != expected.x || sample.roi.y != expected.y)
        throw std::invalid_argument("depth HUD box differs from sampled ROI");

    std::array<CenterDepthMask, 2> masks;
    const int box_size = sample.roi.width;
    center_depth_detail::initialize_mask(masks[0], sample.roi.x - 1, sample.roi.y - 1, box_size + 2,
        box_size + 2, panel_width, height);
    center_depth_detail::MaskCanvas box_canvas(masks[0]);
    center_depth_detail::draw_roi_box(box_canvas, sample);

    char label[64];
    if (sample.valid && std::isfinite(sample.z_m) && sample.z_m > 0.0F)
        std::snprintf(label, sizeof(label), "%.2f m", static_cast<double>(sample.z_m));
    else
        std::snprintf(label, sizeof(label), "-- m");
    const auto text = center_depth_detail::label_geometry(panel_width, height, label, sample);
    if (text.scale > 0)
        center_depth_detail::initialize_mask(
            masks[1], text.x - 4, text.y - 2, text.width + 8, text.height + 4, panel_width, height);
    center_depth_detail::MaskCanvas label_canvas(masks[1]);
    center_depth_detail::draw_label(label_canvas, text, label);
    return masks;
}

} // namespace las2

#endif // CENTER_DEPTH_OVERLAY_H
