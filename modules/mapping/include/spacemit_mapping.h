// Public RGB-D mapping, distance-field and rendering interface.
#ifndef SPACEMIT_MAPPING_H
#define SPACEMIT_MAPPING_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(SPACEMIT_MAPPING_EXPORT)
#define SPACEMIT_MAPPING_API __declspec(dllexport)
#else
#define SPACEMIT_MAPPING_API __declspec(dllimport)
#endif
#else
#define SPACEMIT_MAPPING_API __attribute__((visibility("default")))
#endif

namespace spacemit_mapping::sdk
{

// Stable C++17 functional contract on the same platform/C++ runtime ABI.
// Distances are metres; timestamps are nanoseconds on the caller's shared clock.
// All mutable objects have one serialized caller. Independent objects may run
// concurrently. Input buffers are borrowed only until the synchronous call returns.
// Invalid input throws std::invalid_argument, invalid lifecycle std::logic_error,
// resource failure std::bad_alloc and backend failure std::runtime_error.
enum class Capability : std::uint64_t
{
    RgbdFusion = 1,
    DistanceQueries = 2,
    HorizontalDistance = 4,
    SurfacePoints = 8,
    Rendering = 16,
    ExclusionMask = 32
};
// Returns a bitmask of supported Capability values.
SPACEMIT_MAPPING_API std::uint64_t capabilities() noexcept;

struct Vec3
{
    float x = 0, y = 0, z = 0;
};
struct Pose
{
    // Row-major map_from_camera rotation and translation. Camera x right/y down/z forward.
    std::array<float, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<float, 3> translation{};
};
struct Camera
{
    std::uint32_t width = 0, height = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0;
};
struct ChunkKey
{
    // Opaque identity only. Never decode as a spatial index or storage address.
    std::uint64_t low = 0, high = 0;
    bool operator==(const ChunkKey& other) const noexcept
    {
        return low == other.low && high == other.high;
    }
    bool operator!=(const ChunkKey& other) const noexcept { return !(*this == other); }
};
struct ChunkInfo
{
    ChunkKey key;
    bool removed = false;
};

class Map;
class DistanceField;
class Renderer;
// Immutable owning updates. select(i) retains ONLY that item's shared payload,
// never the whole source batch. This keeps queues bounded by queued chunk count.
// Snapshots may cross threads and outlive their producer. No storage layout is public.
class SPACEMIT_MAPPING_API Snapshot
{
public:
    Snapshot() noexcept;
    ~Snapshot();
    Snapshot(const Snapshot&) noexcept;
    Snapshot& operator=(const Snapshot&) noexcept;
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot&&) noexcept;
    std::size_t size() const;
    bool empty() const;
    ChunkInfo info(std::size_t index) const;
    std::vector<ChunkInfo> changes() const;
    Snapshot select(std::size_t index) const;
    static Snapshot merge(const std::vector<Snapshot>& parts);

private:
    struct Impl;
    std::shared_ptr<const Impl> impl_;
    explicit Snapshot(std::shared_ptr<const Impl> impl) noexcept;
    friend class Map;
    friend class DistanceField;
    friend class Renderer;
};

struct MapOptions
{
    float resolution_m = .05F;
    float surface_truncation_m = .2F;
    float maximum_observation_distance_m = 7;
    float invalid_observation_decay = -1;
    std::size_t pending_change_limit = 100000;
};
struct RgbdFrame
{
    std::int64_t timestamp_ns = 0;
    Camera camera;
    Pose map_from_camera;
    const float* depth_m = nullptr;
    std::size_t depth_bytes = 0, depth_stride_bytes = 0;
    const std::uint8_t* rgb8 = nullptr;
    std::size_t rgb_bytes = 0, rgb_stride_bytes = 0;
    // Optional bytes: nonzero excludes geometry AND color. Null means all active.
    const std::uint8_t* exclude_mask = nullptr;
    std::size_t mask_bytes = 0, mask_stride_bytes = 0;
};
struct IntegrationStats
{
    std::uint64_t allocated_chunks = 0, candidate_samples = 0, changed_chunks = 0;
};
struct SurfaceOptions
{
    float minimum_confidence = 1, surface_band_m = -1;
    float minimum_height_m = -std::numeric_limits<float>::max();
    float maximum_height_m = std::numeric_limits<float>::max();
    bool interpolate_surface = true;
};
struct SurfacePoint
{
    float x = 0, y = 0, z = 0;
    std::uint32_t rgba = 0; // 0xAARRGGBB
};
class SPACEMIT_MAPPING_API Map
{
public:
    explicit Map(const MapOptions& options = {});
    ~Map();
    Map(Map&&) noexcept;
    Map& operator=(Map&&) noexcept;
    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;
    void clear();
    // Fuses one frame and returns integration counters. Frame buffers are
    // borrowed for the call. Supply the camera pose in map coordinates.
    IntegrationStats integrate(const RgbdFrame& frame);
    std::size_t chunk_count() const;
    Snapshot capture_changes() const;
    Snapshot capture_all() const;
    Snapshot capture_keys(const std::vector<ChunkKey>& keys) const;
    // Sequential last-write-wins; snapshot kind/resolution must match this map.
    void apply(const std::vector<Snapshot>& updates);
    Snapshot clear_outside(Vec3 center, float radius_m);
    // max_points==0 returns all; otherwise uniformly thin to at most that count.
    // total_available reports the unthinned count when supplied.
    std::vector<SurfacePoint> surface_points(const SurfaceOptions& options = {},
        std::size_t max_points = 0, std::size_t* total_available = nullptr) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend class DistanceField;
};

struct DistanceOptions
{
    float maximum_distance_m = 2, minimum_surface_confidence = .0001F;
};
struct DistanceRequest
{
    bool slice = false, all_input = false;
    float minimum_height_m = -.25F, maximum_height_m = .25F, output_height_m = 0;
};
struct DistanceStats
{
    std::uint64_t input_chunks = 0, changed_chunks = 0, propagated_chunks = 0, resident_chunks = 0;
};
struct DistanceSample
{
    bool known = false;
    float distance_m = std::numeric_limits<float>::quiet_NaN();
};
class SPACEMIT_MAPPING_API DistanceField
{
public:
    explicit DistanceField(float resolution_m, const DistanceOptions& options = {});
    ~DistanceField();
    DistanceField(DistanceField&&) noexcept;
    DistanceField& operator=(DistanceField&&) noexcept;
    DistanceField(const DistanceField&) = delete;
    DistanceField& operator=(const DistanceField&) = delete;
    // all_input adds every resident chunk to supplied keys; removal keys are retained.
    DistanceStats update(
        const Map& map, const std::vector<ChunkKey>& keys, const DistanceRequest& request = {});
    // Retires distance entries for removed map chunks. Call after update()
    // propagates deletion effects. Removed geometry is retained.
    void retire(
        const Map& map, const std::vector<ChunkKey>& removed, const DistanceRequest& request);
    Snapshot geometry(bool slice = false) const;
    std::size_t chunk_count(bool slice = false) const;
    // Queries map-frame positions in metres. The result has one sample per
    // position, in input order. Each sample reports whether a distance is known.
    std::vector<DistanceSample> query(const std::vector<Vec3>& positions, bool slice = false) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct RendererOptions
{
    std::string render_device = "/dev/dri/renderD128", display_device = "/dev/dri/card1";
    std::uint32_t width = 1280, height = 720;
    bool headless = true, split_view = true;
    std::size_t maximum_chunks = 4096;
    // Zero selects an implementation capacity without exposing storage block size.
    std::size_t maximum_vertices_per_chunk = 0, maximum_indices_per_chunk = 0;
    std::size_t maximum_trajectory_points = 8192;
    float point_size_pixels = 2, trajectory_step_m = .01F, camera_axis_m = .25F;
    std::array<float, 4> clear_rgba{.015F, .02F, .03F, 1};
    bool frustum_culling = true, colored_cells = false, wait_for_display = true;
    bool secondary_camera_indicator = true;
    float secondary_fov_degrees = 70, secondary_fov_range_m = 1.5F, secondary_marker_m = .12F;
    float secondary_marker_pixels = 18, secondary_overlay_height_m = 0;
    std::int32_t overlay_x = 16, overlay_y = 16, overlay_maximum_width = 0;
};
enum class PixelFormat : std::uint32_t
{
    Bgrx8 = 1
};
struct RenderFrame
{
    int dma_fd = -1;
    std::uint32_t width = 0, height = 0, stride_bytes = 0;
    PixelFormat pixel_format = PixelFormat::Bgrx8;
    std::uint64_t sequence = 0;
};
struct Nv12Overlay
{
    int dma_fd = -1;
    std::uint32_t width = 0, height = 0, stride_bytes = 0;
};
struct RenderStats
{
    std::uint64_t frames = 0, updates = 0, removals = 0, evictions = 0, uploaded_bytes = 0;
    std::uint64_t compute_dispatches = 0;
    std::uint64_t resident_chunks = 0, visible_chunks = 0, resident_vertices = 0;
    std::uint64_t gpu_capacity_bytes = 0;
    bool indexed_multi_draw = false;
    double prepare_ms = 0, draw_ms = 0, fence_ms = 0, page_flip_ms = 0;
    double apply_changes_ms = 0, prepare_geometry_ms = 0, submit_geometry_ms = 0;
    std::uint64_t input_chunks = 0, output_chunks = 0;
};
class SPACEMIT_MAPPING_API Renderer
{
public:
    explicit Renderer(const RendererOptions& options = {});
    ~Renderer();
    Renderer(Renderer&&) noexcept;
    Renderer& operator=(Renderer&&) noexcept;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    void open();
    void close();
    // Column-major clip_from_map. layer 0=surface, 1=distance.
    void projection(const std::array<float, 16>& matrix, std::uint32_t layer = 0);
    void camera(const Pose& pose);
    void trajectory(const std::vector<Vec3>& points);
    void overlay(const Nv12Overlay& frame);
    void surface(const std::vector<Snapshot>& updates, const SurfaceOptions& options = {});
    void geometry(const std::vector<Snapshot>& updates, std::uint32_t layer = 1);
    // Callback runs synchronously in the current GL context; it must not reenter
    // this renderer. Exceptions are transported back after the private call returns.
    RenderFrame render(const std::function<void(int, int)>& draw_overlay = {});
    // BGRX8 DMA fd is borrowed until next render/close; duplicate/import before then.
    RenderFrame frame() const;
    RenderStats statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace spacemit_mapping::sdk
#endif // SPACEMIT_MAPPING_H
