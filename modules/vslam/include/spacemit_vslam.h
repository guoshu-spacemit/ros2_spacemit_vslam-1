// Public visual localization and map persistence interface.
#ifndef SPACEMIT_VSLAM_H
#define SPACEMIT_VSLAM_H

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace spacemit_vslam::localization
{
// Functional C++17 interface for Linux applications using a compatible compiler
// and C++ standard-library ABI. Backend objects and storage layouts are private.
#if defined(_WIN32)
#if defined(SPACEMIT_VSLAM_EXPORT)
#define SPACEMIT_VSLAM_PUBLIC __declspec(dllexport)
#else
#define SPACEMIT_VSLAM_PUBLIC __declspec(dllimport)
#endif
#else
#define SPACEMIT_VSLAM_PUBLIC __attribute__((visibility("default")))
#endif
// Metres and unit quaternion (x,y,z,w). Camera axes: x right, y down, z forward.
// A transform named destination_from_source maps source coordinates to destination.
struct Pose
{
    std::array<float, 3> translation_m{};
    std::array<float, 4> orientation_xyzw{0, 0, 0, 1};
};
struct ImageExtent
{
    std::int32_t width_px = 0, height_px = 0;
};
struct CameraIntrinsics
{
    ImageExtent image;
    std::array<float, 2> focal_length_px{}, principal_point_px{};
};
struct LensDistortion
{
    enum class Model
    {
        Pinhole,
        Equidistant,
        Radtan,
        Rational
    };
    Model model = Model::Pinhole;
    // Pinhole: none; Equidistant: k1,k2,k3,k4; Radtan: k1,k2,p1,p2,k3;
    // Rational: k1,k2,p1,p2,k3,k4,k5,k6. Coefficients are dimensionless.
    std::vector<float> coefficients;
};
struct CameraCalibration
{
    // Resize only the image extent, focal length and principal point.
    CameraIntrinsics intrinsics;
    LensDistortion distortion;
    Pose body_from_camera;
};
struct SensorNoise
{
    float white_noise_density = 0;
    float bias_random_walk = 0;
};
struct InertialCalibration
{
    float sample_rate_hz = 0;
    // Angular: rad/s/sqrt(Hz), rad/s^2/sqrt(Hz).
    SensorNoise angular_noise;
    // Linear: m/s^2/sqrt(Hz), m/s^3/sqrt(Hz).
    SensorNoise linear_noise;
    Pose body_from_sensor;
};
struct RigCalibration
{
    std::vector<CameraCalibration> cameras;
    std::vector<InertialCalibration> inertial_sensors;
};
// Borrowed read-only host pixels until process/capture_observation returns. Padded rows
// are supported; pitch=0 selects packed rows. DMA-BUF ownership stays outside.
struct InputImage
{
    enum class PixelFormat : std::uint8_t
    {
        Mono8 = 0,
        Rgb8 = 1
    };
    PixelFormat format = PixelFormat::Mono8;
    const void* pixels = nullptr;
    std::int32_t width = 0, height = 0, pitch = 0;
    std::int64_t timestamp_ns = 0;
    std::uint32_t camera_index = 0;
};
struct InertialSample
{
    std::int64_t timestamp_ns = 0;
    // Calibrated inertial-sensor axes, before estimated bias correction.
    std::array<float, 3> acceleration_m_s2{}, angular_velocity_rad_s{};
};
struct PoseCovariance
{
    // Row-major 6x6 covariance ordered x,y,z,roll,pitch,yaw. Translation is in
    // metres and fixed XYZ roll/pitch/yaw angles in radians. These are the
    // backend's existing pose-parameter coordinates, not a Lie-tangent covariance.
    std::array<float, 36> matrix{};
};
struct PoseEstimate
{
    std::int64_t timestamp_ns = 0;
    std::optional<Pose> tracking_from_body;
    // Unavailable uncertainty is absent; it is never represented by a zero matrix.
    std::optional<PoseCovariance> uncertainty;
};
struct FeatureObservation
{
    std::uint64_t track_id = 0;
    std::uint32_t sensor_id = 0;
    std::array<float, 2> pixel{};
};
struct InertialBias
{
    // Calibrated inertial-sensor axes.
    std::array<float, 3> angular_rad_s{}, linear_m_s2{};
};
struct InertialState
{
    // Velocity of the inertial sensor origin, expressed in tracking axes.
    std::array<float, 3> velocity_tracking_m_s{};
    InertialBias estimated_bias;
};
struct Metric
{
    std::string name;
    double value = 0;
};
// Optional functional metrics: tracking.feature_count, tracking.pose_input_count,
// tracking.reprojection_rmse_px, tracking.predicted_pose, tracking.processing_ms.
// Missing names are unavailable; ignore unknown names. pose_inputs are candidate
// observations, not guaranteed accepted inliers. Solver internals stay private.
struct FrameQuality
{
    bool available = false;
    std::vector<Metric> metrics;
    std::vector<FeatureObservation> pose_inputs;
};
struct TrackingResult
{
    PoseEstimate image_pose;
    std::optional<Pose> map_pose;
    bool imu_fusion_active = false;
    std::optional<InertialState> imu;
    std::optional<std::array<float, 3>> gravity;
    FrameQuality diagnostics;
    std::string map_error;
    double map_submit_ms = 0;
};
struct TimedPose
{
    std::int64_t timestamp_ns = 0;
    Pose pose;
};
struct MapStatistics
{
    std::int64_t timestamp_ns = 0;
    std::vector<Metric> metrics;
};
using TaskId = std::uint64_t;
enum class OperationStatus : std::uint8_t
{
    Succeeded,
    NotFound,
    Unsupported,
    StaleObservation,
    Failed
};
struct MapSource
{
    // Backend-owned map directory. Maps are portable only between compatible backends.
    std::string location;
};
struct PosePrior
{
    Pose map_from_body;
};
struct SearchRegion
{
    // Axis-aligned box in map coordinates, centered on prior.map_from_body.
    // The returned body position must lie inside all three positive half extents.
    std::array<float, 3> half_extent_m{0.5F, 0.5F, 0.5F};
};
// Immutable owned pixels plus the producing session's tracking reference.
// Copies share storage; snapshots can outlive the source image and be passed
// between threads. Submission still follows Session's caller-sequence rule.
class SPACEMIT_VSLAM_PUBLIC ObservationSnapshot
{
public:
    ObservationSnapshot() noexcept;
    ~ObservationSnapshot();
    ObservationSnapshot(const ObservationSnapshot&) noexcept;
    ObservationSnapshot& operator=(const ObservationSnapshot&) noexcept;
    ObservationSnapshot(ObservationSnapshot&&) noexcept;
    ObservationSnapshot& operator=(ObservationSnapshot&&) noexcept;
    // Reports whether this snapshot contains an observation.
    bool empty() const noexcept;
    // Returns the observation time on the source clock in nanoseconds.
    std::int64_t timestamp_ns() const noexcept;

private:
    struct Impl;
    explicit ObservationSnapshot(std::shared_ptr<const Impl> impl) noexcept;
    std::shared_ptr<const Impl> impl_;
    friend class Session;
};
struct RelocalizationRequest
{
    MapSource map;
    ObservationSnapshot observation;
    // A backend without global search returns Unsupported when no prior is supplied.
    std::optional<PosePrior> prior;
    SearchRegion search;
};
struct LocalizationOutcome
{
    TaskId task = 0;
    OperationStatus status = OperationStatus::Failed;
    std::int64_t source_timestamp_ns = 0;
    std::optional<Pose> map_from_body;
    // Present when the observation belongs to a retained tracking reference.
    std::optional<Pose> map_from_tracking;
    std::string error;
};
struct MapExportRequest
{
    std::string destination;
};
struct SaveOutcome
{
    TaskId task = 0;
    OperationStatus status = OperationStatus::Failed;
    MapSource map;
    // Owned optimized trajectory captured before invoking completion.
    std::vector<TimedPose> trajectory;
    std::string error;
};
struct MapOptions
{
    bool enabled = false;
    bool asynchronous = true;
    bool planar_motion = false;
    std::string cache_directory;
    std::uint32_t observation_retention_ms = 5000;
    std::uint32_t pending_update_limit = 0;
    std::uint32_t trajectory_limit = 0;
};
enum class InputMode : std::uint8_t
{
    Stereo = 0,
    StereoInertial = 1,
    Monocular = 2
};

struct Options
{
    InputMode input_mode = InputMode::Stereo;
    bool export_features = false;
    bool export_pose_inputs = false;
    bool export_quality = false;
    // Allow background processing; this is not a cross-backend determinism guarantee.
    bool asynchronous_processing = true;
    bool rectified_input = false;
    bool allow_stationary_initialization = false;
    double minimum_depth_m = 0;
    int verbosity = 0;
    MapOptions mapping;
};
enum class Capability : std::uint64_t
{
    Stereo = 1ULL << 0,
    Inertial = 1ULL << 1,
    Mapping = 1ULL << 2,
    MapPersistence = 1ULL << 3,
    Relocalization = 1ULL << 4,
    Features = 1ULL << 5,
    Quality = 1ULL << 6,
    PoseInputs = 1ULL << 7,
    Monocular = 1ULL << 8
};

// Serialize inputs, mutation and queries on one caller sequence. Inputs share
// one monotonic clock; no shift is added here. Stereo image times differ <1 ms.
// Map callbacks can run inline or on an internal worker and must not reenter,
// destroy or move this Session. Callback exceptions are contained by the library.
// At most one map task is outstanding. Invalid/rejected submissions throw before
// acceptance; accepted tasks call completion exactly once, including failures.
// Destruction waits for tasks and callbacks while their closures remain alive.
class SPACEMIT_VSLAM_PUBLIC Session
{
public:
    // Preserve conversion from a rig calibration with default options.
    Session(const RigCalibration& rig, const Options& options = {}); // NOLINT(runtime/explicit)
    ~Session();
    Session(Session&&) noexcept;
    Session& operator=(Session&&) noexcept;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    // Returns a bitmask of supported Capability values.
    static std::uint64_t capabilities() noexcept;
    void reset();
    // Supplies a sample on the shared input clock in the calibrated IMU axes.
    void submit_imu(const InertialSample& sample);
    // Processes one observation. Image and optional mask pixels are borrowed
    // for this call. The result can omit a pose when tracking is unavailable.
    TrackingResult process(
        const std::vector<InputImage>& images, const std::vector<InputImage>& masks = {});
    std::vector<FeatureObservation> features(std::uint32_t camera) const;
    std::vector<FeatureObservation> pose_inputs(std::uint32_t camera) const;
    std::vector<TimedPose> trajectory(std::uint32_t maximum_poses = 0) const;
    std::vector<TimedPose> loop_closures() const;
    MapStatistics map_stats() const;
    // Copies the input pixels into a snapshot with the current tracking reference.
    ObservationSnapshot capture_observation(const std::vector<InputImage>& images) const;
    // Submits a search against the requested saved map. Returns the accepted task ID.
    TaskId relocalize(
        const RelocalizationRequest& request, std::function<void(LocalizationOutcome)> completion);
    // Submits a map export to the requested destination. Returns the accepted task ID.
    TaskId save_map(const MapExportRequest& request, std::function<void(SaveOutcome)> completion);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace spacemit_vslam::localization
#endif // SPACEMIT_VSLAM_H
