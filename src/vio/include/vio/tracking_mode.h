#ifndef TRACKING_MODE_H
#define TRACKING_MODE_H
namespace vio
{
// Pose validity takes precedence over the configured mode and fusion state.
// The caller obtains fusion_active from spacemit_vslam after Track(), not IMU receipt.
inline const char* tracking_mode_label(bool pose_valid, bool use_imu, bool fusion_active)
{
    if (!pose_valid)
        return "LOST";
    if (!use_imu)
        return "VO";
    return fusion_active ? "VIO" : "VO - IMU INIT";
}
} // namespace vio

#endif // TRACKING_MODE_H
