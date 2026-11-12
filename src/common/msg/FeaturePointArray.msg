# Visual observations and optional functional quality for this source camera/frame.
std_msgs/Header header
uint32 source_frame_index
uint32 image_width
uint32 image_height
# Whether this source frame has a valid pose, independent of optional diagnostics.
bool pose_valid
bool diagnostics_available
# Number of finite observations exported in points for this camera.
uint32 tracked_feature_count
# Candidate observations for pose estimation. Availability is independent of count.
bool pose_inputs_available
uint32 pose_input_count
# Optional prediction status. No prediction history is inferred when unavailable.
bool prediction_available
bool prediction_active
uint32 prediction_streak
float32 prediction_span_ms
# Optional same-frame image reprojection fit; it is not world-frame pose accuracy.
bool reprojection_available
float32 reprojection_rmse_px
common/FeaturePoint[] points
