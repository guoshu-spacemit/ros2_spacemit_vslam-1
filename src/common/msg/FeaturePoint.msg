# A visual observation in the source camera image.
uint64 id
float32 x
float32 y
# A candidate observation supplied to pose estimation for this source frame.
# It is not guaranteed to be an accepted inlier or to belong to a specific solver.
bool pose_input
# Continuous observation history in source image time; first observation is 0 s / 1.
# Missing observations or a session reset restart this history.
float32 track_age_seconds
uint32 track_observation_count
