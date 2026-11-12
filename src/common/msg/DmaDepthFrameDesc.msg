# Descriptor for an MPP VB buffer containing a single float32 depth plane.
# The token is imported with VB_Import() and released with VB_ReleaseBuffer().
# Pixels are not serialized through DDS.

std_msgs/Header header

uint32 version
uint64 token
uint64 pool_id
uint64 buffer_id
uint32 frame_index

uint32 width
uint32 height
uint32 stride_bytes
uint32 encoding
uint64 pts
uint32 total_size

# RGB8 image rectified and aligned pixel-for-pixel with this depth image.
# It is carried by a second MPP VB token, so DDS only transports metadata.
uint64 rgb_token
uint64 rgb_pool_id
uint64 rgb_buffer_id
uint32 rgb_width
uint32 rgb_height
uint32 rgb_stride_bytes
uint32 rgb_encoding
uint32 rgb_total_size

# Intrinsics of this depth image and the rigid transform from the depth
# camera into the VIO left-camera frame. They are repeated in each descriptor
# so a downstream node can start independently without a parameter server.
float64 fx
float64 fy
float64 cx
float64 cy
float64[16] left_from_depth
