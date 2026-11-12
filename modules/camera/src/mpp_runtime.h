// Internal native camera support for mpp runtime.
#ifndef MPP_RUNTIME_H
#define MPP_RUNTIME_H

namespace camera_sdk::detail
{
// Share the existing Nori runtime reference count without changing its capture path.
void acquire_mpp_runtime();
void release_mpp_runtime() noexcept;
} // namespace camera_sdk::detail

#endif // MPP_RUNTIME_H
