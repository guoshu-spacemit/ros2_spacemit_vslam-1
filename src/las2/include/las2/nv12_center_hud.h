#ifndef NV12_CENTER_HUD_H
#define NV12_CENTER_HUD_H
#include "las2/center_depth_overlay.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace las2
{

// Producer-side GPU overlay for one private NV12 preview pool. The caller
// retains the VB lease until rendering finishes and exports only on success.
class Nv12CenterHud
{
public:
    Nv12CenterHud(std::uint64_t pool_id, std::size_t maximum_imports, int width, int height);
    ~Nv12CenterHud();
    Nv12CenterHud(const Nv12CenterHud&) = delete;
    Nv12CenterHud& operator=(const Nv12CenterHud&) = delete;

    bool render(std::uint64_t pool_id, std::uint64_t buffer_id, int dma_fd, int y_stride,
        int uv_stride, std::uint64_t total_size, const CenterDepthSample& sample,
        std::string& error);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace las2

#endif // NV12_CENTER_HUD_H
