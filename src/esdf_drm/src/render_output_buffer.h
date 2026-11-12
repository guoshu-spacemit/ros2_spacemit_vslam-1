#ifndef RENDER_OUTPUT_BUFFER_H
#define RENDER_OUTPUT_BUFFER_H
#include <deque>
#include <stdexcept>

#include <sys/vb_api.h>

namespace esdf_drm
{

// Owns the VB_GetBuffer reference and, until queued, the export reference.
// Tokens identify an export generation; only the original handle is releasable.
class RenderOutputBuffer
{
public:
    explicit RenderOutputBuffer(UL handle) : handle_(handle) {}

    ~RenderOutputBuffer()
    {
        if (exported_)
        {
            (void)VB_Unexport(handle_);
        }
        (void)VB_ReleaseBuffer(handle_);
    }

    RenderOutputBuffer(const RenderOutputBuffer&) = delete;
    RenderOutputBuffer& operator=(const RenderOutputBuffer&) = delete;

    U64 export_token()
    {
        U64 token = 0;
        if (VB_Export(handle_, &token) != 0)
        {
            throw std::runtime_error("VB_Export failed for ESDF render output");
        }
        exported_ = true;
        return token;
    }

    // Push can allocate/throw: retain ownership until insertion succeeds.
    void retain_export(std::deque<UL>& handles)
    {
        handles.push_back(handle_);
        exported_ = false;
    }

private:
    UL handle_;
    bool exported_{false};
};

} // namespace esdf_drm

#endif // RENDER_OUTPUT_BUFFER_H
