#ifndef FEATURE_OVERLAY_PANEL_H
#define FEATURE_OVERLAY_PANEL_H
#include "esdf_drm/gles_render_pipeline.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>

// K3's gl2ext.h needs the GLES core declarations above.
#include <GLES2/gl2ext.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace esdf_drm
{

// Render-thread-only diagnostic panel. The source NV12 DMA-BUF is imported as
// a temporary EGLImage and sampled into one retained grayscale GPU texture.
// No image pixels pass through CPU memory or retain a camera buffer while idle.
class FeatureOverlayPanel
{
public:
    bool initialize(std::string& error, std::uint32_t age_max_frames = 14);
    void shutdown();
    bool update(const FeatureOverlayFrame& frame, std::string& error);
    void draw(int output_width, int output_height, bool reserve_preview_space);

private:
    struct TextVertex
    {
        float x, y, r, g, b;
    };

    void append_rectangle(float x, float y, float width, float height, float r, float g, float b);
    void append_text(float x, float y, const char* value, float scale = 3.0F,
        std::array<float, 3> rgb = {0.15F, 1.0F, 0.3F});
    void draw_label(int width, int height);

    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    PFNEGLCREATEIMAGEKHRPROC create_image_ = nullptr;
    PFNEGLDESTROYIMAGEKHRPROC destroy_image_ = nullptr;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC image_texture_ = nullptr;
    GLuint image_program_ = 0;
    GLuint copy_program_ = 0;
    GLuint copy_framebuffer_ = 0;
    GLuint point_program_ = 0;
    GLuint text_program_ = 0;
    GLuint image_vertex_array_ = 0;
    GLuint point_vertex_array_ = 0;
    GLuint point_buffer_ = 0;
    GLuint text_vertex_array_ = 0;
    GLuint text_buffer_ = 0;
    GLuint image_texture_id_ = 0;
    GLint image_sampler_location_ = -1;
    GLint copy_sampler_location_ = -1;
    GLint point_image_size_location_ = -1;
    GLint point_age_max_location_ = -1;
    GLint point_age_colors_location_ = -1;
    std::uint32_t age_max_frames_ = 14;
    GLint text_viewport_size_location_ = -1;
    int image_width_ = 0;
    int image_height_ = 0;
    std::size_t point_count_ = 0;
    std::size_t source_point_count_ = 0;
    std::size_t pose_input_count_ = 0;
    std::size_t label_pose_input_count_ = 0;
    bool pose_inputs_available_ = false, label_pose_inputs_available_ = false;
    std::vector<TextVertex> text_vertices_;
    std::size_t label_count_ = 0;
    int label_width_ = 0;
    int label_height_ = 0;
    bool label_ready_ = false;
};

} // namespace esdf_drm

#endif // FEATURE_OVERLAY_PANEL_H
