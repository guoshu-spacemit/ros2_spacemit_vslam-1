/**
 * @file feature_overlay_panel.cpp
 * @brief Compose synchronized VIO image observations and feature diagnostics.
 */
#include "feature_overlay_panel.h"

#include <drm/drm_fourcc.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace esdf_drm
{
namespace
{

constexpr int kMarginPixels = 16;
constexpr std::size_t kMaxDrawnPoints = 2048;
constexpr std::size_t kMaxTextVertices = 32768;
// GLES point colors from young to old observations.
constexpr std::array<float, 21> kAgeColors = {0.65F, 0.0F, 1.0F, 0.0F, 0.2F, 1.0F, 0.0F, 1.0F, 1.0F,
    0.0F, 1.0F, 0.0F, 1.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F};

constexpr const char* kImageVertexShader = R"(#version 310 es
out highp vec2 texture_coordinate;
void main() {
    const vec2 positions[3] = vec2[3](
        vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    vec2 position = positions[gl_VertexID];
    texture_coordinate = position * 0.5 + 0.5;
    gl_Position = vec4(position, 0.0, 1.0);
})";

constexpr const char* kCopyFragmentShader = R"(#version 310 es
#extension GL_OES_EGL_image_external_essl3 : require
precision highp float;
uniform samplerExternalOES input_nv12;
in highp vec2 texture_coordinate;
layout(location = 0) out vec4 fragment_color;
void main() {
    vec3 rgb = texture(input_nv12, texture_coordinate).rgb;
    float gray = dot(rgb, vec3(0.299, 0.587, 0.114));
    fragment_color = vec4(vec3(gray), 1.0);
})";

constexpr const char* kImageFragmentShader = R"(#version 310 es
precision highp float;
uniform sampler2D input_gray;
in highp vec2 texture_coordinate;
layout(location = 0) out vec4 fragment_color;
void main() {
    float gray = texture(input_gray, texture_coordinate).r;
    fragment_color = vec4(vec3(gray), 1.0);
})";

constexpr const char* kPointVertexShader = R"(#version 310 es
layout(location = 0) in vec2 feature_xy;
layout(location = 1) in float pose_input;
layout(location = 2) in float track_observation_count;
uniform vec2 image_size;
uniform float age_max_frames;
uniform vec3 age_colors[7];
flat out highp float marker_size;
flat out highp float selected_pose_input;
flat out highp vec3 age_color;
void main() {
    gl_Position = vec4(feature_xy / image_size * 2.0 - 1.0, 0.0, 1.0);
    selected_pose_input = pose_input;
    // First observation is frame 1 (purple); the configured limit is red.
    float age = clamp((track_observation_count - 1.0) / (age_max_frames - 1.0), 0.0, 1.0) * 6.0;
    int segment = min(int(age), 5);
    age_color = mix(age_colors[segment], age_colors[segment + 1], age - float(segment));
    marker_size = pose_input > 0.5 ? 8.0 : 6.0;
    gl_PointSize = marker_size;
})";

constexpr const char* kPointFragmentShader = R"(#version 310 es
precision mediump float;
flat in highp float marker_size;
flat in highp float selected_pose_input;
flat in highp vec3 age_color;
layout(location = 0) out vec4 fragment_color;
void main() {
    vec2 center = (gl_PointCoord - vec2(0.5)) * marker_size;
    float radius_squared = dot(center, center);
    if (radius_squared > marker_size * marker_size * 0.25) discard;
    // Four screen pixels across (previously five), independent of image scale.
    vec3 color = radius_squared <= 4.0 ? age_color : vec3(0.0);
    // White denotes pose-input membership independently of observation age.
    if (selected_pose_input > 0.5 && radius_squared >= 9.0) color = vec3(1.0);
    fragment_color = vec4(color, 1.0);
})";

constexpr const char* kTextVertexShader = R"(#version 310 es
layout(location = 0) in vec2 position;
layout(location = 1) in vec3 rgb;
uniform vec2 viewport_size;
out lowp vec3 text_color;
void main() {
    gl_Position = vec4(position / viewport_size * 2.0 - 1.0, 0.0, 1.0);
    // The GBM scanout buffer is read top-down on this path, as in the HUD.
    gl_Position.y = -gl_Position.y;
    text_color = rgb;
})";

constexpr const char* kTextFragmentShader = R"(#version 310 es
precision mediump float;
in lowp vec3 text_color;
layout(location = 0) out vec4 fragment_color;
void main() { fragment_color = vec4(text_color, 1.0); })";

GLuint compile_shader(GLenum type, const char* source, std::string& error)
{
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE)
    {
        char log[1024]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        error = std::string("feature overlay shader compile failed: ") + log;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint create_program(const char* vertex_source, const char* fragment_source, std::string& error)
{
    const GLuint vertex = compile_shader(GL_VERTEX_SHADER, vertex_source, error);
    if (!vertex)
        return 0;
    const GLuint fragment = compile_shader(GL_FRAGMENT_SHADER, fragment_source, error);
    if (!fragment)
    {
        glDeleteShader(vertex);
        return 0;
    }
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE)
    {
        char log[1024]{};
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        error = std::string("feature overlay shader link failed: ") + log;
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

// Snapshot upload runs outside renderFrame and must restore its caller's state.
struct CopyGlState
{
    GLint program{}, vertex_array{}, array_buffer{}, unpack_buffer{}, framebuffer{};
    GLint viewport[4]{}, active_texture{}, external_texture{}, gray_texture{};
    GLboolean color_mask[4]{};
    const std::array<GLenum, 6> capabilities{GL_DEPTH_TEST, GL_BLEND, GL_SCISSOR_TEST, GL_CULL_FACE,
        GL_STENCIL_TEST, GL_RASTERIZER_DISCARD};
    std::array<GLboolean, 6> enabled{};

    CopyGlState()
    {
        glGetIntegerv(GL_CURRENT_PROGRAM, &program);
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vertex_array);
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &array_buffer);
        glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpack_buffer);
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &framebuffer);
        glGetIntegerv(GL_VIEWPORT, viewport);
        glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
        glActiveTexture(GL_TEXTURE4);
        glGetIntegerv(GL_TEXTURE_BINDING_EXTERNAL_OES, &external_texture);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &gray_texture);
        glGetBooleanv(GL_COLOR_WRITEMASK, color_mask);
        for (std::size_t index = 0; index < capabilities.size(); ++index)
            enabled[index] = glIsEnabled(capabilities[index]);
    }

    ~CopyGlState()
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(framebuffer));
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glUseProgram(static_cast<GLuint>(program));
        glBindVertexArray(static_cast<GLuint>(vertex_array));
        glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(array_buffer));
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpack_buffer));
        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, static_cast<GLuint>(external_texture));
        glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(gray_texture));
        glActiveTexture(static_cast<GLenum>(active_texture));
        glColorMask(color_mask[0], color_mask[1], color_mask[2], color_mask[3]);
        for (std::size_t index = 0; index < capabilities.size(); ++index)
            if (enabled[index])
                glEnable(capabilities[index]);
            else
                glDisable(capabilities[index]);
    }
};

std::array<unsigned char, 7> glyph(char value)
{
    switch (value)
    {
    case '0':
        return {14, 17, 19, 21, 25, 17, 14};
    case '1':
        return {4, 12, 20, 4, 4, 4, 31};
    case '2':
        return {14, 17, 1, 2, 4, 8, 31};
    case '3':
        return {30, 1, 1, 14, 1, 1, 30};
    case '4':
        return {2, 6, 10, 18, 31, 2, 2};
    case '5':
        return {31, 16, 16, 30, 1, 1, 30};
    case '6':
        return {14, 16, 16, 30, 17, 17, 14};
    case '7':
        return {31, 1, 2, 4, 8, 8, 8};
    case '8':
        return {14, 17, 17, 14, 17, 17, 14};
    case '9':
        return {14, 17, 17, 15, 1, 1, 14};
    case 'D':
        return {30, 17, 17, 17, 17, 17, 30};
    case 'F':
        return {31, 16, 16, 30, 16, 16, 16};
    case 'R':
        return {30, 17, 17, 30, 20, 18, 17};
    case 'A':
        return {14, 17, 17, 31, 17, 17, 17};
    case 'M':
        return {17, 27, 21, 21, 17, 17, 17};
    case 'E':
        return {31, 16, 16, 30, 16, 16, 31};
    case 'G':
        return {14, 17, 16, 23, 17, 17, 14};
    case 'S':
        return {15, 16, 16, 14, 1, 1, 30};
    case '.':
        return {0, 0, 0, 0, 0, 0, 4};
    case '+':
        return {0, 4, 4, 31, 4, 4, 0};
    case 'B':
        return {30, 17, 17, 30, 17, 17, 30};
    case 'C':
        return {14, 17, 16, 16, 16, 17, 14};
    case 'I':
        return {14, 4, 4, 4, 4, 4, 14};
    case 'H':
        return {17, 17, 17, 31, 17, 17, 17};
    case 'K':
        return {17, 18, 20, 24, 20, 18, 17};
    case 'L':
        return {16, 16, 16, 16, 16, 16, 31};
    case 'N':
        return {17, 25, 21, 19, 17, 17, 17};
    case 'O':
        return {14, 17, 17, 17, 17, 17, 14};
    case 'P':
        return {30, 17, 17, 30, 16, 16, 16};
    case 'T':
        return {31, 4, 4, 4, 4, 4, 4};
    case 'U':
        return {17, 17, 17, 17, 17, 17, 14};
    case 'V':
        return {17, 17, 17, 17, 17, 10, 4};
    case 'W':
        return {17, 17, 17, 21, 21, 21, 10};
    case 'X':
        return {17, 17, 10, 4, 10, 17, 17};
    case 'Y':
        return {17, 17, 10, 4, 4, 4, 4};
    case 'Z':
        return {31, 1, 2, 4, 8, 16, 31};
    case '/':
        return {1, 2, 2, 4, 8, 8, 16};
    case '-':
        return {0, 0, 0, 31, 0, 0, 0};
    case ':':
        return {0, 4, 4, 0, 4, 4, 0};
    case '>':
        return {16, 8, 4, 2, 4, 8, 16};
    case '<':
        return {1, 2, 4, 8, 4, 2, 1};
    default:
        return {};
    }
}

} // namespace

bool FeatureOverlayPanel::initialize(std::string& error, std::uint32_t age_max_frames)
{
    if (age_max_frames < 2 || age_max_frames > 1000000)
    {
        error = "feature age color limit must be in [2, 1000000] processed frames";
        return false;
    }
    age_max_frames_ = age_max_frames;
    label_ready_ = false;
    egl_display_ = eglGetCurrentDisplay();
    if (egl_display_ == EGL_NO_DISPLAY || eglGetCurrentContext() == EGL_NO_CONTEXT)
    {
        error = "feature overlay requires a current EGL context";
        return false;
    }
    create_image_ =
        reinterpret_cast<PFNEGLCREATEIMAGEKHRPROC>(eglGetProcAddress("eglCreateImageKHR"));
    destroy_image_ =
        reinterpret_cast<PFNEGLDESTROYIMAGEKHRPROC>(eglGetProcAddress("eglDestroyImageKHR"));
    image_texture_ = reinterpret_cast<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>(
        eglGetProcAddress("glEGLImageTargetTexture2DOES"));
    if (!create_image_ || !destroy_image_ || !image_texture_)
    {
        error = "feature overlay EGL DMA-BUF import extensions are unavailable";
        return false;
    }

    image_program_ = create_program(kImageVertexShader, kImageFragmentShader, error);
    copy_program_ =
        image_program_ ? create_program(kImageVertexShader, kCopyFragmentShader, error) : 0;
    point_program_ =
        copy_program_ ? create_program(kPointVertexShader, kPointFragmentShader, error) : 0;
    text_program_ =
        point_program_ ? create_program(kTextVertexShader, kTextFragmentShader, error) : 0;
    if (!text_program_)
    {
        shutdown();
        return false;
    }
    image_sampler_location_ = glGetUniformLocation(image_program_, "input_gray");
    copy_sampler_location_ = glGetUniformLocation(copy_program_, "input_nv12");
    point_image_size_location_ = glGetUniformLocation(point_program_, "image_size");
    point_age_max_location_ = glGetUniformLocation(point_program_, "age_max_frames");
    point_age_colors_location_ = glGetUniformLocation(point_program_, "age_colors[0]");
    text_viewport_size_location_ = glGetUniformLocation(text_program_, "viewport_size");

    GLint previous_vertex_array = 0, previous_buffer = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previous_vertex_array);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &previous_buffer);
    glGenVertexArrays(1, &image_vertex_array_);
    glGenFramebuffers(1, &copy_framebuffer_);
    glGenVertexArrays(1, &point_vertex_array_);
    glBindVertexArray(point_vertex_array_);
    glGenBuffers(1, &point_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, point_buffer_);
    glBufferData(
        GL_ARRAY_BUFFER, kMaxDrawnPoints * sizeof(FeatureOverlayPoint), nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint),
        reinterpret_cast<const void*>(offsetof(FeatureOverlayPoint, pose_input)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint),
        reinterpret_cast<const void*>(offsetof(FeatureOverlayPoint, track_observation_count)));

    glGenVertexArrays(1, &text_vertex_array_);
    glBindVertexArray(text_vertex_array_);
    glGenBuffers(1, &text_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, text_buffer_);
    glBufferData(GL_ARRAY_BUFFER, kMaxTextVertices * sizeof(TextVertex), nullptr, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(TextVertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(TextVertex),
        reinterpret_cast<const void*>(2 * sizeof(float)));
    glBindVertexArray(static_cast<GLuint>(previous_vertex_array));
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(previous_buffer));
    text_vertices_.reserve(kMaxTextVertices);
    const GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR)
    {
        error = "feature overlay GLES initialization failed: " + std::to_string(gl_error);
        shutdown();
        return false;
    }
    return true;
}

void FeatureOverlayPanel::shutdown()
{
    if (image_texture_id_)
        glDeleteTextures(1, &image_texture_id_);
    if (copy_framebuffer_)
        glDeleteFramebuffers(1, &copy_framebuffer_);
    if (text_buffer_)
        glDeleteBuffers(1, &text_buffer_);
    if (text_vertex_array_)
        glDeleteVertexArrays(1, &text_vertex_array_);
    if (point_buffer_)
        glDeleteBuffers(1, &point_buffer_);
    if (point_vertex_array_)
        glDeleteVertexArrays(1, &point_vertex_array_);
    if (image_vertex_array_)
        glDeleteVertexArrays(1, &image_vertex_array_);
    if (text_program_)
        glDeleteProgram(text_program_);
    if (point_program_)
        glDeleteProgram(point_program_);
    if (image_program_)
        glDeleteProgram(image_program_);
    if (copy_program_)
        glDeleteProgram(copy_program_);
    image_texture_id_ = text_buffer_ = text_vertex_array_ = point_buffer_ = point_vertex_array_ =
        image_vertex_array_ = 0;
    text_program_ = point_program_ = image_program_ = copy_program_ = copy_framebuffer_ = 0;
    image_width_ = image_height_ = 0;
    point_count_ = source_point_count_ = 0;
    label_ready_ = false;
}

bool FeatureOverlayPanel::update(const FeatureOverlayFrame& frame, std::string& error)
{
    if (!image_program_ || frame.dma_fd < 0 || frame.width <= 0 || frame.height <= 0 ||
        frame.stride_bytes < frame.width || (frame.width & 1) || (frame.height & 1))
    {
        error = "invalid feature overlay NV12 frame";
        return false;
    }
    const EGLint attributes[] = {EGL_WIDTH, frame.width, EGL_HEIGHT, frame.height,
        EGL_LINUX_DRM_FOURCC_EXT, static_cast<EGLint>(DRM_FORMAT_NV12), EGL_DMA_BUF_PLANE0_FD_EXT,
        frame.dma_fd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT,
        frame.stride_bytes, EGL_DMA_BUF_PLANE1_FD_EXT, frame.dma_fd, EGL_DMA_BUF_PLANE1_OFFSET_EXT,
        frame.stride_bytes * frame.height, EGL_DMA_BUF_PLANE1_PITCH_EXT, frame.stride_bytes,
        EGL_YUV_COLOR_SPACE_HINT_EXT, EGL_ITU_REC601_EXT, EGL_SAMPLE_RANGE_HINT_EXT,
        EGL_YUV_NARROW_RANGE_EXT, EGL_NONE};
    const EGLImageKHR new_image =
        create_image_(egl_display_, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attributes);
    if (new_image == EGL_NO_IMAGE_KHR)
    {
        error = "feature overlay NV12 EGLImage import failed";
        return false;
    }

    const CopyGlState previous_state;
    GLuint input_texture = 0;
    GLuint new_texture = 0;
    glGenTextures(1, &input_texture);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, input_texture);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    image_texture_(GL_TEXTURE_EXTERNAL_OES, new_image);
    const GLenum import_error = glGetError();
    if (import_error != GL_NO_ERROR)
    {
        glDeleteTextures(1, &input_texture);
        destroy_image_(egl_display_, new_image);
        error = "feature overlay NV12 GLES import failed: " + std::to_string(import_error);
        return false;
    }

    // Preserve the exact paired image without holding a shared camera pool slot.
    glGenTextures(1, &new_texture);
    glBindTexture(GL_TEXTURE_2D, new_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_R8, frame.width, frame.height, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, copy_framebuffer_);
    glFramebufferTexture2D(
        GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, new_texture, 0);
    const bool framebuffer_ready =
        glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    if (framebuffer_ready)
    {
        for (const auto capability : previous_state.capabilities)
            glDisable(capability);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glViewport(0, 0, frame.width, frame.height);
        glUseProgram(copy_program_);
        glUniform1i(copy_sampler_location_, 4);
        glBindVertexArray(image_vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        // Only the render thread waits; capture/VIO remain independently scheduled.
        // The source VB lease can be released as soon as this GPU read is done.
        glFinish();
    }
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glDeleteTextures(1, &input_texture);
    destroy_image_(egl_display_, new_image);
    const GLenum copy_error = glGetError();
    if (!framebuffer_ready || copy_error != GL_NO_ERROR)
    {
        glDeleteTextures(1, &new_texture);
        error = "feature overlay grayscale GPU copy failed: " + std::to_string(copy_error);
        return false;
    }

    std::vector<FeatureOverlayPoint> visible;
    visible.reserve(std::min(frame.points.size(), kMaxDrawnPoints));
    for (const auto& point : frame.points)
    {
        if (visible.size() == kMaxDrawnPoints)
            break;
        if (std::isfinite(point.x) && std::isfinite(point.y) && point.x >= 0.0F &&
            point.y >= 0.0F && point.x < frame.width && point.y < frame.height)
        {
            auto safe_point = point;
            if (!std::isfinite(safe_point.track_observation_count) ||
                safe_point.track_observation_count < 1.0F)
                safe_point.track_observation_count = 1.0F;
            visible.push_back(safe_point);
        }
    }
    // Commit the image and its points together; a failed upload keeps the old pair.
    GLuint new_point_buffer = 0;
    glGenBuffers(1, &new_point_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, new_point_buffer);
    glBufferData(
        GL_ARRAY_BUFFER, kMaxDrawnPoints * sizeof(FeatureOverlayPoint), nullptr, GL_STREAM_DRAW);
    if (!visible.empty())
    {
        glBufferSubData(
            GL_ARRAY_BUFFER, 0, visible.size() * sizeof(FeatureOverlayPoint), visible.data());
    }
    const GLenum upload_error = glGetError();
    if (upload_error != GL_NO_ERROR)
    {
        glDeleteBuffers(1, &new_point_buffer);
        glDeleteTextures(1, &new_texture);
        error = "feature overlay point upload failed: " + std::to_string(upload_error);
        return false;
    }

    glBindVertexArray(point_vertex_array_);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint),
        reinterpret_cast<const void*>(offsetof(FeatureOverlayPoint, pose_input)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(FeatureOverlayPoint),
        reinterpret_cast<const void*>(offsetof(FeatureOverlayPoint, track_observation_count)));
    glDeleteBuffers(1, &point_buffer_);
    point_buffer_ = new_point_buffer;
    if (image_texture_id_)
        glDeleteTextures(1, &image_texture_id_);
    image_texture_id_ = new_texture;
    image_width_ = frame.width;
    image_height_ = frame.height;
    point_count_ = visible.size();
    source_point_count_ = frame.tracked_feature_count;
    pose_input_count_ = frame.pose_input_count;
    pose_inputs_available_ = frame.pose_inputs_available;
    if (source_point_count_ != label_count_ || pose_input_count_ != label_pose_input_count_ ||
        pose_inputs_available_ != label_pose_inputs_available_)
        label_ready_ = false;
    return true;
}

void FeatureOverlayPanel::append_rectangle(
    float x, float y, float width, float height, float r, float g, float b)
{
    if (text_vertices_.size() + 6 > kMaxTextVertices)
        return;
    const std::array<TextVertex, 6> vertices{
        {{x, y, r, g, b}, {x + width, y, r, g, b}, {x, y + height, r, g, b},
            {x, y + height, r, g, b}, {x + width, y, r, g, b}, {x + width, y + height, r, g, b}}};
    text_vertices_.insert(text_vertices_.end(), vertices.begin(), vertices.end());
}

void FeatureOverlayPanel::append_text(
    float x, float y, const char* value, float scale, std::array<float, 3> rgb)
{
    for (; *value; ++value, x += 6.0F * scale)
    {
        const auto rows = glyph(*value);
        for (int row = 0; row < 7; ++row)
            for (int col = 0; col < 5; ++col)
                if (rows[row] & (1 << (4 - col)))
                    append_rectangle(x + col * scale, y + (6 - row) * scale, scale, scale, rgb[0],
                        rgb[1], rgb[2]);
    }
}

void FeatureOverlayPanel::draw_label(int width, int height)
{
    if (!label_ready_ || label_width_ != width || label_height_ != height)
    {
        text_vertices_.clear();
        char label[64];
        if (pose_inputs_available_)
            std::snprintf(label, sizeof(label), "FEATURES %zu INPUT %zu", source_point_count_,
                pose_input_count_);
        else
            std::snprintf(label, sizeof(label), "FEATURES %zu INPUT --", source_point_count_);
        const int label_width = static_cast<int>(std::strlen(label)) * 18 + 16;
        append_rectangle(0.0F, 0.0F, std::min(width, label_width), 32.0F, 0.01F, 0.01F, 0.01F);
        append_text(8.0F, 6.0F, label);
        glBindBuffer(GL_ARRAY_BUFFER, text_buffer_);
        glBufferSubData(
            GL_ARRAY_BUFFER, 0, text_vertices_.size() * sizeof(TextVertex), text_vertices_.data());
        label_count_ = source_point_count_;
        label_pose_input_count_ = pose_input_count_;
        label_pose_inputs_available_ = pose_inputs_available_;
        label_width_ = width;
        label_height_ = height;
        label_ready_ = true;
    }
    glUseProgram(text_program_);
    glUniform2f(
        text_viewport_size_location_, static_cast<float>(width), static_cast<float>(height));
    glBindVertexArray(text_vertex_array_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(text_vertices_.size()));
}

void FeatureOverlayPanel::draw(int output_width, int output_height, bool reserve_preview_space)
{
    if (!image_texture_id_ || output_width <= 0 || output_height <= 0)
        return;
    // Reserve the configured preview column before its first frame arrives.
    // Arrival order, warmup, temporary loss and preview resolution must not
    // move or resize the diagnostics tile. No model dimensions are needed.
    const int panel_x = kMarginPixels + (reserve_preview_space ? output_width / 2 : 0);
    // Show features at native input resolution when space permits. Do not
    // shrink the tile to half the neighboring preview; that hid feature details.
    int panel_width = std::min(image_width_, output_width - panel_x - kMarginPixels);
    if (panel_width <= 0)
        return;
    int panel_height =
        static_cast<int>(static_cast<double>(panel_width) * image_height_ / image_width_);
    const int maximum_height = std::max(1, output_height / 2 - kMarginPixels);
    if (panel_height > maximum_height)
    {
        panel_height = maximum_height;
        panel_width =
            static_cast<int>(static_cast<double>(panel_height) * image_width_ / image_height_);
    }
    if (panel_width < 64 || panel_height < 48)
        return;

    GLint previous_program = 0, previous_vertex_array = 0, previous_buffer = 0;
    GLint previous_viewport[4]{}, previous_active_texture = 0;
    GLint previous_gray_texture = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &previous_program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previous_vertex_array);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &previous_buffer);
    glGetIntegerv(GL_VIEWPORT, previous_viewport);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &previous_active_texture);
    const GLboolean depth = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glViewport(panel_x, kMarginPixels, panel_width, panel_height);

    glActiveTexture(GL_TEXTURE4);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &previous_gray_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture_id_);
    glUseProgram(image_program_);
    glUniform1i(image_sampler_location_, 4);
    glBindVertexArray(image_vertex_array_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(previous_gray_texture));
    glActiveTexture(static_cast<GLenum>(previous_active_texture));

    if (point_count_ > 0)
    {
        glUseProgram(point_program_);
        glUniform2f(point_image_size_location_, static_cast<float>(image_width_),
            static_cast<float>(image_height_));
        glUniform1f(point_age_max_location_, static_cast<float>(age_max_frames_));
        glUniform3fv(point_age_colors_location_, 7, kAgeColors.data());
        glBindVertexArray(point_vertex_array_);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(point_count_));
    }
    draw_label(panel_width, panel_height);

    glBindVertexArray(static_cast<GLuint>(previous_vertex_array));
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(previous_buffer));
    glUseProgram(static_cast<GLuint>(previous_program));
    glViewport(
        previous_viewport[0], previous_viewport[1], previous_viewport[2], previous_viewport[3]);
    if (depth)
        glEnable(GL_DEPTH_TEST);
    if (blend)
        glEnable(GL_BLEND);
}

} // namespace esdf_drm
