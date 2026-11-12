/**
 * @file performance_hud.cpp
 * @brief Compose rendering and pipeline timing diagnostics for the display HUD.
 */
#include "performance_hud.h"

#include <array>
#include <cstdio>

namespace esdf_drm
{
namespace
{
// Same 5x7 ASCII glyph style as the ROBOT RTSP overlay. All HUD labels use
// uppercase ASCII; no system fonts or missing CJK glyphs on clean boards.
std::array<unsigned char, 7> glyph(char c)
{
    switch (c)
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
    case 'A':
        return {14, 17, 17, 31, 17, 17, 17};
    case 'C':
        return {14, 17, 16, 16, 16, 17, 14};
    case 'D':
        return {30, 17, 17, 17, 17, 17, 30};
    case 'E':
        return {31, 16, 16, 30, 16, 16, 31};
    case 'F':
        return {31, 16, 16, 30, 16, 16, 16};
    case 'G':
        return {14, 17, 16, 23, 17, 17, 15};
    case 'I':
        return {31, 4, 4, 4, 4, 4, 31};
    case 'H':
        return {17, 17, 17, 31, 17, 17, 17};
    case 'K':
        return {17, 18, 20, 24, 20, 18, 17};
    case 'L':
        return {16, 16, 16, 16, 16, 16, 31};
    case 'M':
        return {17, 27, 21, 21, 17, 17, 17};
    case 'N':
        return {17, 25, 21, 19, 17, 17, 17};
    case 'O':
        return {14, 17, 17, 17, 17, 17, 14};
    case 'P':
        return {30, 17, 17, 30, 16, 16, 16};
    case 'R':
        return {30, 17, 17, 30, 20, 18, 17};
    case 'S':
        return {15, 16, 16, 14, 1, 1, 30};
    case 'T':
        return {31, 4, 4, 4, 4, 4, 4};
    case 'U':
        return {17, 17, 17, 17, 17, 17, 14};
    case 'V':
        return {17, 17, 17, 17, 17, 10, 4};
    case 'W':
        return {17, 17, 17, 21, 21, 21, 10};
    case '.':
        return {0, 0, 0, 0, 0, 12, 12};
    case '-':
        return {0, 0, 0, 31, 0, 0, 0};
    default:
        return {};
    }
}
constexpr std::size_t kMaxVertices = 24000;
} // namespace

bool PerformanceHud::initialize(std::string& error)
{
    const char* vertex_source = R"(#version 300 es
layout(location=0) in vec3 vertex;
uniform vec2 screen_size;
out float shade;
void main() {
    gl_Position = vec4(vertex.xy / screen_size * 2.0 - 1.0, 0.0, 1.0);
    // Exported GBM scanout storage has a top-left row origin on this path.
    // Convert our bottom-left HUD coordinates before rendering into it.
    gl_Position.y = -gl_Position.y;
    shade = vertex.z;
})";
    const char* fragment_source = R"(#version 300 es
precision mediump float;
in float shade;
out vec4 color;
void main() { color = vec4(vec3(shade), 1.0); }
)";
    const GLuint vs = glCreateShader(GL_VERTEX_SHADER), fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(vs, 1, &vertex_source, nullptr);
    glShaderSource(fs, 1, &fragment_source, nullptr);
    glCompileShader(vs);
    glCompileShader(fs);
    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint linked = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        char log[1024]{};
        glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
        error = log;
        shutdown();
        return false;
    }
    size_location_ = glGetUniformLocation(program_, "screen_size");
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, kMaxVertices * sizeof(Vertex), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    vertices_.reserve(kMaxVertices);
    next_update_ = {};
    vertices_.clear();
    const GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR)
    {
        error = "GLES HUD allocation failed: " + std::to_string(gl_error);
        shutdown();
        return false;
    }
    return true;
}

void PerformanceHud::shutdown()
{
    if (vbo_)
        glDeleteBuffers(1, &vbo_);
    if (vao_)
        glDeleteVertexArrays(1, &vao_);
    if (program_)
        glDeleteProgram(program_);
    vbo_ = vao_ = program_ = 0;
}

void PerformanceHud::rectangle(float x, float y, float w, float h, float shade)
{
    if (vertices_.size() + 6 > kMaxVertices)
        return;
    const std::array<Vertex, 6> quad{{{x, y, shade}, {x + w, y, shade}, {x, y + h, shade},
        {x, y + h, shade}, {x + w, y, shade}, {x + w, y + h, shade}}};
    vertices_.insert(vertices_.end(), quad.begin(), quad.end());
}

void PerformanceHud::text(float x, float y, const char* value)
{
    constexpr float scale = 3.0F;
    for (; *value; ++value, x += 6 * scale)
    {
        const auto rows = glyph(*value);
        for (int row = 0; row < 7; ++row)
            for (int col = 0; col < 5; ++col)
                if (rows[row] & (1 << (4 - col)))
                    rectangle(x + col * scale, y + (6 - row) * scale, scale, scale, 1.0F);
    }
}

void PerformanceHud::draw(int width, int height, const PerformanceStats& stats)
{
    if (!program_ || width <= 0 || height <= 0)
        return;
    GLint program, vao, buffer, viewport[4];
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &buffer);
    glGetIntegerv(GL_VIEWPORT, viewport);
    const GLboolean depth = glIsEnabled(GL_DEPTH_TEST), blend = glIsEnabled(GL_BLEND);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const auto now = EventWindow::Clock::now();
    if (now >= next_update_)
    {
        const auto vio = stats.vio.sample(now), tsdf = stats.tsdf.sample(now);
        const auto las2 = stats.las2.sample(now);
        vertices_.clear();
        rectangle(16, 16, 520, 158, 0.025F);
        char line[64];
        std::snprintf(line, sizeof(line), "MODE %s", stats.tracking_mode.sample(now));
        text(28, 142, line);
        std::snprintf(line, sizeof(line), "VSLAM FRONT %6.2f FPS", vio.fps);
        text(28, 112, line);
        std::snprintf(line, sizeof(line), "TSDF        %6.2f FPS", tsdf.fps);
        text(28, 82, line);
        if (las2.valid)
            std::snprintf(line, sizeof(line), "OPENCL GPU  %6.2f MS", las2.mean_ms);
        else
            std::snprintf(line, sizeof(line), "OPENCL GPU      -- MS");
        text(28, 52, line);
        std::snprintf(line, sizeof(line), "DEPTH       %6.2f FPS", las2.fps);
        text(28, 22, line);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_.size() * sizeof(Vertex), vertices_.data());
        // Advance on a 30 Hz phase, without accumulating missed text updates.
        constexpr auto period = std::chrono::nanoseconds(33333334);
        if (next_update_ == EventWindow::Clock::time_point{})
            next_update_ = now;
        next_update_ += period * ((now - next_update_) / period + 1);
    }
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glUseProgram(program_);
    glUniform2f(size_location_, static_cast<float>(width), static_cast<float>(height));
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices_.size()));
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glUseProgram(program);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    if (depth)
        glEnable(GL_DEPTH_TEST);
    if (blend)
        glEnable(GL_BLEND);
}
} // namespace esdf_drm
