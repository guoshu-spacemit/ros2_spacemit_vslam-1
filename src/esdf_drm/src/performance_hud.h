#ifndef PERFORMANCE_HUD_H
#define PERFORMANCE_HUD_H
#include "esdf_drm/performance_stats.h"

#include <GLES3/gl3.h>
#include <string>
#include <vector>

namespace esdf_drm
{
// Render-thread-only GLES resources. No scanout readback or CPU frame copy.
class PerformanceHud
{
public:
    bool initialize(std::string& error);
    void shutdown();
    void draw(int width, int height, const PerformanceStats& stats);

private:
    struct Vertex
    {
        float x, y, shade;
    };
    void rectangle(float x, float y, float w, float h, float shade);
    void text(float x, float y, const char* value);
    GLuint program_ = 0, vao_ = 0, vbo_ = 0;
    GLint size_location_ = -1;
    std::vector<Vertex> vertices_;
    EventWindow::Clock::time_point next_update_{};
};
} // namespace esdf_drm

#endif // PERFORMANCE_HUD_H
