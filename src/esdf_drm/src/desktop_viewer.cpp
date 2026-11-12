// Desktop presentation consumes the renderer's private NV12 output. No camera,
// mapper, codec, KMS ownership, or CPU pixel staging is involved here.
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sys/sys_api.h>
#include <sys/vb_api.h>
#undef SUCCESS
#undef FAILURE

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <drm_fourcc.h>

#include <poll.h>
#include <pwd.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace
{
using FrameMessage = mpp_msgs::msg::MppVideoFrameDesc;

constexpr std::uint32_t kFrameDescriptorVersion = 1;
constexpr std::uint32_t kNv12PlaneCount = 2;
constexpr int kMinimumWindowDimension = 64;
constexpr int kMaximumFrameDimension = 16384;
constexpr std::uint64_t kDisplayLogIntervalFrames = 300;

struct CloseDisplay
{
    void operator()(Display* display) const noexcept
    {
        if (display != nullptr)
            XCloseDisplay(display);
    }
};
using OwnedDisplay = std::unique_ptr<Display, CloseDisplay>;

// Connect before ROS/MPP create threads: only X11 authentication uses the
// explicitly selected desktop identity, while DMA-BUF access keeps its caller.
// Returns an empty display without an override; invalid identities throw before
// any ROS/MPP thread is started. The caller owns the resulting X11 connection.
OwnedDisplay ConnectDesktopSession()
{
    const char* uid_text = std::getenv("SPACEMIT_DESKTOP_UID");
    if (uid_text == nullptr)
        return {};
    std::uint64_t parsed_uid = 0;
    const char* end = uid_text + std::strlen(uid_text);
    const auto conversion = std::from_chars(uid_text, end, parsed_uid);
    if (conversion.ec != std::errc{} || conversion.ptr != end ||
        parsed_uid >= std::numeric_limits<uid_t>::max())
        throw std::runtime_error("SPACEMIT_DESKTOP_UID must be a decimal user ID");
    const auto desktop_uid = static_cast<uid_t>(parsed_uid);
    if (getpwuid(desktop_uid) == nullptr)
        throw std::runtime_error("SPACEMIT_DESKTOP_UID does not identify a local account");
    const uid_t original_uid = geteuid();
    if (original_uid != 0 && desktop_uid != original_uid)
        throw std::runtime_error("Only root may select a different desktop user ID");
    const bool switch_uid = desktop_uid != original_uid;
    if (switch_uid && seteuid(desktop_uid) != 0)
        throw std::runtime_error("Cannot select desktop effective user ID");
    OwnedDisplay display(XOpenDisplay(nullptr));
    // No C++ operation that can throw occurs between the credential switch and
    // this restoration. A failure exits before initializing ROS or MPP.
    if (switch_uid && seteuid(original_uid) != 0)
        throw std::runtime_error("Cannot restore runtime effective user ID");
    if (!display)
        throw std::runtime_error("Cannot connect to the selected user's X11 desktop");
    std::cerr << "esdf_desktop_viewer: desktop UID=" << desktop_uid
                << " runtime effective UID=" << original_uid << '\n';
    return display;
}

// Wake an eventfd consumer; the fd must remain valid until ROS shutdown ends.
void NotifyEvent(int event_fd)
{
    const std::uint64_t event_value = 1;
    while (write(event_fd, &event_value, sizeof(event_value)) < 0 && errno == EINTR)
    {
    }
}

class EventSignal
{
public:
    EventSignal() : fd_(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK))
    {
        if (fd_ < 0)
            throw std::runtime_error("eventfd failed");
    }
    ~EventSignal() { close(fd_); }
    int GetFd() const { return fd_; }
    void Drain() const
    {
        std::uint64_t event_value;
        while (read(fd_, &event_value, sizeof(event_value)) < 0 && errno == EINTR)
        {
        }
    }

private:
    int fd_;
};

class MppRuntime
{
public:
    MppRuntime()
    {
        if (SYS_Init() != 0)
            throw std::runtime_error("SYS_Init failed");
        if (VB_Init() != 0)
        {
            (void)SYS_Exit();
            throw std::runtime_error("VB_Init failed");
        }
    }
    ~MppRuntime()
    {
        (void)VB_Exit();
        (void)SYS_Exit();
    }
};

struct ImportedFrame
{
    explicit ImportedFrame(const FrameMessage& message) : descriptor(message) {}
    ~ImportedFrame()
    {
        if (handle != 0)
            (void)VB_ReleaseBuffer(handle);
    }
    ImportedFrame(const ImportedFrame&) = delete;
    ImportedFrame& operator=(const ImportedFrame&) = delete;
    FrameMessage descriptor;
    UL handle = 0;
    S32 dma_fd = -1;
};

// Validate NV12 allocation extents before importing any producer-owned buffer.
bool IsValidDescriptor(const FrameMessage& frame)
{
    const std::uint64_t y_bytes = static_cast<std::uint64_t>(frame.plane_stride[0]) * frame.height;
    const std::uint64_t uv_bytes =
        static_cast<std::uint64_t>(frame.plane_stride[1]) * (frame.height / 2U);
    return frame.version == kFrameDescriptorVersion && frame.token != 0 &&
        frame.pixel_format == MPP_PIXEL_FORMAT_NV12 && frame.compress_mode == COMPRESS_MODE_NONE &&
        frame.plane_num == kNv12PlaneCount && frame.width > 0 && frame.height > 0 &&
        frame.width <= kMaximumFrameDimension && frame.height <= kMaximumFrameDimension &&
        (frame.width & 1U) == 0 && (frame.height & 1U) == 0 &&
        frame.plane_stride[0] >= frame.width && frame.plane_stride[1] >= frame.width &&
        frame.plane_size[0] >= y_bytes && frame.plane_size[1] >= uv_bytes &&
        frame.plane_size_valid[0] >= y_bytes && frame.plane_size_valid[1] >= uv_bytes &&
        static_cast<std::uint64_t>(frame.plane_size[0]) + frame.plane_size[1] <= frame.total_size &&
        frame.total_size <= static_cast<std::uint32_t>(std::numeric_limits<EGLint>::max());
}

class DescriptorViewer : public rclcpp::Node
{
public:
    explicit DescriptorViewer(int event_fd) : Node("esdf_desktop_viewer"), event_fd_(event_fd)
    {
        const auto topic = declare_parameter<std::string>("input_topic", "/esdf/render/frame_desc");
        width = declare_parameter<int>("window_width", 1280);
        height = declare_parameter<int>("window_height", 720);
        fullscreen = declare_parameter<bool>("fullscreen", false);
        allow_software_rendering = declare_parameter<bool>("allow_software_rendering", false);
        title = declare_parameter<std::string>("title", "ROBOT VIO / Depth / TSDF / ESDF");
        if (topic.empty() || width < kMinimumWindowDimension || height < kMinimumWindowDimension ||
            width > kMaximumFrameDimension || height > kMaximumFrameDimension)
            throw std::runtime_error("invalid desktop viewer topic or window dimensions");
        subscription_ =
            create_subscription<FrameMessage>(topic, rclcpp::SensorDataQoS().keep_last(1),
                [this](FrameMessage::ConstSharedPtr message) { ReceiveFrame(*message); });
        RCLCPP_INFO(get_logger(), "Desktop NV12 input: %s; latest-frame queue=1", topic.c_str());
    }

    // Thread-safe ownership transfer; the caller releases the returned VB lease.
    std::unique_ptr<ImportedFrame> TakeLatest()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(pending_, {});
    }
    int width = 1280;
    int height = 720;
    bool fullscreen = false;
    bool allow_software_rendering = false;
    std::string title;

private:
    void ReceiveFrame(const FrameMessage& message)
    {
        if (!IsValidDescriptor(message))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Invalid NV12 descriptor skipped");
            return;
        }
        // Drop the old pending lease before importing its replacement. Together
        // with the frame being drawn, imports are strictly bounded to two.
        TakeLatest().reset();
        auto frame = std::make_unique<ImportedFrame>(message);
        if (VB_Import(message.token, &frame->handle) != 0 || frame->handle == 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "NV12 token import failed (possibly expired); frame skipped");
            return;
        }
        if (VB_GetDmaBufFd(frame->handle, &frame->dma_fd) != 0 || frame->dma_fd < 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "NV12 DMA-BUF lookup failed");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame.swap(pending_);
        }
        NotifyEvent(event_fd_);
    }

    int event_fd_;
    std::mutex mutex_;
    std::unique_ptr<ImportedFrame> pending_;
    rclcpp::Subscription<FrameMessage>::SharedPtr subscription_;
};

bool HasExtension(const char* extensions, const char* name)
{
    if (extensions == nullptr)
        return false;
    const std::string all = std::string(" ") + extensions + " ";
    return all.find(std::string(" ") + name + " ") != std::string::npos;
}

// GL APIs are also implemented by software rasterizers; a valid context alone
// cannot prove that this DMA-BUF presentation path has hardware acceleration.
bool IsSoftwareRenderer(const char* renderer)
{
    std::string name(renderer);
    std::transform(name.begin(), name.end(), name.begin(),
        [](unsigned char character) { return std::tolower(character); });
    return name.find("llvmpipe") != std::string::npos ||
        name.find("softpipe") != std::string::npos || name.find("swrast") != std::string::npos ||
        name.find("software rasterizer") != std::string::npos;
}

// Return a compiled shader owned by the caller; delete it after program linking.
// Compilation errors throw after releasing the failed shader.
GLuint CompileShader(GLenum kind, const char* source)
{
    const GLuint shader = glCreateShader(kind);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE)
    {
        char log[1024]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("Desktop shader compilation failed: ") + log);
    }
    return shader;
}

// All EGL/GL operations, including destruction, run on the presentation thread.
// DDS callbacks only hand over imported leases and never access the GL context.
class DesktopWindow
{
public:
    ~DesktopWindow()
    {
        if (context_ != EGL_NO_CONTEXT)
        {
            glDeleteProgram(program_);
            glDeleteVertexArrays(1, &vertex_array_);
            (void)eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            (void)eglDestroyContext(egl_display_, context_);
        }
        if (surface_ != EGL_NO_SURFACE)
            (void)eglDestroySurface(egl_display_, surface_);
        if (egl_display_ != EGL_NO_DISPLAY)
            (void)eglTerminate(egl_display_);
        if (window_ != 0)
            XDestroyWindow(display_, window_);
        if (colormap_ != 0)
            XFreeColormap(display_, colormap_);
        if (display_ != nullptr)
            XCloseDisplay(display_);
    }

    void Initialize(const DescriptorViewer& options, OwnedDisplay desktop_display)
    {
        width_ = options.width;
        height_ = options.height;
        display_ = desktop_display ? desktop_display.release() : XOpenDisplay(nullptr);
        if (display_ == nullptr)
            throw std::runtime_error("Cannot open X11 desktop; set DISPLAY and desktop XAUTHORITY");
        const auto get_platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
            eglGetProcAddress("eglGetPlatformDisplayEXT"));
        egl_display_ = get_platform_display != nullptr
            ? get_platform_display(EGL_PLATFORM_X11_EXT, display_, nullptr)
            : eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(display_));
        if (egl_display_ == EGL_NO_DISPLAY ||
            eglInitialize(egl_display_, nullptr, nullptr) != EGL_TRUE ||
            eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE)
            throw std::runtime_error("Cannot initialize desktop X11 EGL");
        if (!HasExtension(
                eglQueryString(egl_display_, EGL_EXTENSIONS), "EGL_EXT_image_dma_buf_import"))
            throw std::runtime_error("Desktop EGL lacks EGL_EXT_image_dma_buf_import");

        const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_RENDERABLE_TYPE,
            EGL_OPENGL_ES3_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_NONE};
        EGLConfig config = nullptr;
        EGLint count = 0;
        if (eglChooseConfig(egl_display_, attributes, &config, 1, &count) != EGL_TRUE || count != 1)
            throw std::runtime_error("No desktop EGL GLES3 window configuration");
        EGLint visual_id = 0;
        if (eglGetConfigAttrib(egl_display_, config, EGL_NATIVE_VISUAL_ID, &visual_id) != EGL_TRUE)
            throw std::runtime_error("Cannot query desktop EGL visual");
        XVisualInfo match{};
        match.visualid = static_cast<VisualID>(visual_id);
        int visual_count = 0;
        XVisualInfo* visual = XGetVisualInfo(display_, VisualIDMask, &match, &visual_count);
        if (visual == nullptr || visual_count < 1)
        {
            if (visual != nullptr)
                XFree(visual);
            throw std::runtime_error("Desktop EGL visual is unavailable in X11");
        }
        const Window root = RootWindow(display_, visual->screen);
        colormap_ = XCreateColormap(display_, root, visual->visual, AllocNone);
        XSetWindowAttributes window_attributes{};
        window_attributes.colormap = colormap_;
        window_attributes.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
        window_ = XCreateWindow(display_, root, 0, 0, static_cast<unsigned>(width_),
            static_cast<unsigned>(height_), 0, visual->depth, InputOutput, visual->visual,
            CWColormap | CWEventMask, &window_attributes);
        XFree(visual);
        if (window_ == 0)
            throw std::runtime_error("Cannot create desktop X11 window");
        XStoreName(display_, window_, options.title.c_str());
        delete_window_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(display_, window_, &delete_window_, 1);
        if (options.fullscreen)
        {
            const Atom state = XInternAtom(display_, "_NET_WM_STATE", False);
            const Atom fullscreen = XInternAtom(display_, "_NET_WM_STATE_FULLSCREEN", False);
            XChangeProperty(display_, window_, state, XA_ATOM, 32, PropModeReplace,
                reinterpret_cast<const unsigned char*>(&fullscreen), 1);
        }
        const EGLint context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        context_ = eglCreateContext(egl_display_, config, EGL_NO_CONTEXT, context_attributes);
        surface_ = eglCreateWindowSurface(
            egl_display_, config, static_cast<EGLNativeWindowType>(window_), nullptr);
        if (context_ == EGL_NO_CONTEXT || surface_ == EGL_NO_SURFACE ||
            eglMakeCurrent(egl_display_, surface_, surface_, context_) != EGL_TRUE)
            throw std::runtime_error("Cannot create desktop EGL window/context");
        const auto* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
        if (renderer == nullptr)
            throw std::runtime_error("Cannot query desktop GLES renderer");
        if (IsSoftwareRenderer(renderer) && !options.allow_software_rendering)
            throw std::runtime_error(std::string("Desktop EGL selected CPU software renderer '") +
                renderer +
                "'. Install the matching board GPU driver packages and log in to a fresh "
                "desktop session. Set allow_software_rendering:=true only for diagnostics.");
        if (!HasExtension(reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS)),
                "GL_OES_EGL_image_external_essl3"))
            throw std::runtime_error("Desktop GLES lacks GL_OES_EGL_image_external_essl3");
        create_image_ =
            reinterpret_cast<PFNEGLCREATEIMAGEKHRPROC>(eglGetProcAddress("eglCreateImageKHR"));
        destroy_image_ =
            reinterpret_cast<PFNEGLDESTROYIMAGEKHRPROC>(eglGetProcAddress("eglDestroyImageKHR"));
        image_texture_ = reinterpret_cast<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>(
            eglGetProcAddress("glEGLImageTargetTexture2DOES"));
        if (create_image_ == nullptr || destroy_image_ == nullptr || image_texture_ == nullptr)
            throw std::runtime_error("Desktop EGLImage entry points are unavailable");
        InitializeShader();
        glGenVertexArrays(1, &vertex_array_);
        (void)eglSwapInterval(egl_display_, 1);
        XMapWindow(display_, window_);
        XFlush(display_);
        glClearColor(0.015F, 0.020F, 0.030F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        if (eglSwapBuffers(egl_display_, surface_) != EGL_TRUE)
            throw std::runtime_error("Initial desktop EGL swap failed");
        RCLCPP_INFO(options.get_logger(),
            "Desktop EGL ready: vendor=%s GLES=%s renderer=%s fullscreen=%s",
            eglQueryString(egl_display_, EGL_VENDOR),
            reinterpret_cast<const char*>(glGetString(GL_VERSION)), renderer,
            options.fullscreen ? "true" : "false");
    }

    int GetConnectionFd() const { return ConnectionNumber(display_); }

    bool ProcessEvents()
    {
        while (XPending(display_) > 0)
        {
            XEvent event{};
            XNextEvent(display_, &event);
            if (event.type == ClientMessage &&
                static_cast<Atom>(event.xclient.data.l[0]) == delete_window_)
                return false;
            if (event.type == DestroyNotify)
            {
                window_ = 0;
                return false;
            }
            if (event.type == KeyPress && XLookupKeysym(&event.xkey, 0) == XK_Escape)
                return false;
            if (event.type == ConfigureNotify)
            {
                width_ = std::max(1, event.xconfigure.width);
                height_ = std::max(1, event.xconfigure.height);
            }
        }
        // The compositor retains the presented surface. Expose/resize does not
        // require retaining the previous producer token or importing it again.
        return true;
    }

    void PresentFrame(std::unique_ptr<ImportedFrame> frame)
    {
        const auto& descriptor = frame->descriptor;
        const EGLint attributes[] = {EGL_WIDTH, static_cast<EGLint>(descriptor.width), EGL_HEIGHT,
            static_cast<EGLint>(descriptor.height), EGL_LINUX_DRM_FOURCC_EXT,
            static_cast<EGLint>(DRM_FORMAT_NV12), EGL_DMA_BUF_PLANE0_FD_EXT, frame->dma_fd,
            EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0, EGL_DMA_BUF_PLANE0_PITCH_EXT,
            static_cast<EGLint>(descriptor.plane_stride[0]), EGL_DMA_BUF_PLANE1_FD_EXT,
            frame->dma_fd, EGL_DMA_BUF_PLANE1_OFFSET_EXT,
            static_cast<EGLint>(descriptor.plane_size[0]), EGL_DMA_BUF_PLANE1_PITCH_EXT,
            static_cast<EGLint>(descriptor.plane_stride[1]), EGL_YUV_COLOR_SPACE_HINT_EXT,
            EGL_ITU_REC709_EXT, EGL_SAMPLE_RANGE_HINT_EXT, EGL_YUV_NARROW_RANGE_EXT, EGL_NONE};
        const EGLImageKHR image =
            create_image_(egl_display_, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attributes);
        if (image == EGL_NO_IMAGE_KHR)
        {
            const auto error = eglGetError();
            char code[16];
            std::snprintf(code, sizeof(code), "0x%04x", static_cast<unsigned>(error));
            throw std::runtime_error(
                std::string("Desktop EGL cannot import NV12 DMA-BUF: ") + code);
        }
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        image_texture_(GL_TEXTURE_EXTERNAL_OES, image);
        glViewport(0, 0, width_, height_);
        glClear(GL_COLOR_BUFFER_BIT);
        const double scale = std::min(static_cast<double>(width_) / descriptor.width,
            static_cast<double>(height_) / descriptor.height);
        const int draw_width = std::max(1, static_cast<int>(descriptor.width * scale));
        const int draw_height = std::max(1, static_cast<int>(descriptor.height * scale));
        glViewport((width_ - draw_width) / 2, (height_ - draw_height) / 2, draw_width, draw_height);
        glUseProgram(program_);
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        // Complete sampling before destroying the EGLImage and releasing the
        // VB import. The desktop surface contains its own rendered pixels.
        glFinish();
        const GLenum error = glGetError();
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);
        glDeleteTextures(1, &texture);
        (void)destroy_image_(egl_display_, image);
        frame.reset();
        if (error != GL_NO_ERROR)
            throw std::runtime_error("Desktop NV12 draw failed: " + std::to_string(error));
        if (eglSwapBuffers(egl_display_, surface_) != EGL_TRUE)
            throw std::runtime_error("Desktop EGL swap failed");
    }

private:
    void InitializeShader()
    {
        const char* vertex = R"(#version 300 es
out vec2 uv;
void main() {
    const vec2 corners[4] = vec2[4](vec2(-1., -1.), vec2(1., -1.),
                                    vec2(-1., 1.), vec2(1., 1.));
    vec2 p = corners[gl_VertexID];
    gl_Position = vec4(p, 0., 1.);
    uv = vec2((p.x + 1.) * .5, (1. - p.y) * .5);
}
)";
        const char* fragment = R"(#version 300 es
#extension GL_OES_EGL_image_external_essl3 : require
precision mediump float;
uniform samplerExternalOES source;
in vec2 uv;
out vec4 color;
void main() { color = texture(source, uv); }
)";
        const GLuint vertex_shader = CompileShader(GL_VERTEX_SHADER, vertex);
        GLuint fragment_shader = 0;
        try
        {
            fragment_shader = CompileShader(GL_FRAGMENT_SHADER, fragment);
        }
        catch (...)
        {
            glDeleteShader(vertex_shader);
            throw;
        }
        program_ = glCreateProgram();
        glAttachShader(program_, vertex_shader);
        glAttachShader(program_, fragment_shader);
        glLinkProgram(program_);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        GLint linked = GL_FALSE;
        glGetProgramiv(program_, GL_LINK_STATUS, &linked);
        if (linked != GL_TRUE)
            throw std::runtime_error("Desktop NV12 shader link failed");
        glUseProgram(program_);
        glUniform1i(glGetUniformLocation(program_, "source"), 0);
    }

    Display* display_ = nullptr;
    Window window_ = 0;
    Colormap colormap_ = 0;
    Atom delete_window_ = 0;
    int width_ = 0;
    int height_ = 0;
    EGLDisplay egl_display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    GLuint program_ = 0;
    GLuint vertex_array_ = 0;
    PFNEGLCREATEIMAGEKHRPROC create_image_ = nullptr;
    PFNEGLDESTROYIMAGEKHRPROC destroy_image_ = nullptr;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC image_texture_ = nullptr;
};

class ExecutorRunner
{
public:
    explicit ExecutorRunner(const std::shared_ptr<DescriptorViewer>& node)
    {
        executor_.add_node(node);
        thread_ = std::thread(
            [this, node]
            {
                try
                {
                    executor_.spin();
                }
                catch (const std::exception& error)
                {
                    RCLCPP_ERROR(node->get_logger(), "Desktop DDS failed: %s", error.what());
                    failed_.store(true);
                    rclcpp::shutdown();
                }
            });
    }
    ~ExecutorRunner()
    {
        executor_.cancel();
        if (thread_.joinable())
            thread_.join();
    }
    bool HasFailed() const { return failed_.load(); }

private:
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread thread_;
    std::atomic<bool> failed_{false};
};
} // namespace

int main(int argc, char** argv)
{
    int result = 0;
    bool ros_initialized = false;
    try
    {
        auto desktop_display = ConnectDesktopSession();
        rclcpp::init(argc, argv);
        ros_initialized = true;
        EventSignal wake;
        rclcpp::on_shutdown([fd = wake.GetFd()] { NotifyEvent(fd); });
        try
        {
            MppRuntime mpp;
            auto node = std::make_shared<DescriptorViewer>(wake.GetFd());
            DesktopWindow window;
            window.Initialize(*node, std::move(desktop_display));
            ExecutorRunner executor(node);
            std::uint64_t displayed = 0;
            while (rclcpp::ok() && window.ProcessEvents())
            {
                auto frame = node->TakeLatest();
                if (frame)
                {
                    window.PresentFrame(std::move(frame));
                    ++displayed;
                    if (displayed == 1 || displayed % kDisplayLogIntervalFrames == 0)
                        RCLCPP_INFO(node->get_logger(), "Desktop displayed frames=%llu",
                            static_cast<unsigned long long>(displayed));
                    continue;
                }
                pollfd events[] = {
                    {window.GetConnectionFd(), POLLIN, 0}, {wake.GetFd(), POLLIN, 0}};
                const int ready = poll(events, 2, -1);
                if (ready < 0 && errno != EINTR)
                    throw std::runtime_error("Desktop event wait failed");
                if ((events[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
                    throw std::runtime_error("Desktop X11 connection closed");
                if ((events[1].revents & POLLIN) != 0)
                    wake.Drain();
            }
            result = executor.HasFailed() ? 1 : 0;
            RCLCPP_INFO(node->get_logger(), "Desktop viewer stopped after %llu frames",
                static_cast<unsigned long long>(displayed));
        }
        catch (const std::exception& error)
        {
            std::cerr << "esdf_desktop_viewer: " << error.what() << '\n';
            result = 1;
        }
        // Wake callback's fd remains valid until ROS signal handling is stopped.
        if (ros_initialized)
            rclcpp::shutdown();
    }
    catch (const std::exception& error)
    {
        std::cerr << "esdf_desktop_viewer: " << error.what() << '\n';
        if (ros_initialized)
            rclcpp::shutdown();
        result = 1;
    }
    return result;
}
