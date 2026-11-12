/**
 * @file nv12_center_hud.cpp
 * @brief Draw metric center-depth annotations into private NV12 preview buffers.
 */
#include "las2/nv12_center_hud.h"

#define CL_TARGET_OPENCL_VERSION 200
#include <CL/cl.h>
#include <CL/cl_ext.h>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace las2
{
namespace
{

// Each work item owns a complete 2x2 NV12 cell in one panel. No UV byte has
// multiple writers, and padding/pixels outside the small mask remain intact.
constexpr const char* kHudKernel = R"CLC(
__kernel void nv12_center_hud(__global uchar* output,
                                __global const uchar* mask,
                                int mask_width, int mask_height,
                                int origin_x, int origin_y, int first_panel, int panel_width,
                                int height, int y_stride, int uv_stride,
                                uchar chroma_u, uchar chroma_v) {
    const int x = (int)get_global_id(0) * 2;
    const int y = (int)get_global_id(1) * 2;
    const int panel = first_panel + (int)get_global_id(2);
    if (x >= mask_width || y >= mask_height || panel >= 2) return;
    const int output_x = origin_x + panel * panel_width + x;
    const int output_y = origin_y + y;
    int marked = 0;
    for (int dy = 0; dy < 2; ++dy) {
        for (int dx = 0; dx < 2; ++dx) {
            const uchar value = mask[(y + dy) * mask_width + x + dx];
            if (value != 0) {
                output[(output_y + dy) * y_stride + output_x + dx] = value;
                marked = 1;
            }
        }
    }
    if (marked) {
        const int uv = y_stride * height + (output_y / 2) * uv_stride + output_x;
        output[uv] = chroma_u;
        output[uv + 1] = chroma_v;
    }
}
)CLC";

void check_cl(cl_int status, const char* operation)
{
    if (status != CL_SUCCESS)
    {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(status));
    }
}

} // namespace

struct Nv12CenterHud::Impl
{
    struct ImportedOutput
    {
        int fd;
        std::uint64_t size;
        cl_mem memory;
    };

    Impl(std::uint64_t pool, std::size_t limit, int width, int height)
        : pool_id(pool), maximum_imports(limit), width(width), height(height)
    {
        if (pool == 0 || limit == 0 || width < 4 || height < 2 || width % 4 != 0 || height % 2 != 0)
        {
            throw std::invalid_argument("invalid GPU center HUD pool/geometry");
        }
        try
        {
            initialize();
        }
        catch (...)
        {
            cleanup();
            throw;
        }
    }

    ~Impl() { cleanup(); }

    void initialize()
    {
        check_cl(clGetPlatformIDs(1, &platform, nullptr), "HUD clGetPlatformIDs");
        check_cl(clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr),
            "HUD clGetDeviceIDs GPU");
        cl_int status = CL_SUCCESS;
        context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &status);
        check_cl(status, "HUD clCreateContext");
        queue = clCreateCommandQueueWithProperties(context, device, nullptr, &status);
        check_cl(status, "HUD clCreateCommandQueueWithProperties");
        import_memory = reinterpret_cast<clImportMemoryARM_fn>(
            clGetExtensionFunctionAddressForPlatform(platform, "clImportMemoryARM"));
        acquire = reinterpret_cast<clEnqueueAcquireExternalMemObjectsKHR_fn>(
            clGetExtensionFunctionAddressForPlatform(
                platform, "clEnqueueAcquireExternalMemObjectsKHR"));
        release = reinterpret_cast<clEnqueueReleaseExternalMemObjectsKHR_fn>(
            clGetExtensionFunctionAddressForPlatform(
                platform, "clEnqueueReleaseExternalMemObjectsKHR"));
        if (!import_memory || !acquire || !release)
        {
            throw std::runtime_error(
                "GPU center HUD requires ARM DMA-BUF import and KHR acquire/release");
        }
        const char* source = kHudKernel;
        const std::size_t length = std::strlen(source);
        program = clCreateProgramWithSource(context, 1, &source, &length, &status);
        check_cl(status, "HUD clCreateProgramWithSource");
        status = clBuildProgram(program, 1, &device, "", nullptr, nullptr);
        if (status != CL_SUCCESS)
        {
            std::size_t log_size = 0;
            (void)clGetProgramBuildInfo(
                program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
            std::string log(log_size, '\0');
            (void)clGetProgramBuildInfo(
                program, device, CL_PROGRAM_BUILD_LOG, log.size(), log.data(), nullptr);
            throw std::runtime_error("GPU center HUD kernel build failed: " + log);
        }
        kernel = clCreateKernel(program, "nv12_center_hud", &status);
        check_cl(status, "HUD clCreateKernel");
        // One persistent upload buffer serves the label and a configurable
        // box of at most 31x31 depth pixels plus its one-pixel surround.
        const auto box_side = static_cast<std::size_t>(std::min({width / 2, height, 31}) + 2);
        mask_capacity = std::max(
            {std::size_t{64}, static_cast<std::size_t>(width / 2) * 20, box_side * box_side});
        mask_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY, mask_capacity, nullptr, &status);
        check_cl(status, "HUD clCreateBuffer mask");
    }

    void cleanup() noexcept
    {
        if (queue)
            (void)clFinish(queue);
        for (const auto& entry : imports)
        {
            (void)clReleaseMemObject(entry.second.memory);
        }
        imports.clear();
        if (mask_buffer)
            (void)clReleaseMemObject(std::exchange(mask_buffer, nullptr));
        if (kernel)
            (void)clReleaseKernel(std::exchange(kernel, nullptr));
        if (program)
            (void)clReleaseProgram(std::exchange(program, nullptr));
        if (queue)
            (void)clReleaseCommandQueue(std::exchange(queue, nullptr));
        if (context)
            (void)clReleaseContext(std::exchange(context, nullptr));
    }

    cl_mem output_memory(std::uint64_t buffer_id, int fd, std::uint64_t size)
    {
        const auto found = imports.find(buffer_id);
        if (found != imports.end())
        {
            if (found->second.fd != fd || found->second.size != size)
                throw std::runtime_error("GPU center HUD preview buffer identity changed");
            return found->second.memory;
        }
        if (imports.size() >= maximum_imports)
            throw std::runtime_error("GPU center HUD bounded import cache exhausted");
        const cl_import_properties_arm properties[] = {
            CL_IMPORT_TYPE_ARM, CL_IMPORT_TYPE_DMA_BUF_ARM, 0};
        cl_int status = CL_SUCCESS;
        cl_mem memory = import_memory(
            context, CL_MEM_READ_WRITE, properties, &fd, static_cast<std::size_t>(size), &status);
        check_cl(status, "HUD clImportMemoryARM");
        if (!memory)
            throw std::runtime_error("HUD clImportMemoryARM returned null");
        try
        {
            imports.emplace(buffer_id, ImportedOutput{fd, size, memory});
        }
        catch (...)
        {
            (void)clReleaseMemObject(memory);
            throw;
        }
        return memory;
    }

    void enqueue_mask(cl_mem output, const CenterDepthMask& mask)
    {
        if (mask.pixels.empty())
            return;
        if (mask.pixels.size() > mask_capacity)
            throw std::runtime_error("GPU center HUD mask exceeds fixed capacity");
        if (mask.first_panel < 0 || mask.panel_count < 1 || mask.first_panel + mask.panel_count > 2)
            throw std::runtime_error("GPU center HUD mask has invalid panel range");
        // Blocking upload allows the small CPU mask to be destroyed safely on
        // an ensuing kernel/argument error. No NV12 CPU mapping is accessed.
        check_cl(clEnqueueWriteBuffer(queue, mask_buffer, CL_TRUE, 0, mask.pixels.size(),
                        mask.pixels.data(), 0, nullptr, nullptr),
            "HUD clEnqueueWriteBuffer mask");
        int arg = 0;
        check_cl(clSetKernelArg(kernel, arg++, sizeof(output), &output), "HUD output argument");
        check_cl(
            clSetKernelArg(kernel, arg++, sizeof(mask_buffer), &mask_buffer), "HUD mask argument");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(mask.width), &mask.width), "HUD mask width");
        check_cl(
            clSetKernelArg(kernel, arg++, sizeof(mask.height), &mask.height), "HUD mask height");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(mask.x), &mask.x), "HUD origin x");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(mask.y), &mask.y), "HUD origin y");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(mask.first_panel), &mask.first_panel),
            "HUD first panel");
        const int panel_width = width / 2;
        check_cl(
            clSetKernelArg(kernel, arg++, sizeof(panel_width), &panel_width), "HUD panel width");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(height), &height), "HUD output height");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(y_stride), &y_stride), "HUD Y stride");
        check_cl(clSetKernelArg(kernel, arg++, sizeof(uv_stride), &uv_stride), "HUD UV stride");
        check_cl(
            clSetKernelArg(kernel, arg++, sizeof(mask.chroma_u), &mask.chroma_u), "HUD chroma U");
        check_cl(
            clSetKernelArg(kernel, arg++, sizeof(mask.chroma_v), &mask.chroma_v), "HUD chroma V");
        const std::size_t global[] = {static_cast<std::size_t>(mask.width / 2),
            static_cast<std::size_t>(mask.height / 2), static_cast<std::size_t>(mask.panel_count)};
        check_cl(
            clEnqueueNDRangeKernel(queue, kernel, 3, nullptr, global, nullptr, 0, nullptr, nullptr),
            "HUD clEnqueueNDRangeKernel");
    }

    std::uint64_t pool_id;
    std::size_t maximum_imports;
    int width;
    int height;
    int y_stride{0};
    int uv_stride{0};
    std::size_t mask_capacity{0};
    std::mutex mutex;
    bool failed{false};
    cl_platform_id platform{nullptr};
    cl_device_id device{nullptr};
    cl_context context{nullptr};
    cl_command_queue queue{nullptr};
    cl_program program{nullptr};
    cl_kernel kernel{nullptr};
    cl_mem mask_buffer{nullptr};
    clImportMemoryARM_fn import_memory{nullptr};
    clEnqueueAcquireExternalMemObjectsKHR_fn acquire{nullptr};
    clEnqueueReleaseExternalMemObjectsKHR_fn release{nullptr};
    std::unordered_map<std::uint64_t, ImportedOutput> imports;
};

Nv12CenterHud::Nv12CenterHud(
    std::uint64_t pool_id, std::size_t maximum_imports, int width, int height)
    : impl_(std::make_unique<Impl>(pool_id, maximum_imports, width, height))
{
}

Nv12CenterHud::~Nv12CenterHud() = default;

bool Nv12CenterHud::render(std::uint64_t pool_id, std::uint64_t buffer_id, int dma_fd, int y_stride,
    int uv_stride, std::uint64_t total_size, const CenterDepthSample& sample, std::string& error)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    cl_mem output = nullptr;
    bool acquired = false;
    try
    {
        if (impl_->failed)
            throw std::runtime_error("GPU center HUD disabled after previous error");
        const auto y_size = static_cast<std::uint64_t>(y_stride) * impl_->height;
        const auto uv_size = static_cast<std::uint64_t>(uv_stride) * (impl_->height / 2);
        if (pool_id != impl_->pool_id || buffer_id == 0 || dma_fd < 0 || y_stride < impl_->width ||
            uv_stride < impl_->width || total_size < y_size || total_size - y_size < uv_size ||
            total_size > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("invalid GPU center HUD preview descriptor");
        const auto masks = make_center_depth_masks(impl_->width / 2, impl_->height, sample);
        impl_->y_stride = y_stride;
        impl_->uv_stride = uv_stride;
        output = impl_->output_memory(buffer_id, dma_fd, total_size);
        check_cl(impl_->acquire(impl_->queue, 1, &output, 0, nullptr, nullptr), "HUD acquire");
        acquired = true;
        for (const auto& mask : masks)
            impl_->enqueue_mask(output, mask);
        check_cl(impl_->release(impl_->queue, 1, &output, 0, nullptr, nullptr), "HUD release");
        acquired = false;
        check_cl(clFinish(impl_->queue), "HUD clFinish");
        error.clear();
        return true;
    }
    catch (const std::exception& exception)
    {
        // Caller quarantines this private VB lease until shutdown, so a failed
        // driver synchronization cannot race buffer reuse or publish partial HUD.
        impl_->failed = true;
        error = exception.what();
        if (acquired)
        {
            const cl_int released = impl_->release(impl_->queue, 1, &output, 0, nullptr, nullptr);
            if (released != CL_SUCCESS)
                error += "; cleanup release failed: " + std::to_string(released);
        }
        const cl_int finished = clFinish(impl_->queue);
        if (finished != CL_SUCCESS)
            error += "; cleanup finish failed: " + std::to_string(finished);
        return false;
    }
}

} // namespace las2
