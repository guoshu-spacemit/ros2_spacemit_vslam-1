#ifndef DMA_IMPORT_AUDIT_H
#define DMA_IMPORT_AUDIT_H
#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

namespace vio
{
struct DmaImportAudit
{
    std::atomic<std::uint64_t> imports{0}, import_failures{0};
    std::atomic<std::uint64_t> release_calls{0}, release_failures{0}, map_errors{0};
    std::atomic<std::uint64_t> cache_start_calls{0}, cache_start_errors{0};
    std::atomic<std::uint64_t> cache_end_calls{0}, cache_end_errors{0}, copies{0};
};

// Ops supplies the SDK import/release calls. No fd or mapping is owned here.
// Owning one successful reference survives a later mapping exception; close
// attempts exactly one release even when the SDK reports a release failure.
template <class Handle, class Ops> class AuditedImportReference
{
public:
    AuditedImportReference(std::uint64_t token, std::shared_ptr<DmaImportAudit> audit, Ops ops)
        : audit_(std::move(audit)), ops_(std::move(ops))
    {
        if (!token || ops_.import(token, &handle_) != 0)
        {
            ++audit_->import_failures;
            throw std::runtime_error("VB_Import failed");
        }
        ++audit_->imports;
        owned_ = true;
        if (handle_ == Handle{})
        {
            ++audit_->map_errors;
            close();
            throw std::runtime_error("VB_Import returned an invalid handle");
        }
    }
    ~AuditedImportReference() { close(); }
    AuditedImportReference(const AuditedImportReference&) = delete;
    AuditedImportReference& operator=(const AuditedImportReference&) = delete;
    Handle handle() const { return handle_; }
    void close() noexcept
    {
        if (!owned_)
            return;
        owned_ = false;
        ++audit_->release_calls;
        const int result = ops_.release(handle_);
        if (result != 0)
        {
            ++audit_->release_failures;
            ops_.release_error(result);
        }
    }

private:
    std::shared_ptr<DmaImportAudit> audit_;
    Ops ops_;
    Handle handle_{};
    bool owned_{false};
};
} // namespace vio

#endif // DMA_IMPORT_AUDIT_H
