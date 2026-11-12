#ifndef MPP_STREAM_COMPAT_H
#define MPP_STREAM_COMPAT_H
#include <type_traits>
#include <utility>

namespace camera_io::detail
{
template <typename Stream, typename = void> struct HasLegacyStreamDmaFd : std::false_type
{
};

template <typename Stream>
struct HasLegacyStreamDmaFd<Stream, std::void_t<decltype(std::declval<Stream&>().s32DmaBufFd)>>
    : std::true_type
{
};

// A CPU packet has no legacy DMA fd. Newer stream structures have no such
// member; leave their zero-initialized VB handle and all packet metadata alone.
template <typename Stream> constexpr void invalidate_legacy_stream_dma_fd(Stream& stream) noexcept
{
    if constexpr (HasLegacyStreamDmaFd<Stream>::value)
        stream.s32DmaBufFd = -1;
}
} // namespace camera_io::detail

#endif // MPP_STREAM_COMPAT_H
