// Opt-in compressed DMA-BUF input-pool configuration.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_DECODER_INPUT_POOL_CONFIG_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_DECODER_INPUT_POOL_CONFIG_H_

#include <cstdint>
#include <stdexcept>

namespace camera_sdk
{

// Compressed input storage, independent of decoded output buffer count.
// Opt-in: requires MPP's same-process SYS DMA-BUF stream pool API.
struct DecoderInputPoolConfig
{
    bool enabled = false;
    std::uint32_t slot_bytes = 0;
    std::uint32_t slot_count = 12;

    void validate() const
    {
        if (enabled && (slot_bytes == 0 || slot_count == 0 || slot_count > 16))
        {
            throw std::invalid_argument(
                "decoder input pool needs slot_bytes > 0 and slot_count in [1,16]");
        }
    }
};

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_DECODER_INPUT_POOL_CONFIG_H_
