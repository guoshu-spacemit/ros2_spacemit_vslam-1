// Internal native camera support for mpp decoder input pool.
#ifndef MPP_DECODER_INPUT_POOL_H
#define MPP_DECODER_INPUT_POOL_H

#include "spacemit_camera/decoder_input_pool_config.h"
#include "sys_api.h"
#include "vdec/vdec_api.h"

namespace camera_sdk::detail
{

inline void validate_decoder_input_pool(const DecoderInputPoolConfig& config)
{
    config.validate();
#ifndef CAMERA_SDK_MPP_HAS_INPUT_POOL
    if (config.enabled)
    {
        throw std::invalid_argument(
            "decoder input pool requires MPP headers and library with "
            "SYS_ConfigStreamDmaBufPool; rebuild against a supporting MPP version");
    }
#endif
}

inline void set_decoder_input_mode(VdecChnAttr& attr, const DecoderInputPoolConfig& config)
{
    validate_decoder_input_pool(config);
#ifdef CAMERA_SDK_MPP_HAS_INPUT_POOL
    attr.bEnableInputDmaBuf = config.enabled;
#else
    (void)attr;
#endif
}

inline int configure_decoder_input_pool(
    const MppNode& source, const MppNode& sink, const DecoderInputPoolConfig& config)
{
    validate_decoder_input_pool(config);
#ifdef CAMERA_SDK_MPP_HAS_INPUT_POOL
    if (config.enabled)
    {
        return SYS_ConfigStreamDmaBufPool(&source, &sink, config.slot_bytes, config.slot_count);
    }
#else
    (void)source;
    (void)sink;
#endif
    return 0;
}

} // namespace camera_sdk::detail

#endif // MPP_DECODER_INPUT_POOL_H
