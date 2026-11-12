#ifndef DECODER_INPUT_POOL_MPP_H
#define DECODER_INPUT_POOL_MPP_H
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spacemit_camera/decoder_input_pool_config.h"
#include "sys/sys_api.h"
#include "vdec/vdec_api.h"

namespace camera_io::detail
{

inline camera_sdk::DecoderInputPoolConfig load_decoder_input_pool(
    const boost::property_tree::ptree& root)
{
    camera_sdk::DecoderInputPoolConfig config;
    const auto pool = root.get_child_optional("buffers.decoder_input_pool");
    if (!pool)
    {
        return config;
    }
    if (!pool->data().empty())
    {
        throw std::invalid_argument("buffers.decoder_input_pool must be an object");
    }
    for (const auto& entry : *pool)
    {
        if (pool->count(entry.first) != 1)
        {
            throw std::invalid_argument("duplicate decoder_input_pool option: " + entry.first);
        }
        if (entry.first != "enabled" && entry.first != "slot_bytes" && entry.first != "slot_count")
        {
            throw std::invalid_argument("unknown decoder_input_pool option: " + entry.first);
        }
    }
    config.enabled = pool->count("enabled") ? pool->get<bool>("enabled") : false;
    const auto bytes = pool->count("slot_bytes") ? pool->get<std::int64_t>("slot_bytes") : 0;
    const auto count = pool->count("slot_count") ? pool->get<std::int64_t>("slot_count") : 12;
    if (bytes < 0 || bytes > std::numeric_limits<std::uint32_t>::max() || count < 1 || count > 16)
    {
        throw std::invalid_argument("invalid decoder_input_pool capacity");
    }
    config.slot_bytes = static_cast<std::uint32_t>(bytes);
    config.slot_count = static_cast<std::uint32_t>(count);
    config.validate();
    return config;
}

inline void validate_decoder_input_pool(
    const camera_sdk::DecoderInputPoolConfig& config, bool decode_enabled)
{
    config.validate();
    if (!config.enabled)
    {
        return;
    }
    if (!decode_enabled)
    {
        throw std::invalid_argument("decoder_input_pool requires camera.decode=true");
    }
#ifndef CAMERA_MPP_HAS_INPUT_POOL
    throw std::invalid_argument(
        "decoder_input_pool requires MPP headers and library with SYS_ConfigStreamDmaBufPool; "
        "rebuild with a supporting MPP version or disable this option");
#endif
}

inline void set_decoder_input_pool_mode(
    VdecChnAttr& attr, const camera_sdk::DecoderInputPoolConfig& config)
{
    validate_decoder_input_pool(config, true);
#ifdef CAMERA_MPP_HAS_INPUT_POOL
    attr.bEnableInputDmaBuf = config.enabled;
#else
    (void)attr;
#endif
}

inline S32 configure_decoder_input_pool(
    const MppNode& source, const MppNode& sink, const camera_sdk::DecoderInputPoolConfig& config)
{
    validate_decoder_input_pool(config, true);
#ifdef CAMERA_MPP_HAS_INPUT_POOL
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

} // namespace camera_io::detail

#endif // DECODER_INPUT_POOL_MPP_H
