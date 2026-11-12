#ifndef CAMERA_MPP_CONFIG_H
#define CAMERA_MPP_CONFIG_H

#include <string>

#include "spacemit_camera/decoder_input_pool_config.h"
#include "spacemit_camera/exposure_matcher.h"
#include "spacemit_camera/mahony_ahrs.h"
#include "spacemit_camera/startup_alignment.h"

namespace camera_io::detail
{

struct CropConfig
{
    int x{0};
    int y{0};
    int width{0};
    int height{0};
};

struct StreamConfig
{
    bool enabled{true};
    std::string topic;
    std::string camera_info_topic;
    std::string frame_id;
    CropConfig crop;
    int output_width{0};
    int output_height{0};
};

struct BridgeConfig
{
    std::string input_type{"uvc"};
    std::string input_file;
    std::string device;
    int width{0};
    int height{0};
    int fps{0};
    std::string input_format;
    std::string decoded_format;
    bool decode_enabled{true};
    std::string timestamp_source{"ros_clock"};
    bool gpio_sync_enabled{false}, gpio_active_low{false};
    std::string gpio_chip;
    unsigned gpio_line{0};
    camera_sdk::ExposureMatcher::Config gpio_match;
    camera_sdk::StartupAlignment::Config startup_alignment;
    std::string calibration_file;
    bool publish_imu_calibration_tf{true};
    bool metadata_enabled{false};
    std::string metadata_parser;
    CropConfig metadata_region;
    StreamConfig left;
    StreamConfig right;
    StreamConfig full;
    bool full_enabled{false};
    bool full_only{false};
    StreamConfig compressed;
    bool compressed_enabled{false};
    bool imu_enabled{false};
    std::string imu_raw_dds_topic;
    std::string imu_raw_ros_topic;
    bool ahrs_enabled{false};
    std::string imu_fused_dds_topic;
    std::string imu_fused_ros_topic;
    std::string imu_frame_id;
    int domain_id{0};
    bool domain_id_from_environment{false};
    int imu_queue_depth{16};
    float imu_nominal_frequency_hz{0.0F};
    camera_sdk::AhrsConfig ahrs_config;
    float orientation_variance{0.0025F};
    int max_exported_pairs{6};
    int decoder_buffer_count{0};
    camera_sdk::DecoderInputPoolConfig decoder_input_pool;
};

BridgeConfig load_config(const std::string& path, int fps_override);

} // namespace camera_io::detail

#endif // CAMERA_MPP_CONFIG_H
