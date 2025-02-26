
#include "hl2ss_lnm.h"

namespace hl2ss
{
namespace lnm
{
//------------------------------------------------------------------------------
// Default Settings
//------------------------------------------------------------------------------

float get_video_codec_default_factor(uint8_t profile)
{
    switch (profile)
    {
    case hl2ss::video_profile::H264_BASE:
    case hl2ss::video_profile::H264_MAIN:
    case hl2ss::video_profile::H264_HIGH: return 1.0f /  75.0f;
    case hl2ss::video_profile::H265_MAIN: return 1.0f / 150.0f;
    default:                              return 1.0f;
    }
}

uint8_t get_video_coded_default_gop_size(uint8_t framerate, uint8_t divisor, uint8_t profile)
{
    return (profile != hl2ss::video_profile::RAW) ? framerate : 1;
}

uint32_t get_video_codec_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, float factor)
{
    return (uint32_t)((double)width*(double)height*(double)framerate*12.0*(double)factor);
}

uint32_t get_video_codec_default_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile)
{
    return get_video_codec_bitrate(width, height, framerate, divisor, get_video_codec_default_factor(profile));
}

std::vector<uint64_t> get_video_codec_default_options(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile)
{
    double  const exposure_factor = 0.0;
    int64_t const constant_factor = -125000;

    std::vector<uint64_t> default_options;
    default_options.push_back(hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize);
    default_options.push_back(get_video_coded_default_gop_size(framerate, divisor, profile));
    default_options.push_back(hl2ss::h26x_encoder_property::HL2SSAPI_VLCHostTicksOffsetExposure);
    default_options.push_back(*(uint64_t*)&exposure_factor);
    default_options.push_back(hl2ss::h26x_encoder_property::HL2SSAPI_VLCHostTicksOffsetConstant);
    default_options.push_back(*(uint64_t*)&constant_factor);
    return default_options;
}

#ifdef HL2SS_ENABLE_DP
dp::mrc_configuration get_mrc_configuration(bool pv, bool holo, bool mic, bool loopback, bool RenderFromCamera, bool vstab, int vstabbuffer)
{
    return {holo, pv, mic, loopback, RenderFromCamera, vstab, vstabbuffer};
}
#endif

//------------------------------------------------------------------------------
// Stream Sync Period
//------------------------------------------------------------------------------

uint64_t get_sync_frame_stamp(uint64_t frame_stamp, uint64_t sync_period)
{
    return frame_stamp + ((sync_period - (frame_stamp % sync_period)) % sync_period);
}

uint64_t get_sync_period(hl2ss::rx const* rx)
{
    switch (rx->port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:     return ((hl2ss::rx_rm_vlc*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:      return ((hl2ss::rx_rm_vlc*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    return ((hl2ss::rx_rm_vlc*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    return ((hl2ss::rx_rm_vlc*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::RM_DEPTH_AHAT:        return ((hl2ss::rx_rm_depth_ahat*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   return 1;
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER: return 1;
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:     return 1;
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  return 1;
    case hl2ss::stream_port::PERSONAL_VIDEO:       return ((hl2ss::rx_pv*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::MICROPHONE:           return 1;
    case hl2ss::stream_port::SPATIAL_INPUT:        return 1;
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: return 1;
    case hl2ss::stream_port::EXTENDED_AUDIO:       return 1;
    case hl2ss::stream_port::EXTENDED_VIDEO:       return ((hl2ss::rx_pv*)rx)->options[hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize];
    case hl2ss::stream_port::EXTENDED_DEPTH:       return 1;
    default:                                       return 1;
    }
}

//------------------------------------------------------------------------------
// Control
//------------------------------------------------------------------------------

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc, bool hologram_composition, bool recording_indicator, bool video_stabilization, bool blank_protected, bool show_mesh, bool shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective)
{
    hl2ss::start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);
}

void stop_subsystem_pv(char const* host, uint16_t port)
{
    hl2ss::stop_subsystem_pv(host, port);
}

//------------------------------------------------------------------------------
// Modes 0, 1
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const* options, bool decoded)
{
    std::vector<uint64_t> default_options;
    if (bitrate <= 0) { bitrate = get_video_codec_default_bitrate(hl2ss::parameters_rm_vlc::WIDTH, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::FPS, divisor, profile); }
    if (options == nullptr)
    {
    default_options = get_video_codec_default_options(hl2ss::parameters_rm_vlc::WIDTH, hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::FPS, divisor, profile);
    options = &default_options;
    }
    return decoded ? std::make_unique<hl2ss::rx_decoded_rm_vlc>(host, port, chunk, mode, divisor, profile, level, bitrate, *options) : std::make_unique<hl2ss::rx_rm_vlc>(host, port, chunk, mode, divisor, profile, level, bitrate, *options);
}

std::unique_ptr<hl2ss::rx_rm_depth_ahat> rx_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const* options, bool decoded)
{
    std::vector<uint64_t> default_options;
    if (bitrate <= 0) { bitrate = get_video_codec_default_bitrate(hl2ss::parameters_rm_depth_ahat::WIDTH, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::FPS, divisor, profile_ab) * ((profile_z == hl2ss::depth_profile::SAME) ? 16 : 1); }
    if (options == nullptr) 
    {
    default_options = get_video_codec_default_options(hl2ss::parameters_rm_depth_ahat::WIDTH, hl2ss::parameters_rm_depth_ahat::HEIGHT, hl2ss::parameters_rm_depth_ahat::FPS, divisor, profile_ab);
    options = &default_options;
    }
    return decoded ? std::make_unique<hl2ss::rx_decoded_rm_depth_ahat>(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, *options) : std::make_unique<hl2ss::rx_rm_depth_ahat>(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, *options);
}

std::unique_ptr<hl2ss::rx_rm_depth_longthrow> rx_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter, bool decoded)
{
    return decoded ? std::make_unique<hl2ss::rx_decoded_rm_depth_longthrow>(host, port, chunk, mode, divisor, png_filter) : std::make_unique<hl2ss::rx_rm_depth_longthrow>(host, port, chunk, mode, divisor, png_filter);
}

std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode)
{
    return std::make_unique<hl2ss::rx_rm_imu>(host, port, chunk, mode);
}

std::unique_ptr<hl2ss::rx_pv> rx_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const* options, uint8_t decoded_format)
{
    std::vector<uint64_t> default_options;
    if (bitrate <= 0) { bitrate = get_video_codec_default_bitrate(width, height, framerate, divisor, profile); }
    if (options == nullptr)
    {
    default_options = get_video_codec_default_options(width, height, framerate, divisor, profile);
    options = &default_options;
    }
    return (decoded_format != hl2ss::video_profile::RAW) ? std::make_unique<hl2ss::rx_decoded_pv>(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, *options, decoded_format) : std::make_unique<hl2ss::rx_pv>(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, *options);
}

std::unique_ptr<hl2ss::rx_microphone> rx_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level, bool decoded)
{
    return decoded ? std::make_unique<hl2ss::rx_decoded_microphone>(host, port, chunk, profile, level) : std::make_unique<hl2ss::rx_microphone>(host, port, chunk, profile, level);
}

std::unique_ptr<hl2ss::rx_si> rx_si(char const* host, uint16_t port, uint64_t chunk)
{
    return std::make_unique<hl2ss::rx_si>(host, port, chunk);
}

std::unique_ptr<hl2ss::rx_eet> rx_eet(char const* host, uint16_t port, uint64_t chunk, uint8_t framerate)
{
    return std::make_unique<hl2ss::rx_eet>(host, port, chunk, framerate);
}

std::unique_ptr<hl2ss::rx_extended_audio> rx_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level, bool decoded)
{
    return decoded ? std::make_unique<hl2ss::rx_decoded_extended_audio>(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level) : std::make_unique<hl2ss::rx_extended_audio>(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level);
}

std::unique_ptr<hl2ss::rx_extended_depth> rx_extended_depth(char const* host, uint16_t port, uint64_t media_index, uint64_t stride_mask, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, bool decoded)
{
    std::vector<uint64_t> options;
    options.push_back(hl2ss::h26x_encoder_property::HL2SSAPI_VideoMediaIndex);
    options.push_back(media_index);
    options.push_back(hl2ss::h26x_encoder_property::HL2SSAPI_VideoStrideMask);
    options.push_back(stride_mask);
    return decoded ? std::make_unique<hl2ss::rx_decoded_extended_depth>(host, port, chunk, mode, divisor, profile_z, options) : std::make_unique<hl2ss::rx_extended_depth>(host, port, chunk, mode, divisor, profile_z, options);
}

#ifdef HL2SS_ENABLE_DP
std::unique_ptr<hl2ss::dp::rx_mrc> rx_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, dp::mrc_configuration const* configuration, uint8_t decoded_format)
{
    dp::mrc_configuration default_configuration;
    if (configuration == nullptr)
    {
    default_configuration = get_mrc_configuration();
    configuration = &default_configuration;
    }
    return (decoded_format != hl2ss::video_profile::RAW) ? std::make_unique<hl2ss::dp::rx_decoded_mrc>(host, port, user, password, chunk, *configuration, decoded_format) : std::make_unique<hl2ss::dp::rx_mrc>(host, port, user, password, chunk, *configuration);
}
#endif

//------------------------------------------------------------------------------
// Mode 2
//------------------------------------------------------------------------------

std::shared_ptr<hl2ss::calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port)
{
    return hl2ss::download_calibration_rm_vlc(host, port);
}

std::shared_ptr<hl2ss::calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port)
{
    return hl2ss::download_calibration_rm_depth_ahat(host, port);
}

std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port)
{
    return hl2ss::download_calibration_rm_depth_longthrow(host, port);
}

std::shared_ptr<hl2ss::calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port)
{
    return hl2ss::download_calibration_rm_imu(host, port);
}

std::shared_ptr<hl2ss::calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate)
{
    return hl2ss::download_calibration_pv(host, port, width, height, framerate);
}

std::shared_ptr<std::vector<uint8_t>> download_devicelist_extended_audio(char const* host, uint16_t port)
{
    return hl2ss::download_devicelist_extended_audio(host, port);
}

std::shared_ptr<std::vector<uint8_t>> download_devicelist_extended_video(char const* host, uint16_t port)
{
    return hl2ss::download_devicelist_extended_video(host, port);
}

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::ipc_rc> ipc_rc(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_rc>(host, port);
}

std::unique_ptr<hl2ss::ipc_sm> ipc_sm(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_sm>(host, port);
}

std::unique_ptr<hl2ss::ipc_su> ipc_su(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_su>(host, port);
}

std::unique_ptr<hl2ss::ipc_vi> ipc_vi(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_vi>(host, port);
}

std::unique_ptr<hl2ss::ipc_umq> ipc_umq(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_umq>(host, port);
}

std::unique_ptr<hl2ss::ipc_gmq> ipc_gmq(char const* host, uint16_t port)
{
    return std::make_unique<hl2ss::ipc_gmq>(host, port);
}
}
}
