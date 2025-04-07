
#include "hl2ss.h"

#ifdef HL2SS_ENABLE_DP
#include "hl2ss_dp.h"
#endif

//******************************************************************************
// Inlines
//******************************************************************************

namespace hl2ss
{
namespace lnm
{
//------------------------------------------------------------------------------
// Default Settings
//------------------------------------------------------------------------------

HL2SS_INLINE
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

HL2SS_INLINE
uint8_t get_video_coded_default_gop_size(uint8_t framerate, uint8_t divisor, uint8_t profile)
{
    return (profile != hl2ss::video_profile::RAW) ? framerate : 1;
}

HL2SS_INLINE
uint32_t get_video_codec_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, float factor)
{
    return (uint32_t)((double)width*(double)height*(double)framerate*12.0*(double)factor);
}

HL2SS_INLINE
uint32_t get_video_codec_default_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile)
{
    return get_video_codec_bitrate(width, height, framerate, divisor, get_video_codec_default_factor(profile));
}

HL2SS_INLINE
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
HL2SS_INLINE
dp::mrc_configuration create_configuration_for_dp_mrc(bool pv=true, bool holo=false, bool mic=true, bool loopback=false, bool RenderFromCamera=true, bool vstab=false, int vstabbuffer=15)
{
    return { holo, pv, mic, loopback, RenderFromCamera, vstab, vstabbuffer };
}
#endif

HL2SS_INLINE
bool get_video_codec_option(std::vector<uint64_t> const& options, uint64_t key, uint64_t& out)
{
    for (size_t i = 0; i < (options.size() / 2); ++i)
    {
    if (options[2*i] != key) { continue; }
    out = options[2*i + 1];
    return true;
    }
    return false;
}
}
}

//******************************************************************************
// Implementation
//******************************************************************************

#ifndef HL2SS_LNM_SHARED

namespace hl2ss
{
namespace lnm
{
//------------------------------------------------------------------------------
// Control
//------------------------------------------------------------------------------

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc=false, bool hologram_composition=true, bool recording_indicator=false, bool video_stabilization=false, bool blank_protected=false, bool show_mesh=false, bool shared=false, float global_opacity=0.9f, float output_width=0.0f, float output_height=0.0f, uint32_t video_stabilization_length=0, uint32_t hologram_perspective=hl2ss::hologram_perspective::PV);
void stop_subsystem_pv(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// Modes 0, 1
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_VLC, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_depth_ahat> rx_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_DEPTH_AHAT, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile_z=hl2ss::depth_profile::SAME, uint8_t profile_ab=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_depth_longthrow> rx_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_DEPTH_LONGTHROW, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t png_filter=hl2ss::png_filter_mode::PAETH, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_IMU, uint8_t mode=hl2ss::stream_mode::MODE_1, bool decoded=true);
std::unique_ptr<hl2ss::rx_pv> rx_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk=hl2ss::chunk_size::PERSONAL_VIDEO, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, uint8_t decoded_format=hl2ss::pv_decoded_format::BGR);
std::unique_ptr<hl2ss::rx_microphone> rx_microphone(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::MICROPHONE, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, bool decoded=true);
std::unique_ptr<hl2ss::rx_si> rx_si(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::SPATIAL_INPUT);
std::unique_ptr<hl2ss::rx_eet> rx_eet(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::EXTENDED_EYE_TRACKER, uint8_t framerate=30);
std::unique_ptr<hl2ss::rx_extended_audio> rx_extended_audio(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::EXTENDED_AUDIO, uint32_t mixer_mode=hl2ss::mixer_mode::BOTH, float loopback_gain=1.0f, float microphone_gain=1.0f, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, bool decoded=true);
std::unique_ptr<hl2ss::rx_extended_depth> rx_extended_depth(char const* host, uint16_t port, uint64_t media_index=0xFFFFFFFF, uint64_t stride_mask=0x3F, uint64_t chunk=hl2ss::chunk_size::EXTENDED_DEPTH, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile_z=hl2ss::depth_profile::ZDEPTH, bool decoded=true);

#ifdef HL2SS_ENABLE_DP
std::unique_ptr<hl2ss::dp::rx_mrc> rx_dp_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk=dp::chunk_size::MRC, dp::mrc_configuration const* configuration=nullptr, uint8_t decoded_format=hl2ss::pv_decoded_format::BGR);
#endif

//------------------------------------------------------------------------------
// Mode 2
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port);
std::unique_ptr<hl2ss::calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port);
std::unique_ptr<hl2ss::calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port);
std::unique_ptr<hl2ss::calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port);
std::unique_ptr<hl2ss::calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate);
std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_audio(char const* host, uint16_t port, uint8_t profile, uint8_t level);
std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_video(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::ipc_rc> ipc_rc(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_sm> ipc_sm(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_su> ipc_su(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_vi> ipc_vi(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_umq> ipc_umq(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_gmq> ipc_gmq(char const* host, uint16_t port);
}
}

#endif
