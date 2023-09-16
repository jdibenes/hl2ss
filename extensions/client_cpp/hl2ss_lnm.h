
#include "hl2ss.h"

namespace hl2ss
{
namespace lnm
{
//------------------------------------------------------------------------------
// Default Settings
//------------------------------------------------------------------------------

float get_video_codec_default_factor(uint8_t profile);
uint8_t get_video_coded_default_gop_size(uint8_t framerate, uint8_t divisor, uint8_t profile);
uint32_t get_video_codec_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, float factor);
uint32_t get_video_codec_default_bitrate(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile);
std::vector<uint64_t> get_video_codec_default_options(uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile);

//------------------------------------------------------------------------------
// Stream Sync Period
//------------------------------------------------------------------------------

uint64_t get_sync_frame_stamp(uint64_t frame_stamp, uint64_t sync_period);
uint64_t get_sync_period(hl2ss::rx* rx);

//------------------------------------------------------------------------------
// Control
//------------------------------------------------------------------------------

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc=false, bool hologram_composition=true, bool recording_indicator=false, bool video_stabilization=false, bool blank_protected=false, bool show_mesh=false, float global_opacity=0.9f, float output_width=0.0f, float output_height=0.0f, uint32_t video_stabilization_length=0, uint32_t hologram_perspective=hl2ss::hologram_perspective::PV);
void stop_subsystem_pv(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// Modes 0, 1
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::rx_rm_vlc> rx_rm_vlc(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::RM_VLC, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_depth_ahat> rx_rm_depth_ahat(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::RM_DEPTH_AHAT, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile_z=hl2ss::depth_profile::SAME, uint8_t profile_ab=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_depth_longthrow> rx_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::RM_DEPTH_LONGTHROW, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t png_filter=hl2ss::png_filter_mode::PAETH, bool decoded=true);
std::unique_ptr<hl2ss::rx_rm_imu> rx_rm_imu(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::RM_IMU, uint8_t mode=hl2ss::stream_mode::MODE_1);
std::unique_ptr<hl2ss::rx_pv> rx_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, size_t chunk=hl2ss::chunk_size::PERSONAL_VIDEO, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, std::vector<uint64_t> const* options=nullptr, uint8_t decoded_format=hl2ss::pv_decoded_format::BGR);
std::unique_ptr<hl2ss::rx_microphone> rx_microphone(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::MICROPHONE, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, bool decoded=true);
std::unique_ptr<hl2ss::rx_si> rx_si(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::SPATIAL_INPUT);
std::unique_ptr<hl2ss::rx_eet> rx_eet(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::EXTENDED_EYE_TRACKER, uint8_t framerate=hl2ss::eet_framerate::FPS_30);
std::unique_ptr<hl2ss::rx_extended_audio> rx_extended_audio(char const* host, uint16_t port, size_t chunk=hl2ss::chunk_size::EXTENDED_AUDIO, uint32_t mixer_mode=hl2ss::mixer_mode::BOTH, float loopback_gain=1.0f, float microphone_gain=1.0f, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, bool decoded=true);

//------------------------------------------------------------------------------
// Mode 2
//------------------------------------------------------------------------------

std::shared_ptr<hl2ss::calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port);
std::shared_ptr<hl2ss::calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port);
std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port);
std::shared_ptr<hl2ss::calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port);
std::shared_ptr<hl2ss::calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate);

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------

std::unique_ptr<hl2ss::ipc_rc> ipc_rc(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_sm> ipc_sm(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_su> ipc_su(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_vi> ipc_vi(char const* host, uint16_t port);
std::unique_ptr<hl2ss::ipc_umq> ipc_umq(char const* host, uint16_t port);
}
}
