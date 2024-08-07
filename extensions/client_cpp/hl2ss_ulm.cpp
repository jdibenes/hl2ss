
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

#ifdef WIN32
#define HL2SS_CLIENT_EXPORT extern "C" __declspec(dllexport)
#else
#define HL2SS_CLIENT_EXPORT extern "C"
#endif

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------

static std::unique_ptr<hl2ss::mt::source> g_source_rm_vlc[4];
static std::unique_ptr<hl2ss::mt::source> g_source_rm_depth_ahat;
static std::unique_ptr<hl2ss::mt::source> g_source_rm_depth_longthrow;
static std::unique_ptr<hl2ss::mt::source> g_source_rm_imu[3];
static std::unique_ptr<hl2ss::mt::source> g_source_pv;
static std::unique_ptr<hl2ss::mt::source> g_source_microphone;
static std::unique_ptr<hl2ss::mt::source> g_source_si;
static std::unique_ptr<hl2ss::mt::source> g_source_eet;
static std::unique_ptr<hl2ss::mt::source> g_source_extended_audio;
static std::unique_ptr<hl2ss::mt::source> g_source_ev;
static std::unique_ptr<hl2ss::ipc_rc>     g_ipc_rc;
static std::unique_ptr<hl2ss::ipc_sm>     g_ipc_sm;
static std::unique_ptr<hl2ss::ipc_su>     g_ipc_su;
static std::unique_ptr<hl2ss::ipc_vi>     g_ipc_vi;
static std::unique_ptr<hl2ss::ipc_umq>    g_ipc_umq;
static std::unique_ptr<hl2ss::ipc_gmq>    g_ipc_gmq;

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

namespace hl2ss
{
namespace ulm
{
//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

static void log_error(uint32_t error_size, char* error_data, char const* message)
{
    if ((error_size <= 0) || !error_data || !message) { return; }
    uint32_t size = (uint32_t)strlen(message) + 1;
    uint32_t n = ((size < error_size) ? size : error_size) - 1;
    memcpy(error_data, message, n);
    error_data[n] = '\0';
}

static hl2ss::mt::source* get_source(uint16_t port)
{
    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:     return g_source_rm_vlc[0].get();          break;
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:      return g_source_rm_vlc[1].get();          break;
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    return g_source_rm_vlc[2].get();          break;
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    return g_source_rm_vlc[3].get();          break;
    case hl2ss::stream_port::RM_DEPTH_AHAT:        return g_source_rm_depth_ahat.get();      break;
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   return g_source_rm_depth_longthrow.get(); break;
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER: return g_source_rm_imu[0].get();          break;
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:     return g_source_rm_imu[1].get();          break;
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  return g_source_rm_imu[2].get();          break;
    case hl2ss::stream_port::PERSONAL_VIDEO:       return g_source_pv.get();                 break;
    case hl2ss::stream_port::MICROPHONE:           return g_source_microphone.get();         break;
    case hl2ss::stream_port::SPATIAL_INPUT:        return g_source_si.get();                 break;
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: return g_source_eet.get();                break;
    case hl2ss::stream_port::EXTENDED_AUDIO:       return g_source_extended_audio.get();     break;
    case hl2ss::stream_port::EXTENDED_VIDEO:       return g_source_ev.get();                 break;
    default:                                       return nullptr;
    }
}

static int validate_source(hl2ss::mt::source* source, uint32_t error_size, char* error_data)
{
    if (!source)
    {
        log_error(error_size, error_data, "Port not open");
        return -1;
    }

    std::exception source_error;

    if (!source->status(source_error))
    {
        log_error(error_size, error_data, source_error.what());
        return -2;
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Initialize
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void initialize()
{
    hl2ss::client::initialize();
}

//-----------------------------------------------------------------------------
// Open
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void open_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint64_t buffer_size)
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    bool decoded = true;

    (g_source_rm_vlc[port - hl2ss::stream_port::RM_VLC_LEFTFRONT] = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded)))->start();
}

HL2SS_CLIENT_EXPORT
void open_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint64_t buffer_size)
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    bool decoded = true;

    (g_source_rm_depth_ahat = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded)))->start();
}

HL2SS_CLIENT_EXPORT
void open_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter, uint64_t buffer_size)
{
    bool decoded = true;

    (g_source_rm_depth_longthrow = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter, decoded)))->start();
}

HL2SS_CLIENT_EXPORT
void open_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint64_t buffer_size)
{
    (g_source_rm_imu[port - hl2ss::stream_port::RM_IMU_ACCELEROMETER] = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_imu(host, port, chunk, mode)))->start();
}

HL2SS_CLIENT_EXPORT
void open_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint8_t decoded_format, uint64_t buffer_size)
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    uint8_t decoded = decoded_format % 5;

    switch (port)
    {
    case hl2ss::stream_port::PERSONAL_VIDEO: (g_source_pv = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_pv(host, port, width, height, framerate, chunk, mode, divisor, profile, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded)))->start(); break;
    case hl2ss::stream_port::EXTENDED_VIDEO: (g_source_ev = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_pv(host, port, width, height, framerate, chunk, mode, divisor, profile, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded)))->start(); break;
    }
}

HL2SS_CLIENT_EXPORT
void open_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level, uint64_t buffer_size)
{
    bool decoded = true;

    (g_source_microphone = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_microphone(host, port, chunk, profile, level, decoded)))->start();
}

HL2SS_CLIENT_EXPORT
void open_si(char const* host, uint16_t port, uint64_t chunk, uint64_t buffer_size)
{
    (g_source_si = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_si(host, port, chunk)))->start();
}

HL2SS_CLIENT_EXPORT
void open_eet(char const* host, uint16_t port, uint64_t chunk, uint8_t framerate, uint64_t buffer_size)
{
    (g_source_eet = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_eet(host, port, chunk, framerate)))->start();
}

HL2SS_CLIENT_EXPORT
void open_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level, uint64_t buffer_size)
{
    bool decoded = true;

    (g_source_extended_audio = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level, decoded)))->start();
}

HL2SS_CLIENT_EXPORT
void open_rc(char const* host, uint16_t port)
{
    (g_ipc_rc = hl2ss::lnm::ipc_rc(host, port))->open();
}

HL2SS_CLIENT_EXPORT
void open_sm(char const* host, uint16_t port)
{
    (g_ipc_sm = hl2ss::lnm::ipc_sm(host, port))->open();
}

HL2SS_CLIENT_EXPORT
void open_su(char const* host, uint16_t port)
{
    (g_ipc_su = hl2ss::lnm::ipc_su(host, port))->open();
}

HL2SS_CLIENT_EXPORT
void open_vi(char const* host, uint16_t port)
{
    (g_ipc_vi = hl2ss::lnm::ipc_vi(host, port))->open();
}

HL2SS_CLIENT_EXPORT
void open_umq(char const* host, uint16_t port)
{
    (g_ipc_umq = hl2ss::lnm::ipc_umq(host, port))->open();
}

HL2SS_CLIENT_EXPORT
void open_gmq(char const* host, uint16_t port)
{
    (g_ipc_gmq = hl2ss::lnm::ipc_gmq(host, port))->open();
}

//-----------------------------------------------------------------------------
// Close
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void close(uint16_t port)
{
    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:     g_source_rm_vlc[0]          = nullptr; break;
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:      g_source_rm_vlc[1]          = nullptr; break;
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    g_source_rm_vlc[2]          = nullptr; break;
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    g_source_rm_vlc[3]          = nullptr; break;
    case hl2ss::stream_port::RM_DEPTH_AHAT:        g_source_rm_depth_ahat      = nullptr; break;
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   g_source_rm_depth_longthrow = nullptr; break;
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER: g_source_rm_imu[0]          = nullptr; break;
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:     g_source_rm_imu[1]          = nullptr; break;
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  g_source_rm_imu[2]          = nullptr; break;
    case hl2ss::stream_port::PERSONAL_VIDEO:       g_source_pv                 = nullptr; break;
    case hl2ss::stream_port::MICROPHONE:           g_source_microphone         = nullptr; break;
    case hl2ss::stream_port::SPATIAL_INPUT:        g_source_si                 = nullptr; break;
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: g_source_eet                = nullptr; break;
    case hl2ss::stream_port::EXTENDED_AUDIO:       g_source_extended_audio     = nullptr; break;
    case hl2ss::stream_port::EXTENDED_VIDEO:       g_source_ev                 = nullptr; break;    
    case hl2ss::ipc_port::REMOTE_CONFIGURATION:    g_ipc_rc                    = nullptr; break;
    case hl2ss::ipc_port::SPATIAL_MAPPING:         g_ipc_sm                    = nullptr; break;
    case hl2ss::ipc_port::SCENE_UNDERSTANDING:     g_ipc_su                    = nullptr; break;
    case hl2ss::ipc_port::VOICE_INPUT:             g_ipc_vi                    = nullptr; break;
    case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:     g_ipc_umq                   = nullptr; break;
    case hl2ss::ipc_port::GUEST_MESSAGE_QUEUE:     g_ipc_gmq                   = nullptr; break;
    }
}

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t get_by_index(uint16_t port, int64_t& frame_stamp, int32_t& status, void*& frame, uint32_t error_size, char* error_data)
{
    hl2ss::mt::source* source = get_source(port);
    int valid = validate_source(source, error_size, error_data);
    if (valid < 0) { return valid; }
    std::shared_ptr<hl2ss::packet> data = source->get_packet(frame_stamp, status);
    if (status == 0) { frame = new (std::nothrow) std::shared_ptr<hl2ss::packet>(data); }
    return valid;
}

HL2SS_CLIENT_EXPORT
int32_t get_by_timestamp(uint16_t port, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, int64_t& frame_stamp, int32_t& status, void*& frame, uint32_t error_size, char* error_data)
{
    hl2ss::mt::source* source = get_source(port);
    int valid = validate_source(source, error_size, error_data);
    if (valid < 0) { return valid; }    
    std::shared_ptr<hl2ss::packet> data = g_source_pv->get_packet(timestamp, time_preference, tiebreak_right, frame_stamp, status);
    if (status == 0) { frame = new (std::nothrow) std::shared_ptr<hl2ss::packet>(data); }
    return valid;
}

HL2SS_CLIENT_EXPORT
void release_frame(void* frame)
{
    if (frame) { delete (std::shared_ptr<hl2ss::packet>*)frame; }
}

HL2SS_CLIENT_EXPORT
int32_t unpack_frame(void* frame, uint64_t& timestamp, uint32_t& payload_size, uint8_t*& payload_data, matrix_4x4*& pose_data)
{
    if (!frame) { return -1; }

    std::shared_ptr<hl2ss::packet> data = *(std::shared_ptr<hl2ss::packet>*)frame;
    if (!data) { return -2; }

    timestamp    = data->timestamp;
    payload_size = data->sz_payload;
    payload_data = data->payload.get();
    pose_data    = data->pose.get();

    return 0;
}

//------------------------------------------------------------------------------
// Unpacking
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void unpack_rm_vlc(uint8_t* payload, uint8_t*& image)
{
    hl2ss::unpack_rm_vlc(payload, &image);
}

HL2SS_CLIENT_EXPORT
void unpack_rm_depth_ahat(uint8_t* payload, uint16_t*& depth, uint16_t*& ab)
{
    hl2ss::unpack_rm_depth_ahat(payload, &depth, &ab);
}

HL2SS_CLIENT_EXPORT
void unpack_rm_depth_longthrow(uint8_t* payload, uint16_t*& depth, uint16_t*& ab)
{
    hl2ss::unpack_rm_depth_longthrow(payload, &depth, &ab);
}

HL2SS_CLIENT_EXPORT
void unpack_rm_imu(uint8_t* payload, rm_imu_sample*& samples)
{
    hl2ss::unpack_rm_imu(payload, &samples);
}

HL2SS_CLIENT_EXPORT
void unpack_pv(uint8_t* payload, uint64_t size, uint8_t*& image, pv_intrinsics*& intrinsics)
{
    hl2ss::unpack_pv(payload, size, &image, &intrinsics);
}

HL2SS_CLIENT_EXPORT
void unpack_microphone_raw(uint8_t* payload, int16_t*& samples)
{
    hl2ss::unpack_microphone_raw(payload, &samples);
}

HL2SS_CLIENT_EXPORT
void unpack_microphone_aac(uint8_t* payload, float*& samples)
{
    hl2ss::unpack_microphone_aac(payload, &samples);
}

HL2SS_CLIENT_EXPORT
void unpack_si(uint8_t* payload, si_frame*& si)
{
    hl2ss::unpack_si(payload, &si);
}

HL2SS_CLIENT_EXPORT
void unpack_eet(uint8_t* payload, eet_frame*& eet)
{
    hl2ss::unpack_eet(payload, &eet);
}

HL2SS_CLIENT_EXPORT
void unpack_extended_audio_raw(uint8_t* payload, int16_t*& samples)
{
    hl2ss::unpack_extended_audio_raw(payload, &samples);
}

HL2SS_CLIENT_EXPORT
void unpack_extended_audio_aac(uint8_t* payload, float*& samples)
{
    hl2ss::unpack_extended_audio_aac(payload, &samples);
}

//------------------------------------------------------------------------------
// Stream Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
uint32_t extended_audio_device_mixer_mode(uint32_t mixer_mode, uint32_t device)
{
    return hl2ss::extended_audio_device_mixer_mode(mixer_mode, device);
}

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t start_subsystem_pv(char const* host, uint16_t port, uint8_t enable_mrc, uint8_t hologram_composition, uint8_t recording_indicator, uint8_t video_stabilization, uint8_t blank_protected, uint8_t show_mesh, uint8_t shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot start subsystem while streaming");
        return -1;
    }

    try
    {
        hl2ss::lnm::start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);
        return 0;
    }
    catch (const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

HL2SS_CLIENT_EXPORT
int32_t stop_subsystem_pv(char const* host, uint16_t port, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot stop subsystem while streaming");
        return -1;
    }

    try
    {
        hl2ss::lnm::stop_subsystem_pv(host, port);
        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_vlc(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* undistort_map, float* intrinsics, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot download calibration while streaming");
        return -1;
    }

    try
    {
        std::shared_ptr<hl2ss::calibration_rm_vlc> calibration = hl2ss::lnm::download_calibration_rm_vlc(host, port);

        memcpy(uv2xy,         calibration->uv2xy,         sizeof(calibration->uv2xy));
        memcpy(extrinsics,    calibration->extrinsics,    sizeof(calibration->extrinsics));
        memcpy(undistort_map, calibration->undistort_map, sizeof(calibration->undistort_map));
        memcpy(intrinsics,    calibration->intrinsics,    sizeof(calibration->intrinsics));

        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_depth_ahat(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* scale, float* alias, float* undistort_map, float* intrinsics, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot download calibration while streaming");
        return -1;
    }

    try
    {
        std::shared_ptr<hl2ss::calibration_rm_depth_ahat> calibration = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);

        memcpy(uv2xy,          calibration->uv2xy,         sizeof(calibration->uv2xy));
        memcpy(extrinsics,     calibration->extrinsics,    sizeof(calibration->extrinsics));
        memcpy(scale,         &calibration->scale,         sizeof(calibration->scale));
        memcpy(alias,         &calibration->alias,         sizeof(calibration->alias));
        memcpy(undistort_map,  calibration->undistort_map, sizeof(calibration->undistort_map));
        memcpy(intrinsics,     calibration->intrinsics,    sizeof(calibration->intrinsics));

        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_depth_longthrow(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* scale, float* undistort_map, float* intrinsics, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot download calibration while streaming");
        return -1;
    }

    try
    {
        std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> calibration = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);

        memcpy(uv2xy,          calibration->uv2xy,         sizeof(calibration->uv2xy));
        memcpy(extrinsics,     calibration->extrinsics,    sizeof(calibration->extrinsics));
        memcpy(scale,         &calibration->scale,         sizeof(calibration->scale));
        memcpy(undistort_map,  calibration->undistort_map, sizeof(calibration->undistort_map));
        memcpy(intrinsics,     calibration->intrinsics,    sizeof(calibration->intrinsics));

        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_imu(char const* host, uint16_t port, float* extrinsics, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot download calibration while streaming");
        return -1;
    }

    try
    {
        std::shared_ptr<hl2ss::calibration_rm_imu> calibration = hl2ss::lnm::download_calibration_rm_imu(host, port);

        memcpy(extrinsics, calibration->extrinsics, sizeof(calibration->extrinsics));

        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

HL2SS_CLIENT_EXPORT
int32_t download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, float* focal_length, float* principal_point, float* radial_distortion, float* tangential_distortion, float* projection, float* extrinsics, uint32_t error_size, char* error_data)
{
    if (get_source(port))
    {
        log_error(error_size, error_data, "Cannot download calibration while streaming");
        return -1;
    }

    try
    {
        std::shared_ptr<hl2ss::calibration_pv> calibration = hl2ss::lnm::download_calibration_pv(host, port, width, height, framerate);

        memcpy(focal_length,          calibration->focal_length,          sizeof(calibration->focal_length));
        memcpy(principal_point,       calibration->principal_point,       sizeof(calibration->principal_point));
        memcpy(radial_distortion,     calibration->radial_distortion,     sizeof(calibration->radial_distortion));
        memcpy(tangential_distortion, calibration->tangential_distortion, sizeof(calibration->tangential_distortion));
        memcpy(projection,            calibration->projection,            sizeof(calibration->projection));
        memcpy(extrinsics,            calibration->extrinsics,            sizeof(calibration->extrinsics));

        return 0;
    }
    catch(const std::exception& e)
    {
        log_error(error_size, error_data, e.what());
        return -2;
    }
}

//-----------------------------------------------------------------------------
// IPC
//-----------------------------------------------------------------------------
}
}
