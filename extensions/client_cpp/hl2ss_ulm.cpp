
#include <iostream>
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

#ifdef WIN32
#define HL2SS_CLIENT_EXPORT extern "C" __declspec(dllexport)
#else
#define HL2SS_CLIENT_EXPORT extern "C"
#endif

#define HL2SS_ULM_BEGIN  try
#define HL2SS_ULM_END(v) catch (std::exception const& e) { std::cerr << e.what() << '\n'; return v; } catch (...) { return v; }

namespace hl2ss
{
namespace ulm
{
//-----------------------------------------------------------------------------
// Initialize
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int initialize()
HL2SS_ULM_BEGIN
{
    hl2ss::client::initialize();
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Open
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* open_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    bool decoded = true;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    bool decoded = true;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded));    
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    bool decoded = true;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter, decoded));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_rm_imu(host, port, chunk, mode));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t options_size, uint64_t const* options_data, uint8_t decoded_format, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::vector<uint64_t> options{options_data, options_data + options_size};
    uint8_t decoded = decoded_format % 5;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_pv(host, port, width, height, framerate, chunk, mode, divisor, profile, level, bitrate, (options.size() > 0) ? &options : nullptr, decoded));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    bool decoded = true;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_microphone(host, port, chunk, profile, level, decoded));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_si(char const* host, uint16_t port, uint64_t chunk, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_si(host, port, chunk));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_eet(char const* host, uint16_t port, uint64_t chunk, uint8_t framerate, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_eet(host, port, chunk, framerate));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level, uint64_t buffer_size)
HL2SS_ULM_BEGIN
{
    bool decoded = true;

    std::unique_ptr<hl2ss::mt::source> source = std::make_unique<hl2ss::mt::source>(buffer_size, hl2ss::lnm::rx_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level, decoded));
    source->start();
    return source.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_rc(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_rc(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_sm(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_sm(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_su(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_su(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_vi(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_vi(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_umq(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_umq(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* open_gmq(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::ipc> ipc = hl2ss::lnm::ipc_gmq(host, port);
    ipc->open();
    return ipc.release();
}
HL2SS_ULM_END(nullptr)

//-----------------------------------------------------------------------------
// Close
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void close_source(void* source)
HL2SS_ULM_BEGIN
{
    delete (hl2ss::mt::source*)source;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void close_ipc(void* ipc)
HL2SS_ULM_BEGIN
{
    delete (hl2ss::ipc*)ipc;
}
HL2SS_ULM_END(void())

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

static void unpack_frame(std::shared_ptr<hl2ss::packet> data, void*& frame, uint64_t& timestamp, uint32_t& payload_size, uint8_t*& payload, matrix_4x4*& pose)
{
    if (data)
    {
        frame        = new std::shared_ptr<hl2ss::packet>(data);
        timestamp    = data->timestamp;
        payload_size = data->sz_payload;
        payload      = data->payload.get();
        pose         = data->pose.get();
    }
    else
    {
        frame        = nullptr;
        timestamp    = 0ULL;
        payload_size = 0UL;
        payload      = nullptr;
        pose         = nullptr;
    }
}

HL2SS_CLIENT_EXPORT
int32_t get_by_index(void* source, int64_t& frame_stamp, int32_t& status, void*& frame, uint64_t& frame_timestamp, uint32_t& frame_payload_size, uint8_t*& frame_payload, matrix_4x4*& frame_pose)
HL2SS_ULM_BEGIN
{
    hl2ss::mt::source* s = (hl2ss::mt::source*)source;

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    std::shared_ptr<hl2ss::packet> data = s->get_packet(frame_stamp, status);
    if (status == 0) { unpack_frame(data, frame, frame_timestamp, frame_payload_size, frame_payload, frame_pose); }

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t get_by_timestamp(void* source, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, int64_t& frame_stamp, int32_t& status, void*& frame, uint64_t& frame_timestamp, uint32_t& frame_payload_size, uint8_t*& frame_payload, matrix_4x4*& frame_pose)
HL2SS_ULM_BEGIN
{
    hl2ss::mt::source* s = (hl2ss::mt::source*)source;

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    std::shared_ptr<hl2ss::packet> data = s->get_packet(timestamp, time_preference, tiebreak_right, frame_stamp, status);
    if (status == 0) { unpack_frame(data, frame, frame_timestamp, frame_payload_size, frame_payload, frame_pose); }

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void release_frame(void* frame)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::packet>*)frame;
}
HL2SS_ULM_END(void())

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t start_subsystem_pv(char const* host, uint16_t port, uint8_t enable_mrc, uint8_t hologram_composition, uint8_t recording_indicator, uint8_t video_stabilization, uint8_t blank_protected, uint8_t show_mesh, uint8_t shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective)
HL2SS_ULM_BEGIN
{
    hl2ss::lnm::start_subsystem_pv(host, port, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t stop_subsystem_pv(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    hl2ss::lnm::stop_subsystem_pv(host, port);
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_vlc(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* undistort_map, float* intrinsics)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_vlc> calibration = hl2ss::lnm::download_calibration_rm_vlc(host, port);

    memcpy(uv2xy,         calibration->uv2xy,         sizeof(calibration->uv2xy));
    memcpy(extrinsics,    calibration->extrinsics,    sizeof(calibration->extrinsics));
    memcpy(undistort_map, calibration->undistort_map, sizeof(calibration->undistort_map));
    memcpy(intrinsics,    calibration->intrinsics,    sizeof(calibration->intrinsics));

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_depth_ahat(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* scale, float* alias, float* undistort_map, float* intrinsics)
HL2SS_ULM_BEGIN
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
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_depth_longthrow(char const* host, uint16_t port, float* uv2xy, float* extrinsics, float* scale, float* undistort_map, float* intrinsics)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> calibration = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);

    memcpy(uv2xy,          calibration->uv2xy,         sizeof(calibration->uv2xy));
    memcpy(extrinsics,     calibration->extrinsics,    sizeof(calibration->extrinsics));
    memcpy(scale,         &calibration->scale,         sizeof(calibration->scale));
    memcpy(undistort_map,  calibration->undistort_map, sizeof(calibration->undistort_map));
    memcpy(intrinsics,     calibration->intrinsics,    sizeof(calibration->intrinsics));

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t download_calibration_rm_imu(char const* host, uint16_t port, float* extrinsics)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_imu> calibration = hl2ss::lnm::download_calibration_rm_imu(host, port);

    memcpy(extrinsics, calibration->extrinsics, sizeof(calibration->extrinsics));

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, float* focal_length, float* principal_point, float* radial_distortion, float* tangential_distortion, float* projection, float* extrinsics)
HL2SS_ULM_BEGIN
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
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// IPC
//-----------------------------------------------------------------------------
// TODO: IPC Interfaces
}
}
