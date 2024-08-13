
#include <iostream>
#include <codecvt>
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
// Adapters
//-----------------------------------------------------------------------------

struct packet
{
    int64_t frame_stamp;
    uint64_t timestamp;
    uint32_t sz_payload;
    int32_t status;
    uint8_t* payload;
    matrix_4x4* pose;
};

struct sm_mesh
{
    uint32_t status;
    vector_3 vertex_position_scale;
    uint64_t bounds_size;
    uint64_t vertex_positions_size;
    uint64_t triangle_indices_size;
    uint64_t vertex_normals_size;
    matrix_4x4* pose;    
    uint8_t* bounds_data;
    uint8_t* vertex_positions_data;
    uint8_t* triangle_indices_data;
    uint8_t* vertex_normals_data;
};

struct gmq_message
{
    uint32_t command;
    uint32_t size;
    uint8_t* data;
};

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
void* open_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t const* options_data, uint64_t options_size, uint64_t buffer_size)
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
void* open_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, uint64_t const* options_data, uint64_t options_size, uint64_t buffer_size)
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
void* open_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, uint64_t const* options_data, uint64_t options_size, uint8_t decoded_format, uint64_t buffer_size)
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

static void* unpack_frame(std::shared_ptr<hl2ss::packet> data, int64_t frame_stamp, int32_t status, hl2ss::ulm::packet& packet)
{
    if (data)
    {
        packet.frame_stamp = frame_stamp;
        packet.timestamp   = data->timestamp;
        packet.sz_payload  = data->sz_payload;
        packet.status      = status;
        packet.payload     = data->payload.get();        
        packet.pose        = data->pose.get();
    }
    else
    {
        packet.frame_stamp = frame_stamp;
        packet.timestamp   = 0ULL;
        packet.sz_payload  = 0UL;
        packet.status      = status;
        packet.payload     = nullptr;
        packet.pose        = nullptr;
    }

    return new std::shared_ptr<hl2ss::packet>(data); // delete
}

HL2SS_CLIENT_EXPORT
void* get_by_index(void* source, int64_t frame_stamp, hl2ss::ulm::packet& packet)
HL2SS_ULM_BEGIN
{
    hl2ss::mt::source* s = (hl2ss::mt::source*)source;

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    int32_t status;
    std::shared_ptr<hl2ss::packet> data = s->get_packet(frame_stamp, status);
    return unpack_frame(data, frame_stamp, status, packet);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* get_by_timestamp(void* source, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, hl2ss::ulm::packet& packet)
HL2SS_ULM_BEGIN
{
    hl2ss::mt::source* s = (hl2ss::mt::source*)source;

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    int64_t frame_stamp;
    int32_t status;
    std::shared_ptr<hl2ss::packet> data = s->get_packet(timestamp, time_preference, tiebreak_right, frame_stamp, status);
    return unpack_frame(data, frame_stamp, status, packet);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void release_packet(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::packet>*)reference;
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
void* download_calibration_rm_vlc(char const* host, uint16_t port, hl2ss::calibration_rm_vlc*& calibration)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_vlc> data = hl2ss::lnm::download_calibration_rm_vlc(host, port);
    calibration = data.get();
    return new std::shared_ptr<hl2ss::calibration_rm_vlc>(data); // delete
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* download_calibration_rm_depth_ahat(char const* host, uint16_t port, hl2ss::calibration_rm_depth_ahat*& calibration)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);
    calibration = data.get();
    return new std::shared_ptr<hl2ss::calibration_rm_depth_ahat>(data); // delete
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* download_calibration_rm_depth_longthrow(char const* host, uint16_t port, calibration_rm_depth_longthrow*& calibration)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);
    calibration = data.get();
    return new std::shared_ptr<hl2ss::calibration_rm_depth_longthrow>(data); // delete
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* download_calibration_rm_imu(char const* host, uint16_t port, calibration_rm_imu*& calibration)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_rm_imu> data = hl2ss::lnm::download_calibration_rm_imu(host, port);
    calibration = data.get();
    return new std::shared_ptr<hl2ss::calibration_rm_imu>(data); // delete
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, calibration_pv*& calibration)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::calibration_pv> data = hl2ss::lnm::download_calibration_pv(host, port, width, height, framerate);
    calibration = data.get();
    return new std::shared_ptr<hl2ss::calibration_pv>(data); // delete
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void release_calibration_rm_vlc(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::calibration_rm_vlc>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void release_calibration_rm_depth_ahat(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::calibration_rm_depth_ahat>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void release_calibration_rm_depth_longthrow(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::calibration_rm_depth_longthrow>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void release_calibration_rm_imu(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::calibration_rm_imu>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void release_calibration_pv(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::shared_ptr<hl2ss::calibration_pv>*)reference;
}
HL2SS_ULM_END(void())

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t rc_get_application_version(void* ipc, hl2ss::version& version)
HL2SS_ULM_BEGIN
{
    version = ((hl2ss::ipc_rc*)ipc)->get_application_version();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_get_utc_offset(void* ipc, uint32_t samples, uint64_t& offset)
HL2SS_ULM_BEGIN
{
    offset = ((hl2ss::ipc_rc*)ipc)->get_utc_offset(samples);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_hs_marker_state(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_hs_marker_state(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_get_pv_subsystem_status(void* ipc, uint32_t& status)
HL2SS_ULM_BEGIN
{
    status = ((hl2ss::ipc_rc*)ipc)->get_pv_subsystem_status();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_wait_for_pv_subsystem(void* ipc, uint32_t status)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->wait_for_pv_subsystem(status);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_focus(mode, range, distance, value, driver_fallback);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_video_temporal_denoising(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_video_temporal_denoising(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_white_balance_preset(void* ipc, uint32_t preset)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_white_balance_preset(preset);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_white_balance_value(void* ipc, uint32_t value)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_white_balance_value(value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_exposure(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_exposure(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_exposure_priority_video(void* ipc, uint32_t enabled)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_exposure_priority_video(enabled);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_iso_speed(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_iso_speed(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_backlight_compensation(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_backlight_compensation(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_scene_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_scene_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_flat_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_flat_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_rm_eye_selection(void* ipc, uint32_t enable)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_rm_eye_selection(enable);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_desired_optimization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_desired_optimization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_primary_use(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_primary_use(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_optical_image_stabilization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_optical_image_stabilization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_hdr_video(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_hdr_video(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t rc_set_pv_regions_of_interest(void* ipc, uint32_t clear, uint32_t set, uint32_t auto_exposure, uint32_t auto_focus, uint32_t bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_rc*)ipc)->set_pv_regions_of_interest(clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h);
    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t sm_create_observer(void* ipc)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_sm*)ipc)->create_observer();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t sm_set_volumes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_bounding_volume volumes(count, data, size);
    ((hl2ss::ipc_sm*)ipc)->set_volumes(volumes);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* sm_get_observed_surfaces(void* ipc, uint64_t& size, hl2ss::sm_surface_info*& data)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<std::vector<hl2ss::sm_surface_info>> surfaces = std::make_unique<std::vector<hl2ss::sm_surface_info>>();
    ((hl2ss::ipc_sm*)ipc)->get_observed_surfaces(*surfaces);
    
    size = surfaces->size();
    data = surfaces->data();

    return surfaces.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void sm_release_surfaces(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::vector<hl2ss::sm_surface_info>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
void* sm_get_meshes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size, uint32_t threads)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_mesh_task tasks(count, data, size);
    std::unique_ptr<std::vector<hl2ss::sm_mesh>> meshes = std::make_unique<std::vector<hl2ss::sm_mesh>>();
    ((hl2ss::ipc_sm*)ipc)->get_meshes(tasks, threads, *meshes);
    return meshes.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void sm_release_meshes(void* reference)
HL2SS_ULM_BEGIN
{
    delete (std::vector<hl2ss::sm_mesh>*)reference;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
int32_t sm_unpack_mesh(void* reference, uint32_t index, hl2ss::ulm::sm_mesh& mesh)
HL2SS_ULM_BEGIN
{
    std::vector<hl2ss::sm_mesh>& meshes = *(std::vector<hl2ss::sm_mesh>*)reference;
    hl2ss::sm_mesh& m = meshes[index];

    mesh.status                =  m.status;
    mesh.vertex_position_scale =  m.vertex_position_scale;
    mesh.pose                  = &m.pose;
    mesh.bounds_data           =  m.bounds.data();
    mesh.bounds_size           =  m.bounds.size();
    mesh.vertex_positions_data =  m.vertex_positions.data();
    mesh.vertex_positions_size =  m.vertex_positions.size();
    mesh.triangle_indices_data =  m.triangle_indices.data();
    mesh.triangle_indices_size =  m.triangle_indices.size();
    mesh.vertex_normals_data   =  m.vertex_normals.data();
    mesh.vertex_normals_size   =  m.vertex_normals.size();

    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

// TODO:

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t vi_create_recognizer(void* ipc)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_vi*)ipc)->create_recognizer();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t vi_register_commands(void* ipc, uint32_t clear, char const* utf8_array, uint32_t& status)
HL2SS_ULM_BEGIN
{
    char const* current = utf8_array;
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
    std::vector<std::u16string> commands;
    size_t count;
    
    while ((count = strlen(current)) > 0)
    {
        commands.push_back(convert.from_bytes(current));
        current += count + 1;
    }

    status = ((hl2ss::ipc_vi*)ipc)->register_commands(clear, commands);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t vi_start(void* ipc)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_vi*)ipc)->start();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* vi_pop(void* ipc, uint64_t& size, hl2ss::vi_result*& data)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<std::vector<hl2ss::vi_result>> results = std::make_unique<std::vector<hl2ss::vi_result>>();
    ((hl2ss::ipc_vi*)ipc)->pop(*results);

    size = results->size();
    data = results->data();

    return results.release();    
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void vi_release(void* results)
HL2SS_ULM_BEGIN
{
    delete (std::vector<hl2ss::vi_result>*)results;
}
HL2SS_ULM_END(void())

HL2SS_CLIENT_EXPORT
int32_t vi_clear(void* ipc)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_vi*)ipc)->clear();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t vi_stop(void* ipc)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_vi*)ipc)->stop();
    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t umq_push(void* ipc, uint8_t const* data, uint64_t size)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_umq*)ipc)->push(data, size);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t umq_pull(void* ipc, uint32_t* data, uint32_t count)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_umq*)ipc)->pull(data, count);
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* gmq_pull(void *ipc, hl2ss::ulm::gmq_message& result)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::gmq_message> message = std::make_unique<hl2ss::gmq_message>();
    ((hl2ss::ipc_gmq*)ipc)->pull(*message);

    result.command = message->command;
    result.size    = message->size;
    result.data    = message->data.get();

    return message.release();
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t gmq_push(void* ipc, uint32_t const* response, uint32_t count)
HL2SS_ULM_BEGIN
{
    ((hl2ss::ipc_gmq*)ipc)->push(response, count);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void gmq_release(void* message)
HL2SS_ULM_BEGIN
{
    delete (hl2ss::gmq_message*)message;
}
HL2SS_ULM_END(void())

}
}
