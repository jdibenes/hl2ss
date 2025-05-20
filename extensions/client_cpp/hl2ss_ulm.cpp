
#include <iostream>
#include <codecvt>
#include <locale>

#define HL2SS_ULM_IMPLEMENTATION

#include "hl2ss_ulm.h"

#define HL2SS_ULM_BEGIN  try
#define HL2SS_ULM_END(v) catch (std::exception const& e) { std::cerr << e.what() << '\n'; return v; } catch (...) { return v; }

namespace hl2ss
{
namespace ulm
{
//-----------------------------------------------------------------------------
// Core
//-----------------------------------------------------------------------------

class handle_base
{
protected:
    handle_base()
    {
    }

public:
    virtual ~handle_base()
    {
    }
};

template<typename T>
class typed_handle : public handle_base
{
public:
    std::shared_ptr<T> reference;

    typed_handle(std::shared_ptr<T> object)
    {
        reference = object;
    }

    ~typed_handle()
    {
    }
};

namespace handle
{
template <typename T>
void* create(std::shared_ptr<T> object)
{
    return new (std::nothrow) typed_handle<T>(object);
}

template <typename T>
std::shared_ptr<T> as(void* h)
{
    return ((typed_handle<T>*)h)->reference;
}

void destroy(void* h)
{
    delete (handle_base*)h;
}
};

static std::unique_ptr<hl2ss::rx> open_stream_rm_vlc(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_rm_vlc* p = (configuration_rm_vlc*)configuration;
    std::vector<uint64_t> options{ p->options_data, p->options_data + ((p->options_size < 0) ? 0 : p->options_size) };
    return hl2ss::lnm::rx_rm_vlc(host, port, p->chunk, p->mode, p->divisor, p->profile, p->level, p->bitrate, (p->options_size >= 0) ? &options : nullptr, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_rm_depth_ahat(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_rm_depth_ahat* p = (configuration_rm_depth_ahat*)configuration;
    std::vector<uint64_t> options{ p->options_data, p->options_data + ((p->options_size < 0) ? 0 : p->options_size) };
    return hl2ss::lnm::rx_rm_depth_ahat(host, port, p->chunk, p->mode, p->divisor, p->profile_z, p->profile_ab, p->level, p->bitrate, (p->options_size >= 0) ? &options : nullptr, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_rm_depth_longthrow(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_rm_depth_longthrow* p = (configuration_rm_depth_longthrow*)configuration;
    return hl2ss::lnm::rx_rm_depth_longthrow(host, port, p->chunk, p->mode, p->divisor, p->png_filter, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_rm_imu(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_rm_imu*p = (configuration_rm_imu*)configuration;
    return hl2ss::lnm::rx_rm_imu(host, port, p->chunk, p->mode, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_pv(char const* host, uint16_t port, void const* configuration, uint8_t decoded)
{
    configuration_pv* p = (configuration_pv*)configuration;
    std::vector<uint64_t> options{ p->options_data, p->options_data + ((p->options_size < 0) ? 0 : p->options_size) };
    return hl2ss::lnm::rx_pv(host, port, p->width, p->height, p->framerate, p->chunk, p->mode, p->divisor, p->profile, p->level, p->bitrate, (p->options_size >= 0) ? &options : nullptr, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_microphone(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_microphone* p = (configuration_microphone*)configuration;
    return hl2ss::lnm::rx_microphone(host, port, p->chunk, p->profile, p->level, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_si(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    (void)decoded;
    configuration_si* p = (configuration_si*)configuration;
    return hl2ss::lnm::rx_si(host, port, p->chunk);
}

static std::unique_ptr<hl2ss::rx> open_stream_eet(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    (void)decoded;
    configuration_eet* p = (configuration_eet*)configuration;
    return hl2ss::lnm::rx_eet(host, port, p->chunk, p->fps);
}

static std::unique_ptr<hl2ss::rx> open_stream_extended_audio(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_extended_audio* p = (configuration_extended_audio*)configuration;
    return hl2ss::lnm::rx_extended_audio(host, port, p->chunk, p->mixer_mode, p->loopback_gain, p->microphone_gain, p->profile, p->level, decoded);
}

static std::unique_ptr<hl2ss::rx> open_stream_extended_depth(char const* host, uint16_t port, void const* configuration, bool decoded)
{
    configuration_extended_depth* p = (configuration_extended_depth*)configuration;
    return hl2ss::lnm::rx_extended_depth(host, port, p->media_index, p->stride_mask, p->chunk, p->mode, p->divisor, p->profile_z, decoded);
}

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

    return handle::create(data);
}

static void* unpack_calibration_rm_vlc(char const* host, uint16_t port, void const* configuration, void*& calibration)
{
    (void)configuration;
    std::shared_ptr<hl2ss::calibration_rm_vlc> data = hl2ss::lnm::download_calibration_rm_vlc(host, port);
    calibration = data.get();
    return handle::create(data);
}

static void* unpack_calibration_rm_depth_ahat(char const* host, uint16_t port, void const* configuration, void*& calibration)
{
    (void)configuration;
    std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);
    calibration = data.get();
    return handle::create(data);
}

static void* unpack_calibration_rm_depth_longthrow(char const* host, uint16_t port, void const* configuration, void*& calibration)
{
    (void)configuration;
    std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);
    calibration = data.get();
    return handle::create(data);
}

static void* unpack_calibration_rm_imu(char const* host, uint16_t port, void const* configuration, void*& calibration)
{
    (void)configuration;
    std::shared_ptr<hl2ss::calibration_rm_imu> data = hl2ss::lnm::download_calibration_rm_imu(host, port);
    calibration = data.get();
    return handle::create(data);
}

static void* unpack_calibration_pv(char const* host, uint16_t port, void const* configuration, void*& calibration)
{
    configuration_pv const* p = (configuration_pv*)configuration;
    std::shared_ptr<hl2ss::calibration_pv> data = hl2ss::lnm::download_calibration_pv(host, port, p->width, p->height, p->framerate);
    calibration = data.get();
    return handle::create(data);
}

static std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_audio(char const* host, uint16_t port, void const* configuration)
{
    configuration_extended_audio const* p = (configuration_extended_audio*)configuration;
    return hl2ss::lnm::download_devicelist_extended_audio(host, port, p->profile, p->level);
}

static std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_video(char const* host, uint16_t port, void const* configuration)
{
    (void)configuration;
    return hl2ss::lnm::download_devicelist_extended_video(host, port);
}

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL initialize()
HL2SS_ULM_BEGIN
{
    hl2ss::client::initialize();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL cleanup()
HL2SS_ULM_BEGIN
{
    hl2ss::client::cleanup();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void HL2SS_CALL close_handle(void* h)
HL2SS_ULM_BEGIN
{
    handle::destroy(h);
}
HL2SS_ULM_END(void())

//-----------------------------------------------------------------------------
// Interfaces
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL open_stream(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration, uint8_t decoded)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::rx> rx;

    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    rx = open_stream_rm_vlc(host, port, configuration, decoded);             break;
    case hl2ss::stream_port::RM_DEPTH_AHAT:        rx = open_stream_rm_depth_ahat(host, port, configuration, decoded);      break;
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   rx = open_stream_rm_depth_longthrow(host, port, configuration, decoded); break;
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  rx = open_stream_rm_imu(host, port, configuration, decoded);             break;
    case hl2ss::stream_port::PERSONAL_VIDEO:
    case hl2ss::stream_port::EXTENDED_VIDEO:       rx = open_stream_pv(host, port, configuration, decoded);                 break;
    case hl2ss::stream_port::MICROPHONE:           rx = open_stream_microphone(host, port, configuration, decoded);         break;
    case hl2ss::stream_port::SPATIAL_INPUT:        rx = open_stream_si(host, port, configuration, decoded);                 break;
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: rx = open_stream_eet(host, port, configuration, decoded);                break;
    case hl2ss::stream_port::EXTENDED_AUDIO:       rx = open_stream_extended_audio(host, port, configuration, decoded);     break;
    case hl2ss::stream_port::EXTENDED_DEPTH:       rx = open_stream_extended_depth(host, port, configuration, decoded);     break;
    default:                                       return nullptr;
    }

    std::shared_ptr<hl2ss::mt::source> source = std::make_shared<hl2ss::mt::source>(buffer_size, std::move(rx));
    
    void* h = handle::create(source);
    if (h) { source->start(); }
    return h;
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL open_ipc(char const* host, uint16_t port)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::ipc> ipc;

    switch (port)
    {
    case hl2ss::ipc_port::REMOTE_CONFIGURATION: ipc = hl2ss::lnm::ipc_rc (host, port); break;
    case hl2ss::ipc_port::SPATIAL_MAPPING:      ipc = hl2ss::lnm::ipc_sm (host, port); break;
    case hl2ss::ipc_port::SCENE_UNDERSTANDING:  ipc = hl2ss::lnm::ipc_su (host, port); break;
    case hl2ss::ipc_port::VOICE_INPUT:          ipc = hl2ss::lnm::ipc_vi (host, port); break;
    case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:  ipc = hl2ss::lnm::ipc_umq(host, port); break;
    case hl2ss::ipc_port::GUEST_MESSAGE_QUEUE:  ipc = hl2ss::lnm::ipc_gmq(host, port); break;
    default:                                    return nullptr;
    }

    void* h = handle::create(ipc);
    if (h) { ipc->open(); }
    return h;
}
HL2SS_ULM_END(nullptr)

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL get_by_index(void* source, int64_t frame_stamp, hl2ss::ulm::packet& packet)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::mt::source> s = handle::as<hl2ss::mt::source>(source);

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    int32_t status;
    std::shared_ptr<hl2ss::packet> data = s->get_packet(frame_stamp, status);
    return unpack_frame(data, frame_stamp, status, packet);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL get_by_timestamp(void* source, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, hl2ss::ulm::packet& packet)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::mt::source> s = handle::as<hl2ss::mt::source>(source);

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    int64_t frame_stamp;
    int32_t status;
    std::shared_ptr<hl2ss::packet> data = s->get_packet(timestamp, time_preference, tiebreak_right, frame_stamp, status);
    return unpack_frame(data, frame_stamp, status, packet);
}
HL2SS_ULM_END(nullptr)

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL start_subsystem_pv(char const* host, uint16_t port, void const* configuration)
HL2SS_ULM_BEGIN
{
    configuration_pv_subsystem* p = (configuration_pv_subsystem*)configuration;
    hl2ss::lnm::start_subsystem_pv(host, port, p->enable_mrc, p->hologram_composition, p->recording_indicator, p->video_stabilization, p->blank_protected, p->show_mesh, p->shared, p->global_opacity, p->output_width, p->output_height, p->video_stabilization_length, p->hologram_perspective);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL stop_subsystem_pv(char const* host, uint16_t port)
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
void* HL2SS_CALL download_calibration(char const* host, uint16_t port, void const* configuration, void*& calibration)
HL2SS_ULM_BEGIN
{
    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    return unpack_calibration_rm_vlc(host, port, configuration, calibration);
    case hl2ss::stream_port::RM_DEPTH_AHAT:        return unpack_calibration_rm_depth_ahat(host, port, configuration, calibration);
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   return unpack_calibration_rm_depth_longthrow(host, port, configuration, calibration);
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:     return unpack_calibration_rm_imu(host, port, configuration, calibration);
    case hl2ss::stream_port::PERSONAL_VIDEO:       return unpack_calibration_pv(host, port, configuration, calibration);
    default:                                       return nullptr;
    }
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL download_device_list(char const* host, uint16_t port, void const* configuration, uint64_t& size, uint8_t*& query)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<uint8_t>> data;

    switch (port)
    {
    case hl2ss::stream_port::EXTENDED_AUDIO: data = download_devicelist_extended_audio(host, port, configuration); break;
    case hl2ss::stream_port::EXTENDED_VIDEO: data = download_devicelist_extended_video(host, port, configuration); break;
    default:                                 return nullptr;
    }

    size  = data->size();
    query = data->data();

    return handle::create(data);
}
HL2SS_ULM_END(nullptr)

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_get_application_version(void* ipc, hl2ss::version& version)
HL2SS_ULM_BEGIN
{
    version = handle::as<hl2ss::ipc_rc>(ipc)->ee_get_application_version();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ts_get_utc_offset(void* ipc, uint64_t& offset)
HL2SS_ULM_BEGIN
{
    offset = handle::as<hl2ss::ipc_rc>(ipc)->ts_get_utc_offset();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_hs_set_marker_state(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->hs_set_marker_state(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_get_subsystem_status(void* ipc, uint32_t& status)
HL2SS_ULM_BEGIN
{
    status = handle::as<hl2ss::ipc_rc>(ipc)->pv_get_subsystem_status();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_wait_for_subsystem(void* ipc, uint32_t status)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_wait_for_subsystem(status);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_focus(mode, range, distance, value, driver_fallback);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_video_temporal_denoising(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_video_temporal_denoising(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_white_balance_preset(void* ipc, uint32_t preset)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_white_balance_preset(preset);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_white_balance_value(void* ipc, uint32_t value)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_white_balance_value(value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_exposure(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_exposure(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_exposure_priority_video(void* ipc, uint32_t enabled)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_exposure_priority_video(enabled);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_iso_speed(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_iso_speed(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_backlight_compensation(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_backlight_compensation(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_scene_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_scene_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_set_flat_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->ee_set_flat_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_rm_set_eye_selection(void* ipc, uint32_t enable)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->rm_set_eye_selection(enable);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_desired_optimization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_desired_optimization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_primary_use(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_primary_use(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_optical_image_stabilization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_optical_image_stabilization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_hdr_video(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_hdr_video(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_pv_set_regions_of_interest(void* ipc, uint32_t clear, uint32_t set, uint32_t auto_exposure, uint32_t auto_focus, uint32_t bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->pv_set_regions_of_interest(clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_set_interface_priority(void* ipc, uint16_t port, int32_t priority)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->ee_set_interface_priority(port, priority);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_set_quiet_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->ee_set_quiet_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL rc_rm_map_camera_points(void* ipc, uint16_t port, uint32_t operation, hl2ss::vector_2 const* points, uint32_t count, hl2ss::vector_2*& out)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::vector_2[]> p = handle::as<hl2ss::ipc_rc>(ipc)->rm_map_camera_points(port, operation, points, count);
    out = p.get();
    return handle::create(p);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL rc_rm_get_rignode_world_poses(void* ipc, uint64_t const* timestamps, uint32_t count, hl2ss::matrix_4x4*& out)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::matrix_4x4[]> p = handle::as<hl2ss::ipc_rc>(ipc)->rm_get_rignode_world_poses(timestamps, count);
    out = p.get();
    return handle::create(p);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ts_get_current_time(void* ipc, uint32_t source, uint64_t& timestamp)
HL2SS_ULM_BEGIN
{
    timestamp = handle::as<hl2ss::ipc_rc>(ipc)->ts_get_current_time(source);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_si_set_sampling_delay(void* ipc, int64_t delay)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->si_set_sampling_delay(delay);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_set_encoder_buffering(void* ipc, uint32_t enable)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->ee_set_encoder_buffering(enable);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_ee_set_reader_buffering(void* ipc, uint32_t enable)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->ee_set_reader_buffering(enable);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_rm_set_loop_control(void* ipc, uint16_t port, uint32_t enable)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_rc>(ipc)->rm_set_loop_control(port, enable);
    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL sm_set_volumes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_bounding_volume volumes(count, data, size);
    handle::as<hl2ss::ipc_sm>(ipc)->set_volumes(volumes);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL sm_get_observed_surfaces(void* ipc, uint64_t& size, hl2ss::sm_surface_info*& data)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<hl2ss::sm_surface_info>> surfaces = std::make_shared<std::vector<hl2ss::sm_surface_info>>();
    handle::as<hl2ss::ipc_sm>(ipc)->get_observed_surfaces(*surfaces);    
    size = surfaces->size();
    data = surfaces->data();
    return handle::create(surfaces);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL sm_get_meshes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size, void*& meshes_data)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_mesh_task tasks(count, data, size);
    std::shared_ptr<std::vector<hl2ss::sm_mesh>> meshes = std::make_shared<std::vector<hl2ss::sm_mesh>>();
    handle::as<hl2ss::ipc_sm>(ipc)->get_meshes(tasks, *meshes);
    meshes_data = meshes->data();
    return handle::create(meshes);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL sm_unpack_mesh(void* meshes_data, uint64_t index, hl2ss::ulm::sm_mesh& mesh)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_mesh* m = ((hl2ss::sm_mesh*)meshes_data) + index;

    mesh.status                = m->status;
    mesh.vertex_position_scale = m->vertex_position_scale;
    mesh.pose                  = m->pose;
    mesh.bounds_data           = m->bounds.data();
    mesh.bounds_size           = m->bounds.size();
    mesh.vertex_positions_data = m->vertex_positions.data();
    mesh.vertex_positions_size = m->vertex_positions.size();
    mesh.triangle_indices_data = m->triangle_indices.data();
    mesh.triangle_indices_size = m->triangle_indices.size();
    mesh.vertex_normals_data   = m->vertex_normals.data();
    mesh.vertex_normals_size   = m->vertex_normals.size();

    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL su_query(void* ipc, hl2ss::ulm::su_task const& task, hl2ss::ulm::su_result& header)
HL2SS_ULM_BEGIN
{
    hl2ss::su_task t;

    t.enable_quads         = task.enable_quads;
    t.enable_meshes        = task.enable_meshes;
    t.enable_only_observed = task.enable_only_observed;
    t.enable_world_mesh    = task.enable_world_mesh;
    t.mesh_lod             = task.mesh_lod;
    t.query_radius         = task.query_radius;
    t.create_mode          = task.create_mode;
    t.kind_flags           = task.kind_flags;
    t.get_orientation      = task.get_orientation;
    t.get_position         = task.get_position;
    t.get_location_matrix  = task.get_location_matrix;
    t.get_quad             = task.get_quad;
    t.get_meshes           = task.get_meshes;
    t.get_collider_meshes  = task.get_collider_meshes;
    t.guid_list            = { task.guid_list_data, task.guid_list_data + task.guid_list_size };

    std::shared_ptr<hl2ss::su_result> result = std::make_shared<hl2ss::su_result>();

    handle::as<hl2ss::ipc_su>(ipc)->query(t, *result);

    header.status      = result->status;
    header.extrinsics  = result->extrinsics;
    header.pose        = result->pose;
    header.items_count = result->items.size();
    header.items_data  = result->items.data();

    return handle::create(result);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL su_unpack_item(void* items_data, uint64_t index, hl2ss::ulm::su_item& item)
HL2SS_ULM_BEGIN
{
    hl2ss::su_item* m = ((hl2ss::su_item*)items_data) + index;

    item.id                    = m->id;
    item.kind                  = m->kind;
    item.orientation           = m->orientation;
    item.position              = m->position;
    item.location              = m->location;
    item.alignment             = m->alignment;
    item.extents               = m->extents;
    item.meshes_data           = m->meshes.data();
    item.meshes_count          = m->meshes.size();
    item.collider_meshes_data  = m->collider_meshes.data();
    item.collider_meshes_count = m->collider_meshes.size();

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL su_unpack_item_mesh(void* meshes_data, uint64_t index, hl2ss::ulm::su_mesh& mesh)
HL2SS_ULM_BEGIN
{
    hl2ss::su_mesh* m = ((hl2ss::su_mesh*)meshes_data) + index;

    mesh.vertex_positions_data = m->vertex_positions.data();
    mesh.vertex_positions_size = m->vertex_positions.size();
    mesh.triangle_indices_data = m->triangle_indices.data();
    mesh.triangle_indices_size = m->triangle_indices.size();

    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL vi_start(void* ipc, char const* utf8_array)
HL2SS_ULM_BEGIN
{
    char const* current = utf8_array;
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
    std::vector<std::u16string> commands;
    uint64_t count;
    
    while ((count = strlen(current)) > 0)
    {
    commands.push_back(convert.from_bytes(current));
    current += count + 1;
    }

    handle::as<hl2ss::ipc_vi>(ipc)->start(commands);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL vi_pop(void* ipc, uint64_t& size, hl2ss::vi_result*& data)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<hl2ss::vi_result>> results = std::make_shared<std::vector<hl2ss::vi_result>>();
    
    handle::as<hl2ss::ipc_vi>(ipc)->pop(*results);

    size = results->size();
    data = results->data();

    return handle::create(results);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL vi_stop(void* ipc)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_vi>(ipc)->stop();
    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL umq_push(void* ipc, uint8_t const* data, uint32_t size)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_umq>(ipc)->push(data, size);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL umq_pull(void* ipc, uint32_t* data, uint32_t count)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_umq>(ipc)->pull(data, count);
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL gmq_pull(void *ipc, hl2ss::ulm::gmq_message& result)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::gmq_message> message = std::make_shared<hl2ss::gmq_message>();

    handle::as<hl2ss::ipc_gmq>(ipc)->pull(*message);

    result.command = message->command;
    result.size    = message->size;
    result.data    = message->data.get();

    return handle::create(message);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL gmq_push(void* ipc, uint32_t const* response, uint32_t count)
HL2SS_ULM_BEGIN
{
    handle::as<hl2ss::ipc_gmq>(ipc)->push(response, count);
    return 0;
}
HL2SS_ULM_END(-1)
}

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL pointer_passthrough(void *p)
{
     return p;
}
}
