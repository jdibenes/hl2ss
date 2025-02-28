
#include <iostream>
#include <codecvt>
#include <locale>
#include "hl2ss.h"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

#if defined(_WIN32)
#define HL2SS_CLIENT_EXPORT extern "C" __declspec(dllexport)
#define HL2SS_CALL 
#elif defined(__ANDROID__)
#include <jni.h>
#define HL2SS_CLIENT_EXPORT extern "C" JNIEXPORT
#define HL2SS_CALL JNICALL
#else
#define HL2SS_CLIENT_EXPORT extern "C"
#define HL2SS_CALL 
#endif

#define HL2SS_ULM_BEGIN  try
#define HL2SS_ULM_END(v) catch (std::exception const& e) { std::cerr << e.what() << '\n'; return v; } catch (...) { return v; }

class handle
{
protected:
    void* m_p;

    handle(void* p) : m_p(p) 
    {
    }

public:
    virtual ~handle()
    {
    }
};

template<typename T>
class typed_handle : public handle
{
protected:
    std::shared_ptr<T>* get()
    {
        return (std::shared_ptr<T>*)m_p;
    }

    typed_handle(std::shared_ptr<T>* p) : handle(p)
    {
    }

public:
    ~typed_handle()
    {
        delete get();
    }

    static void* create(std::shared_ptr<T>& p)
    {
        std::shared_ptr<T>* q = new (std::nothrow) std::shared_ptr<T>(p);
        if (!q) { return nullptr; }
        typed_handle<T>* h = new (std::nothrow) typed_handle<T>(q);
        if (h) { return h; }
        delete q;
        return nullptr;
    }

    static std::shared_ptr<T>& get(void* p)
    {
        return *((typed_handle<T>*)p)->get();
    }
};

namespace hl2ss
{
namespace ulm
{

//-----------------------------------------------------------------------------
// Adapters
//-----------------------------------------------------------------------------

struct configuration_rm_vlc
{
    uint64_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint32_t bitrate;
    int64_t options_size;
    uint64_t const* options_data;
    void* _reserved;
};

struct configuration_rm_depth_ahat
{
    uint64_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t profile_z;
    uint8_t profile_ab;
    uint8_t level;
    uint8_t _reserved_0[3];
    uint32_t bitrate;
    uint32_t _reserved_1;
    int64_t options_size;
    uint64_t const* options_data;
    void* _reserved_2;
};

struct configuration_rm_depth_longthrow
{
    uint64_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t png_filter;
    uint8_t _reserved[5];
};

struct configuration_rm_imu
{
    uint64_t chunk;
    uint8_t mode;
    uint8_t _reserved[7];
};

struct configuration_pv
{
    uint64_t chunk;
    uint8_t mode;
    uint8_t _reserved_0;
    uint16_t width;
    uint16_t height;
    uint8_t framerate;
    uint8_t _reserved_1;
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint8_t decoded_format;
    uint32_t bitrate;
    int64_t options_size;
    uint64_t const* options_data;
    void* _reserved_2;
};

struct configuration_microphone
{
    uint64_t chunk;
    uint8_t profile;
    uint8_t level;
    uint8_t _reserved[6];
};

struct configuration_si
{
    uint64_t chunk;
};

struct configuration_eet
{
    uint64_t chunk;
    uint8_t framerate;
    uint8_t _reserved[7];
};

struct configuration_extended_audio
{
    uint64_t chunk;
    uint32_t mixer_mode;
    float loopback_gain;
    float microphone_gain;
    uint8_t profile;
    uint8_t level;
    uint8_t _reserved[2];
};

struct configuration_extended_depth
{
    uint64_t chunk;
    uint64_t media_index;
    uint64_t stride_mask;
    uint8_t mode;
    uint8_t divisor;
    uint8_t profile_z;
    uint8_t _reserved[5];
};

struct configuration_pv_subsystem
{
    uint8_t enable_mrc;
    uint8_t hologram_composition;
    uint8_t recording_indicator;
    uint8_t video_stabilization;
    uint8_t blank_protected;
    uint8_t show_mesh;
    uint8_t shared;
    uint8_t _reserved_0;
    float global_opacity;
    float output_width;
    float output_height;
    uint32_t video_stabilization_length;
    uint32_t hologram_perspective;
    uint32_t _reserved_1;
};

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
    void* _reserved;
};

struct su_mesh
{
    uint64_t vertex_positions_size;
    uint64_t triangle_indices_size;
    uint8_t* vertex_positions_data;
    uint8_t* triangle_indices_data;
};

struct su_item
{
    guid id;
    int32_t kind;
    uint32_t _reserved_0;
    quaternion orientation;
    vector_3 position;
    int32_t alignment;
    vector_2 extents;
    uint32_t meshes_count;
    uint32_t collider_meshes_count;
    matrix_4x4* location;
    void* meshes;
    void* collider_meshes;
    void* _reserved_1;
};

struct su_result_header
{
    uint32_t status;
    uint32_t count;
    matrix_4x4* extrinsics;
    matrix_4x4* pose;
};

struct su_task 
{
    bool enable_quads;
    bool enable_meshes;
    bool enable_only_observed;
    bool enable_world_mesh;
    uint32_t mesh_lod;
    float query_radius;
    uint8_t create_mode;
    uint8_t kind_flags;
    bool get_orientation;
    bool get_position;
    bool get_location_matrix;
    bool get_quad;
    bool get_meshes;
    bool get_collider_meshes;
    uint32_t _reserved_0;
    uint64_t guid_list_size;
    guid* guid_list_data;
    void* _reserved_1;
};

struct gmq_message
{
    uint32_t command;
    uint32_t size;
    uint8_t* data;
    void* _reserved;
};

//-----------------------------------------------------------------------------
// Initialize
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL initialize()
HL2SS_ULM_BEGIN
{
    hl2ss::client::initialize();
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Interfaces
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL open_stream(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::rx> rx;

    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
    {
        configuration_rm_vlc const& p = *(configuration_rm_vlc*)configuration;
        bool decoded = true;
        std::vector<uint64_t> options{ p.options_data, p.options_data + ((p.options_size < 0) ? 0 : p.options_size) };
        rx = hl2ss::lnm::rx_rm_vlc(host, port, p.chunk, p.mode, p.divisor, p.profile, p.level, p.bitrate, (p.options_size >= 0) ? &options : nullptr, decoded);
        break;
    }
    case hl2ss::stream_port::RM_DEPTH_AHAT:
    {
        configuration_rm_depth_ahat const& p = *(configuration_rm_depth_ahat*)configuration;
        bool decoded = true;
        std::vector<uint64_t> options{ p.options_data, p.options_data + ((p.options_size < 0) ? 0 : p.options_size) };
        rx = hl2ss::lnm::rx_rm_depth_ahat(host, port, p.chunk, p.mode, p.divisor, p.profile_z, p.profile_ab, p.level, p.bitrate, (p.options_size >= 0) ? &options : nullptr, decoded);
        break;
    }
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:
    {
        configuration_rm_depth_longthrow const& p = *(configuration_rm_depth_longthrow*)configuration;
        bool decoded = true;
        rx = hl2ss::lnm::rx_rm_depth_longthrow(host, port, p.chunk, p.mode, p.divisor, p.png_filter, decoded);
        break;
    }
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:
    {
        configuration_rm_imu const& p = *(configuration_rm_imu*)configuration;
        rx = hl2ss::lnm::rx_rm_imu(host, port, p.chunk, p.mode);
        break;
    }        
    case hl2ss::stream_port::PERSONAL_VIDEO:
    case hl2ss::stream_port::EXTENDED_VIDEO:
    {
        configuration_pv const& p = *(configuration_pv*)configuration;
        uint8_t decoded = p.decoded_format % 5;
        std::vector<uint64_t> options{ p.options_data, p.options_data + ((p.options_size < 0) ? 0 : p.options_size) };
        rx = hl2ss::lnm::rx_pv(host, port, p.width, p.height, p.framerate, p.chunk, p.mode, p.divisor, p.profile, p.level, p.bitrate, (p.options_size >= 0) ? &options : nullptr, decoded);
        break;
    }        
    case hl2ss::stream_port::MICROPHONE:
    {
        configuration_microphone const& p = *(configuration_microphone*)configuration;
        bool decoded = true;
        rx = hl2ss::lnm::rx_microphone(host, port, p.chunk, p.profile, p.level, decoded);
        break;
    }    
    case hl2ss::stream_port::SPATIAL_INPUT:
    {
        configuration_si const& p = *(configuration_si*)configuration;        
        rx = hl2ss::lnm::rx_si(host, port, p.chunk);
        break;
    }
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER:
    {
        configuration_eet const& p = *(configuration_eet*)configuration;
        rx = hl2ss::lnm::rx_eet(host, port, p.chunk, p.framerate);
        break;
    }
    case hl2ss::stream_port::EXTENDED_AUDIO:
    {
        configuration_extended_audio const& p = *(configuration_extended_audio*)configuration;
        bool decoded = true;
        rx = hl2ss::lnm::rx_extended_audio(host, port, p.chunk, p.mixer_mode, p.loopback_gain, p.microphone_gain, p.profile, p.level, decoded);
        break;
    }
    case hl2ss::stream_port::EXTENDED_DEPTH:
    {
        configuration_extended_depth const& p = *(configuration_extended_depth*)configuration;
        bool decoded = true;
        rx = hl2ss::lnm::rx_extended_depth(host, port, p.media_index, p.stride_mask, p.chunk, p.mode, p.divisor, p.profile_z, decoded);
        break;
    }
    default:
        return nullptr;
    }

    std::shared_ptr<hl2ss::mt::source> source = std::make_shared<hl2ss::mt::source>(buffer_size, std::move(rx));
    source->start();
    return typed_handle<hl2ss::mt::source>::create(source);
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
    default:
        return nullptr;
    }

    ipc->open();
    return typed_handle<hl2ss::ipc>::create(ipc);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void HL2SS_CALL close_handle(void* h)
HL2SS_ULM_BEGIN
{
    delete (handle*)h;
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

    return typed_handle<hl2ss::packet>::create(data);
}

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL get_by_index(void* source, int64_t frame_stamp, hl2ss::ulm::packet& packet)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::mt::source> s = typed_handle<hl2ss::mt::source>::get(source);

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
    std::shared_ptr<hl2ss::mt::source> s = typed_handle<hl2ss::mt::source>::get(source);

    std::exception source_error;
    if (!s->status(source_error)) { throw source_error; }

    int64_t frame_stamp;
    int32_t status;
    std::shared_ptr<hl2ss::packet> data = s->get_packet(timestamp, time_preference, tiebreak_right, frame_stamp, status);
    return unpack_frame(data, frame_stamp, status, packet);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL get_pv_dimensions(void* source, uint16_t& width, uint16_t& height)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<hl2ss::mt::source> s = typed_handle<hl2ss::mt::source>::get(source);
    hl2ss::rx const* rx_base = s->get_rx<hl2ss::rx>();
    switch (rx_base->port)
    {
    case hl2ss::stream_port::PERSONAL_VIDEO:
    case hl2ss::stream_port::EXTENDED_VIDEO:
    {
        hl2ss::rx_decoded_pv const* rx_pv = dynamic_cast<hl2ss::rx_decoded_pv const*>(rx_base);
        if (!rx_pv) { break; }
        width = rx_pv->width;
        height = rx_pv->height;
        return 0;
    }
    case hl2ss::stream_port::EXTENDED_DEPTH:
    {
        hl2ss::rx_decoded_extended_depth const* rx_ez = dynamic_cast<hl2ss::rx_decoded_extended_depth const*>(rx_base);
        if (!rx_ez) { break; }
        width = rx_ez->width;
        height = rx_ez->height;
        return 0;
    }
    }

    throw std::runtime_error("hl2ss::ulm::get_pv_dimensions : source is not compatible");
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL start_subsystem_pv(char const* host, uint16_t port, configuration_pv_subsystem const& c)
HL2SS_ULM_BEGIN
{
    hl2ss::lnm::start_subsystem_pv(host, port, c.enable_mrc, c.hologram_composition, c.recording_indicator, c.video_stabilization, c.blank_protected, c.show_mesh, c.shared, c.global_opacity, c.output_width, c.output_height, c.video_stabilization_length, c.hologram_perspective);
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
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
    {
        std::shared_ptr<hl2ss::calibration_rm_vlc> data = hl2ss::lnm::download_calibration_rm_vlc(host, port);
        calibration = data.get();
        return typed_handle<hl2ss::calibration_rm_vlc>::create(data);
    }
    case hl2ss::stream_port::RM_DEPTH_AHAT:
    {
        std::shared_ptr<hl2ss::calibration_rm_depth_ahat> data = hl2ss::lnm::download_calibration_rm_depth_ahat(host, port);
        calibration = data.get();
        return typed_handle<hl2ss::calibration_rm_depth_ahat>::create(data);
    }
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:
    {
        std::shared_ptr<hl2ss::calibration_rm_depth_longthrow> data = hl2ss::lnm::download_calibration_rm_depth_longthrow(host, port);
        calibration = data.get();
        return typed_handle<hl2ss::calibration_rm_depth_longthrow>::create(data);
    }
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:
    {
        std::shared_ptr<hl2ss::calibration_rm_imu> data = hl2ss::lnm::download_calibration_rm_imu(host, port);
        calibration = data.get();
        return typed_handle<hl2ss::calibration_rm_imu>::create(data);
    }
    case hl2ss::stream_port::PERSONAL_VIDEO:
    {
        configuration_pv const& p = *(configuration_pv*)configuration;
        std::shared_ptr<hl2ss::calibration_pv> data = hl2ss::lnm::download_calibration_pv(host, port, p.width, p.height, p.framerate);
        calibration = data.get();
        return typed_handle<hl2ss::calibration_pv>::create(data);
    }
    default:
        return nullptr;
    }
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL download_device_list(char const* host, uint16_t port, uint64_t& size, uint8_t*& query)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<uint8_t>> data;

    switch (port)
    {
    case hl2ss::stream_port::EXTENDED_AUDIO: data = hl2ss::lnm::download_devicelist_extended_audio(host, port); break;
    case hl2ss::stream_port::EXTENDED_VIDEO: data = hl2ss::lnm::download_devicelist_extended_video(host, port); break;
    default: return nullptr;
    }

    size  = data->size();
    query = data->data();

    return typed_handle<std::vector<uint8_t>>::create(data);
}
HL2SS_ULM_END(nullptr)

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_get_application_version(void* ipc, hl2ss::version& version)
HL2SS_ULM_BEGIN
{
    version = typed_handle<hl2ss::ipc_rc>::get(ipc)->get_application_version();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_get_utc_offset(void* ipc, uint64_t& offset)
HL2SS_ULM_BEGIN
{
    offset = typed_handle<hl2ss::ipc_rc>::get(ipc)->get_utc_offset();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_hs_marker_state(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_hs_marker_state(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_get_pv_subsystem_status(void* ipc, uint32_t& status)
HL2SS_ULM_BEGIN
{
    status = typed_handle<hl2ss::ipc_rc>::get(ipc)->get_pv_subsystem_status();
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_wait_for_pv_subsystem(void* ipc, uint32_t status)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->wait_for_pv_subsystem(status);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_focus(mode, range, distance, value, driver_fallback);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_video_temporal_denoising(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_video_temporal_denoising(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_white_balance_preset(void* ipc, uint32_t preset)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_white_balance_preset(preset);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_white_balance_value(void* ipc, uint32_t value)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_white_balance_value(value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_exposure(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_exposure(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_exposure_priority_video(void* ipc, uint32_t enabled)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_exposure_priority_video(enabled);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_iso_speed(void* ipc, uint32_t mode, uint32_t value)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_iso_speed(mode, value);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_backlight_compensation(void* ipc, uint32_t state)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_backlight_compensation(state);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_scene_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_scene_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_flat_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_flat_mode(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_rm_eye_selection(void* ipc, uint32_t enable)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_rm_eye_selection(enable);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_desired_optimization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_desired_optimization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_primary_use(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_primary_use(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_optical_image_stabilization(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_optical_image_stabilization(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_hdr_video(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_hdr_video(mode);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_pv_regions_of_interest(void* ipc, uint32_t clear, uint32_t set, uint32_t auto_exposure, uint32_t auto_focus, uint32_t bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_pv_regions_of_interest(clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_interface_priority(void* ipc, uint16_t port, int32_t priority)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_interface_priority(port, priority);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL rc_set_quiet_mode(void* ipc, uint32_t mode)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_rc>::get(ipc)->set_quiet_mode(mode);
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
    typed_handle<hl2ss::ipc_sm>::get(ipc)->set_volumes(volumes);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL sm_get_observed_surfaces(void* ipc, uint64_t& size, hl2ss::sm_surface_info*& data)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<hl2ss::sm_surface_info>> surfaces = std::make_shared<std::vector<hl2ss::sm_surface_info>>();
    typed_handle<hl2ss::ipc_sm>::get(ipc)->get_observed_surfaces(*surfaces);    
    size = surfaces->size();
    data = surfaces->data();
    return typed_handle<std::vector<hl2ss::sm_surface_info>>::create(surfaces);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL sm_get_meshes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size)
HL2SS_ULM_BEGIN
{
    hl2ss::sm_mesh_task tasks(count, data, size);
    std::shared_ptr<std::vector<hl2ss::sm_mesh>> meshes = std::make_shared<std::vector<hl2ss::sm_mesh>>();
    typed_handle<hl2ss::ipc_sm>::get(ipc)->get_meshes(tasks, *meshes);
    return typed_handle<std::vector<hl2ss::sm_mesh>>::create(meshes);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL sm_unpack_mesh(void* reference, uint32_t index, hl2ss::ulm::sm_mesh& mesh)
HL2SS_ULM_BEGIN
{
    std::vector<hl2ss::sm_mesh>& meshes = *typed_handle<std::vector<hl2ss::sm_mesh>>::get(reference);
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

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL su_query(void* ipc, hl2ss::ulm::su_task const& task, hl2ss::ulm::su_result_header& header)
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
    typed_handle<hl2ss::ipc_su>::get(ipc)->query(t, *result);

    header.status     =            result->status;
    header.extrinsics =           &result->extrinsics;
    header.pose       =           &result->pose;
    header.count      =  (uint32_t)result->items.size();

    return typed_handle<hl2ss::su_result>::create(result);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL su_unpack_item(void* reference, uint32_t index, hl2ss::ulm::su_item& item)
HL2SS_ULM_BEGIN
{
    std::vector<hl2ss::su_item>& items = typed_handle<hl2ss::su_result>::get(reference)->items;
    hl2ss::su_item& m = items[index];

    item.id                    =           m.id;
    item.kind                  =           m.kind;
    item.orientation           =           m.orientation;
    item.position              =           m.position;
    item.location              =          &m.location;
    item.alignment             =           m.alignment;
    item.extents               =           m.extents;
    item.meshes                =          &m.meshes;
    item.meshes_count          = (uint32_t)m.meshes.size();
    item.collider_meshes       =          &m.collider_meshes;
    item.collider_meshes_count = (uint32_t)m.collider_meshes.size();

    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL su_unpack_item_mesh(void* meshes, uint32_t index, hl2ss::ulm::su_mesh& mesh)
HL2SS_ULM_BEGIN
{
    std::vector<hl2ss::su_mesh>& v = *(std::vector<hl2ss::su_mesh>*)meshes;
    hl2ss::su_mesh& m = v[index];

    mesh.vertex_positions_data = m.vertex_positions.data();
    mesh.vertex_positions_size = m.vertex_positions.size();
    mesh.triangle_indices_data = m.triangle_indices.data();
    mesh.triangle_indices_size = m.triangle_indices.size();

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

    typed_handle<hl2ss::ipc_vi>::get(ipc)->start(commands);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL vi_pop(void* ipc, uint64_t& size, hl2ss::vi_result*& data)
HL2SS_ULM_BEGIN
{
    std::shared_ptr<std::vector<hl2ss::vi_result>> results = std::make_shared<std::vector<hl2ss::vi_result>>();
    typed_handle<hl2ss::ipc_vi>::get(ipc)->pop(*results);

    size = results->size();
    data = results->data();

    return typed_handle<std::vector<hl2ss::vi_result>>::create(results);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL vi_stop(void* ipc)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_vi>::get(ipc)->stop();
    return 0;
}
HL2SS_ULM_END(-1)

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL umq_push(void* ipc, uint8_t const* data, uint64_t size)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_umq>::get(ipc)->push(data, size);
    return 0;
}
HL2SS_ULM_END(-1)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL umq_pull(void* ipc, uint32_t* data, uint32_t count)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_umq>::get(ipc)->pull(data, count);
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
    typed_handle<hl2ss::ipc_gmq>::get(ipc)->pull(*message);

    result.command = message->command;
    result.size    = message->size;
    result.data    = message->data.get();

    return typed_handle<hl2ss::gmq_message>::create(message);
}
HL2SS_ULM_END(nullptr)

HL2SS_CLIENT_EXPORT
int32_t HL2SS_CALL gmq_push(void* ipc, uint32_t const* response, uint32_t count)
HL2SS_ULM_BEGIN
{
    typed_handle<hl2ss::ipc_gmq>::get(ipc)->push(response, count);
    return 0;
}
HL2SS_ULM_END(-1)

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------

HL2SS_CLIENT_EXPORT
void* HL2SS_CALL pointer_passthrough(void *p)
{
     return p;
}

}
}
