////////////////////////////////////////////////////////////////////////////////
// HL2SS Unified Library Methods Module
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <vector>
#include <exception>
#include <stdexcept>
#include <memory>

#ifndef HL2SS_ULM_IMPLEMENTATION
#define HL2SS_SHARED
#define HL2SS_LNM_SHARED
#define HL2SS_MT_SHARED
#endif

#include "hl2ss.h"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

//******************************************************************************
// Types
//******************************************************************************

namespace hl2ss
{
namespace ulm
{
//-----------------------------------------------------------------------------
// Adapters
//-----------------------------------------------------------------------------

struct configuration_rm_vlc
{
    uint64_t chunk = hl2ss::chunk_size::RM_VLC;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t divisor = 1;
    uint8_t profile = hl2ss::video_profile::H265_MAIN;
    uint8_t level = hl2ss::h26x_level::DEFAULT;
    uint32_t bitrate = 0;
    int64_t options_size = -1;
    uint64_t const* options_data = nullptr;
    void* _reserved; //
};

struct configuration_rm_depth_ahat
{
    uint64_t chunk = hl2ss::chunk_size::RM_DEPTH_AHAT;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t divisor = 1;
    uint8_t profile_z = hl2ss::depth_profile::SAME;
    uint8_t profile_ab = hl2ss::video_profile::H265_MAIN;
    uint8_t level = hl2ss::h26x_level::DEFAULT;
    uint8_t _reserved_0[3]; //
    uint32_t bitrate = 0;
    uint32_t _reserved_1; //
    int64_t options_size = -1;
    uint64_t const* options_data = nullptr;
    void* _reserved_2; //
};

struct configuration_rm_depth_longthrow
{
    uint64_t chunk = hl2ss::chunk_size::RM_DEPTH_LONGTHROW;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t divisor = 1;
    uint8_t png_filter = hl2ss::png_filter_mode::PAETH;
    uint8_t _reserved[5]; //
};

struct configuration_rm_imu
{
    uint64_t chunk = hl2ss::chunk_size::RM_IMU;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t _reserved[7]; //
};

struct configuration_pv
{
    uint64_t chunk = hl2ss::chunk_size::PERSONAL_VIDEO;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t _reserved_0; //
    uint16_t width = 1920;
    uint16_t height = 1080;
    uint8_t framerate = 30;
    uint8_t divisor = 1;
    uint8_t profile = hl2ss::video_profile::H265_MAIN;
    uint8_t level = hl2ss::h26x_level::DEFAULT;
    uint16_t _reserved_1; //
    uint32_t bitrate = 0;
    int64_t options_size = -1;
    uint64_t const* options_data = nullptr;
    void* _reserved_2; //
};

struct configuration_microphone
{
    uint64_t chunk = hl2ss::chunk_size::MICROPHONE;
    uint8_t profile = hl2ss::audio_profile::AAC_24000;
    uint8_t level = hl2ss::aac_level::L2;
    uint8_t _reserved[6]; //
};

struct configuration_si
{
    uint64_t chunk = hl2ss::chunk_size::SPATIAL_INPUT;
};

struct configuration_eet
{
    uint64_t chunk = hl2ss::chunk_size::EXTENDED_EYE_TRACKER;
    uint8_t fps = 30;
    uint8_t _reserved[7]; //
};

struct configuration_extended_audio
{
    uint64_t chunk = hl2ss::chunk_size::EXTENDED_AUDIO;
    uint32_t mixer_mode = hl2ss::mixer_mode::BOTH;
    float loopback_gain = 1.0f;
    float microphone_gain = 1.0f;
    uint8_t profile = hl2ss::audio_profile::AAC_24000;
    uint8_t level = hl2ss::aac_level::L2;
    uint8_t _reserved[2]; //
};

struct configuration_extended_depth
{
    uint64_t chunk = hl2ss::chunk_size::EXTENDED_DEPTH;
    uint64_t media_index = 0xFFFFFFFF;
    uint64_t stride_mask = 0x3F;
    uint8_t mode = hl2ss::stream_mode::MODE_1;
    uint8_t divisor = 1;
    uint8_t profile_z = hl2ss::depth_profile::ZDEPTH;
    uint8_t _reserved[5]; //
};

struct configuration_pv_subsystem
{
    uint8_t enable_mrc = false;
    uint8_t hologram_composition = true;
    uint8_t recording_indicator = false;
    uint8_t video_stabilization = false;
    uint8_t blank_protected = false;
    uint8_t show_mesh = false;
    uint8_t shared = false;
    uint8_t _reserved_0; //
    float global_opacity = 0.9f;
    float output_width = 0.0f;
    float output_height = 0.0f;
    uint32_t video_stabilization_length = 0;
    uint32_t hologram_perspective = hl2ss::hologram_perspective::PV;
    uint32_t _reserved_1; //
};

struct packet
{
    int64_t frame_stamp;
    uint64_t timestamp;
    uint32_t sz_payload;
    int32_t status;
    uint8_t* payload;
    hl2ss::matrix_4x4* pose;
};

struct sm_mesh
{
    uint32_t status;
    hl2ss::vector_3 vertex_position_scale;
    uint64_t bounds_size;
    uint64_t vertex_positions_size;
    uint64_t triangle_indices_size;
    uint64_t vertex_normals_size;
    hl2ss::matrix_4x4 pose;
    void* bounds_data;
    void* vertex_positions_data;
    void* triangle_indices_data;
    void* vertex_normals_data;
    void* _reserved;
};

struct su_mesh
{
    uint64_t vertex_positions_size;
    uint64_t triangle_indices_size;
    void* vertex_positions_data;
    void* triangle_indices_data;
};

struct su_item
{
    hl2ss::guid id;
    int32_t kind;
    uint32_t _reserved_0;
    hl2ss::quaternion orientation;
    hl2ss::vector_3 position;
    int32_t alignment;
    hl2ss::vector_2 extents;
    uint64_t meshes_count;
    uint64_t collider_meshes_count;
    hl2ss::matrix_4x4 location;
    void* meshes_data;
    void* collider_meshes_data;
    void* _reserved_1;
};

struct su_result
{
    uint32_t status;
    uint32_t _reserved;
    uint64_t items_count;
    hl2ss::matrix_4x4 extrinsics;
    hl2ss::matrix_4x4 pose;
    void* items_data;
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
    hl2ss::guid const* guid_list_data;
    void* _reserved_1;
};

struct gmq_message
{
    uint32_t command;
    uint32_t size;
    uint8_t* data;
    void* _reserved;
};
}
}

#ifndef HL2SS_ULM_IMPLEMENTATION

#ifdef _WIN32
#define HL2SS_CLIENT_IMPORT extern "C" __declspec(dllimport)
#define HL2SS_CALL 
#else
#define HL2SS_CLIENT_IMPORT extern "C"
#define HL2SS_CALL
#endif

#else

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

#endif

#ifndef HL2SS_ULM_IMPLEMENTATION

//******************************************************************************
// Imports
//******************************************************************************

namespace hl2ss
{
namespace ulm
{
//-----------------------------------------------------------------------------
// Core
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL initialize();

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL cleanup();

HL2SS_CLIENT_IMPORT
void HL2SS_CALL close_handle(void* h);

//-----------------------------------------------------------------------------
// Interfaces
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL open_stream(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration, uint8_t decoded);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL open_ipc(char const* host, uint16_t port);

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL get_by_index(void* source, int64_t frame_stamp, hl2ss::ulm::packet& packet);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL get_by_timestamp(void* source, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, hl2ss::ulm::packet& packet);

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL start_subsystem_pv(char const* host, uint16_t port, void const* configuration);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL stop_subsystem_pv(char const* host, uint16_t port);

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL download_calibration(char const* host, uint16_t port, void const* configuration, void*& calibration);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL download_device_list(char const* host, uint16_t port, void const* configuration, uint64_t& size, uint8_t*& query);

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_get_application_version(void* ipc, hl2ss::version& version);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ts_get_utc_offset(void* ipc, uint64_t& offset);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_hs_set_marker_state(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_get_subsystem_status(void* ipc, uint32_t& status);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_wait_for_subsystem(void* ipc, uint32_t status);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_video_temporal_denoising(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_white_balance_preset(void* ipc, uint32_t preset);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_white_balance_value(void* ipc, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_exposure(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_exposure_priority_video(void* ipc, uint32_t enabled);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_iso_speed(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_backlight_compensation(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_scene_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_set_flat_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_rm_set_eye_selection(void* ipc, uint32_t enable);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_desired_optimization(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_primary_use(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_optical_image_stabilization(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_hdr_video(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_pv_set_regions_of_interest(void* ipc, uint32_t clear, uint32_t set, uint32_t auto_exposure, uint32_t auto_focus, uint32_t bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_set_interface_priority(void* ipc, uint16_t port, int32_t priority);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_set_quiet_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL rc_rm_map_camera_points(void* ipc, uint16_t port, uint32_t operation, hl2ss::vector_2 const* points, uint32_t count, hl2ss::vector_2*& out);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL rc_rm_get_rignode_world_poses(void* ipc, uint64_t const* timestamps, uint32_t count, hl2ss::matrix_4x4*& out);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ts_get_current_time(void* ipc, uint32_t source, uint64_t& timestamp);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_si_set_sampling_delay(void* ipc, int64_t delay);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_set_encoder_buffering(void* ipc, uint32_t enable);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_ee_set_reader_buffering(void* ipc, uint32_t enable);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_rm_set_loop_control(void* ipc, uint16_t port, uint32_t enable);

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL sm_set_volumes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL sm_get_observed_surfaces(void* ipc, uint64_t& size, hl2ss::sm_surface_info*& data);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL sm_get_meshes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size, void*& meshes_data);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL sm_unpack_mesh(void* meshes_data, uint64_t index, hl2ss::ulm::sm_mesh& mesh);

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL su_query(void* ipc, hl2ss::ulm::su_task const& task, hl2ss::ulm::su_result& header);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL su_unpack_item(void* items_data, uint64_t index, hl2ss::ulm::su_item& item);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL su_unpack_item_mesh(void* meshes_data, uint64_t index, hl2ss::ulm::su_mesh& mesh);

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL vi_start(void* ipc, char const* utf8_array);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL vi_pop(void* ipc, uint64_t& size, hl2ss::vi_result*& data);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL vi_stop(void* ipc);

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL umq_push(void* ipc, uint8_t const* data, uint32_t size);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL umq_pull(void* ipc, uint32_t* data, uint32_t count);

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL gmq_pull(void *ipc, hl2ss::ulm::gmq_message& result);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL gmq_push(void* ipc, uint32_t const* response, uint32_t count);
}
}

//******************************************************************************
// Interface
//******************************************************************************

namespace hl2ss
{
namespace shared
{
//-----------------------------------------------------------------------------
// Response
//-----------------------------------------------------------------------------

HL2SS_INLINE
void check_result(int32_t r)
{
    if (r < 0) { throw std::runtime_error("hl2ss::ulm operation error"); }
}

HL2SS_INLINE
void check_handle(void* h)
{
    if (!h) { throw std::runtime_error("hl2ss::ulm invalid handle"); }
}

class handle
{
protected:
    void* m_handle;
    
    handle(void* h) : m_handle(h)
    {
        check_handle(h);
    }

public:
    virtual ~handle()
    {
        hl2ss::ulm::close_handle(m_handle);
    }
};

template<typename T>
struct buffer
{
    uint64_t size;
    T* data;
};

//-----------------------------------------------------------------------------
// Core
//-----------------------------------------------------------------------------

HL2SS_INLINE
void initialize()
{
    check_result(hl2ss::ulm::initialize());
}

HL2SS_INLINE
void cleanup()
{
    check_result(hl2ss::ulm::cleanup());
}

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

template<typename T> class payload_map                                : public T                             { public: payload_map(uint8_t* payload, uint32_t size) {} };
template<>           class payload_map<hl2ss::map_rm_vlc>             : public hl2ss::map_rm_vlc             { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_vlc{ hl2ss::unpack_rm_vlc(payload, size) } {} };
template<>           class payload_map<hl2ss::map_rm_depth_ahat>      : public hl2ss::map_rm_depth_ahat      { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_depth_ahat{ hl2ss::unpack_rm_depth_ahat(payload, size) } {} };
template<>           class payload_map<hl2ss::map_rm_depth_longthrow> : public hl2ss::map_rm_depth_longthrow { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_depth_longthrow{ hl2ss::unpack_rm_depth_longthrow(payload, size) } {} };
template<>           class payload_map<hl2ss::map_rm_imu>             : public hl2ss::map_rm_imu             { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_imu{ hl2ss::unpack_rm_imu(payload, size) } {} };
template<>           class payload_map<hl2ss::map_pv>                 : public hl2ss::map_pv                 { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_pv{ hl2ss::unpack_pv(payload, size) } {} };
template<typename T> class payload_map<hl2ss::map_microphone<T>>      : public hl2ss::map_microphone<T>      { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_microphone<T>{ hl2ss::unpack_microphone<T>(payload, size) } {} };
template<>           class payload_map<hl2ss::map_si>                 : public hl2ss::map_si                 { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_si{ hl2ss::unpack_si(payload, size) } {} };
template<>           class payload_map<hl2ss::map_eet>                : public hl2ss::map_eet                { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_eet{ hl2ss::unpack_eet(payload, size) } {} };
template<>           class payload_map<hl2ss::map_extended_depth>     : public hl2ss::map_extended_depth     { public: payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_extended_depth{ hl2ss::unpack_extended_depth(payload, size) } {} };

class packet_view : protected handle, public hl2ss::ulm::packet
{
public:
    packet_view(void* source_handle, int64_t frame_stamp) : handle(hl2ss::ulm::get_by_index(source_handle, frame_stamp, *this))
    {
    }

    packet_view(void* source_handle, uint64_t timestamp, int32_t time_preference, bool tiebreak_right) : handle(hl2ss::ulm::get_by_timestamp(source_handle, timestamp, time_preference, tiebreak_right, *this))
    {
    }

    template<typename T>
    payload_map<T> unpack()
    {
        return { payload, sz_payload };
    }
};

//-----------------------------------------------------------------------------
// Interfaces
//-----------------------------------------------------------------------------

class source : protected handle
{
public:
    source(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration, uint8_t decoded) : handle(hl2ss::ulm::open_stream(host, port, buffer_size, configuration, decoded))
    {
    }
    
    std::unique_ptr<packet_view> get_by_index(int64_t frame_stamp)
    {
        return std::make_unique<packet_view>(m_handle, frame_stamp);
    }
    
    std::unique_ptr<packet_view> get_by_timestamp(uint64_t timestamp, int32_t time_preference, bool tiebreak_right)
    {
        return std::make_unique<packet_view>(m_handle, timestamp, time_preference, tiebreak_right);
    }
};

class ipc : protected handle
{
public:
    ipc(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }
};

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_INLINE
void start_subsystem_pv(char const* host, uint16_t port, void const* configuration)
{
    check_result(hl2ss::ulm::start_subsystem_pv(host, port, configuration));
}

HL2SS_INLINE
void stop_subsystem_pv(char const* host, uint16_t port)
{
    check_result(hl2ss::ulm::stop_subsystem_pv(host, port));
}

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

template<typename T> 
class calibration_view : protected handle, public buffer<T>
{
public:
    calibration_view(char const* host, uint16_t port, void const* configuration) : handle(hl2ss::ulm::download_calibration(host, port, configuration, (void*&)buffer<T>::data))
    {
        buffer<T>::size = 1;
    } 
};

class device_list_view : protected handle, public buffer<uint8_t>
{
public:
    device_list_view(char const* host, uint16_t port, void const* configuration) : handle(hl2ss::ulm::download_device_list(host, port, configuration, size, data))
    {
    }
};

//-----------------------------------------------------------------------------
// Remote Configuration
//-----------------------------------------------------------------------------

class rc_rm_map_camera_points_view : protected handle, public buffer<hl2ss::vector_2>
{
public:
    rc_rm_map_camera_points_view(void* ipc, uint16_t port, uint32_t operation, hl2ss::vector_2 const* points, uint32_t count) : handle(hl2ss::ulm::rc_rm_map_camera_points(ipc, port, operation, points, count, data))
    {
        size = count;
    }
};

class rc_rm_get_rignode_world_poses_view : protected handle, public buffer<hl2ss::matrix_4x4>
{
public:
    rc_rm_get_rignode_world_poses_view(void* ipc, uint64_t const* timestamps, uint32_t count) : handle(hl2ss::ulm::rc_rm_get_rignode_world_poses(ipc, timestamps, count, data))
    {
        size = count;
    }
};

class ipc_rc : protected ipc
{
public:
    ipc_rc(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    hl2ss::version ee_get_application_version()
    {
        hl2ss::version version;
        check_result(hl2ss::ulm::rc_ee_get_application_version(m_handle, version));
        return version;
    }

    uint64_t ts_get_utc_offset()
    {
        uint64_t offset;
        check_result(hl2ss::ulm::rc_ts_get_utc_offset(m_handle, offset));
        return offset;
    }

    void hs_set_marker_state(uint32_t state)
    {
        check_result(hl2ss::ulm::rc_hs_set_marker_state(m_handle, state));
    }

    uint32_t pv_get_subsystem_status()
    {
        uint32_t status;
        check_result(hl2ss::ulm::rc_pv_get_subsystem_status(m_handle, status));
        return status;
    }

    void pv_wait_for_subsystem(bool status)
    {
        check_result(hl2ss::ulm::rc_pv_wait_for_subsystem(m_handle, status));
    }

    void pv_set_focus(uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
    {
        check_result(hl2ss::ulm::rc_pv_set_focus(m_handle, mode, range, distance, value, driver_fallback));
    }

    void pv_set_video_temporal_denoising(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_video_temporal_denoising(m_handle, mode));
    }

    void pv_set_white_balance_preset(uint32_t preset)
    {
        check_result(hl2ss::ulm::rc_pv_set_white_balance_preset(m_handle, preset));
    }

    void pv_set_white_balance_value(uint32_t value)
    {
        check_result(hl2ss::ulm::rc_pv_set_white_balance_value(m_handle, value));
    }

    void pv_set_exposure(uint32_t mode, uint32_t value)
    {
        check_result(hl2ss::ulm::rc_pv_set_exposure(m_handle, mode, value));
    }

    void pv_set_exposure_priority_video(uint32_t enabled)
    {
        check_result(hl2ss::ulm::rc_pv_set_exposure_priority_video(m_handle, enabled));
    }

    void pv_set_iso_speed(uint32_t mode, uint32_t value)
    {
        check_result(hl2ss::ulm::rc_pv_set_iso_speed(m_handle, mode, value));
    }

    void pv_set_backlight_compensation(uint32_t state)
    {
        check_result(hl2ss::ulm::rc_pv_set_backlight_compensation(m_handle, state));
    }

    void pv_set_scene_mode(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_scene_mode(m_handle, mode));
    }

    void ee_set_flat_mode(bool mode)
    {
        check_result(hl2ss::ulm::rc_ee_set_flat_mode(m_handle, mode));
    }

    void rm_set_eye_selection(bool enable)
    {
        check_result(hl2ss::ulm::rc_rm_set_eye_selection(m_handle, enable));
    }

    void pv_set_desired_optimization(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_desired_optimization(m_handle, mode));
    }

    void pv_set_primary_use(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_primary_use(m_handle, mode));
    }

    void pv_set_optical_image_stabilization(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_optical_image_stabilization(m_handle, mode));
    }

    void pv_set_hdr_video(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_pv_set_hdr_video(m_handle, mode));
    }

    void pv_set_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
    {
        check_result(hl2ss::ulm::rc_pv_set_regions_of_interest(m_handle, clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h));
    }

    void ee_set_interface_priority(uint16_t port, int32_t priority)
    {
        check_result(hl2ss::ulm::rc_ee_set_interface_priority(m_handle, port, priority));
    }

    void ee_set_quiet_mode(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_ee_set_quiet_mode(m_handle, mode));
    }

    std::unique_ptr<rc_rm_map_camera_points_view> rm_map_camera_points(uint16_t port, uint32_t operation, hl2ss::vector_2* points, uint32_t count)
    {
        return std::make_unique<rc_rm_map_camera_points_view>(m_handle, port, operation, points, count);
    }

    std::unique_ptr<rc_rm_get_rignode_world_poses_view> rm_get_rignode_world_poses(uint64_t* timestamps, uint32_t count)
    {
        return std::make_unique<rc_rm_get_rignode_world_poses_view>(m_handle, timestamps, count);
    }

    uint64_t ts_get_current_time(uint32_t source)
    {
        uint64_t timestamp;
        check_result(hl2ss::ulm::rc_ts_get_current_time(m_handle, source, timestamp));
        return timestamp;
    }

    void si_set_sampling_delay(int64_t delay)
    {
        check_result(hl2ss::ulm::rc_si_set_sampling_delay(m_handle, delay));
    }

    void ee_set_encoder_buffering(bool enable)
    {
        check_result(hl2ss::ulm::rc_ee_set_encoder_buffering(m_handle, enable));
    }

    void ee_set_reader_buffering(bool enable)
    {
        check_result(hl2ss::ulm::rc_ee_set_reader_buffering(m_handle, enable));
    }

    void rm_set_loop_control(uint16_t port, bool enable)
    {
        check_result(hl2ss::ulm::rc_rm_set_loop_control(m_handle, port, enable));
    }
};

//-----------------------------------------------------------------------------
// Spatial Mapping
//-----------------------------------------------------------------------------

class sm_surface_info_view : protected handle, public buffer<hl2ss::sm_surface_info>
{
public:
    sm_surface_info_view(void* ipc) : handle(hl2ss::ulm::sm_get_observed_surfaces(ipc, size, data))
    {
    }
};

class sm_mesh_collection_view : protected handle
{
private:
    void* meshes_data;

public:
    std::vector<hl2ss::ulm::sm_mesh> meshes;

    sm_mesh_collection_view(void* ipc, uint32_t count, uint8_t const* data, uint64_t size) : handle(hl2ss::ulm::sm_get_meshes(ipc, count, data, size, meshes_data)), meshes{ count }
    {
        for (uint32_t i = 0; i < count; ++i) { check_result(hl2ss::ulm::sm_unpack_mesh(meshes_data, i, meshes[i])); }
    }
};

class ipc_sm : protected ipc
{
public:
    ipc_sm(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    void set_volumes(hl2ss::sm_bounding_volume const& volumes)
    {
        check_result(hl2ss::ulm::sm_set_volumes(m_handle, volumes.get_count(), volumes.get_data(), volumes.get_size()));
    }

    std::unique_ptr<sm_surface_info_view> get_observed_surfaces()
    {
        return std::make_unique<sm_surface_info_view>(m_handle);
    }

    std::unique_ptr<sm_mesh_collection_view> get_meshes(hl2ss::sm_mesh_task const& tasks)
    {
        return std::make_unique<sm_mesh_collection_view>(m_handle, tasks.get_count(), tasks.get_data(), tasks.get_size());
    }
};

//-----------------------------------------------------------------------------
// Scene Understanding
//-----------------------------------------------------------------------------

class su_item_view : public hl2ss::ulm::su_item
{
public:
    std::vector<hl2ss::ulm::su_mesh> unpacked_meshes;
    std::vector<hl2ss::ulm::su_mesh> unpacked_collider_meshes;
};

class su_result_view : protected handle, public hl2ss::ulm::su_result
{
private:
    void unpack_meshes(void* meshes_data, uint64_t count, std::vector<hl2ss::ulm::su_mesh>& out)
    {
        out.resize(count);
        for (uint64_t i = 0; i < count; ++i) { check_result(hl2ss::ulm::su_unpack_item_mesh(meshes_data, i, out[i])); }
    }

    void unpack_item(uint64_t index)
    {
        su_item_view& item = items[index];
        unpack_meshes(item.meshes_data,          item.meshes_count,          item.unpacked_meshes);
        unpack_meshes(item.collider_meshes_data, item.collider_meshes_count, item.unpacked_collider_meshes);
    }

public:
    std::vector<su_item_view> items;

    su_result_view(void* ipc, hl2ss::ulm::su_task const& task) : handle(hl2ss::ulm::su_query(ipc, task, *this))
    {
        if (status != 0) { return; }
        items.resize(items_count);
        for (uint64_t i = 0; i < items_count; ++i) { check_result(hl2ss::ulm::su_unpack_item(this->items_data, i, items[i])); }
        for (uint64_t i = 0; i < items_count; ++i) { unpack_item(i); }
    }
};

class ipc_su : protected ipc
{
public:
    ipc_su(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    std::unique_ptr<su_result_view> query(hl2ss::su_task const& task)
    {
        hl2ss::ulm::su_task t;

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
        t.guid_list_size       = task.guid_list.size();
        t.guid_list_data       = task.guid_list.data();

        return std::make_unique<su_result_view>(m_handle, t);
    }
};

//-----------------------------------------------------------------------------
// Voice Input
//-----------------------------------------------------------------------------

class vi_result_view : protected handle, public buffer<hl2ss::vi_result>
{
public:
    vi_result_view(void* ipc) : handle(hl2ss::ulm::vi_pop(ipc, size, data))
    {
    }
};

class ipc_vi : protected ipc
{
public:
    ipc_vi(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    void start(char const* utf8_array)
    {
        check_result(hl2ss::ulm::vi_start(m_handle, utf8_array));
    }

    std::unique_ptr<vi_result_view> pop()
    {
        return std::make_unique<vi_result_view>(m_handle);
    }

    void stop()
    {
        check_result(hl2ss::ulm::vi_stop(m_handle));
    }
};

//-----------------------------------------------------------------------------
// Unity Message Queue
//-----------------------------------------------------------------------------

class ipc_umq : protected ipc
{
public:
    ipc_umq(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    void push(uint8_t const* data, uint32_t size)
    {
        check_result(hl2ss::ulm::umq_push(m_handle, data, size));
    }

    void pull(uint32_t* data, uint32_t count)
    {
        check_result(hl2ss::ulm::umq_pull(m_handle, data, count));
    }
};

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

class gmq_message_view : protected handle, public hl2ss::ulm::gmq_message
{
public:
    gmq_message_view(void* ipc) : handle(hl2ss::ulm::gmq_pull(ipc, *this))
    {
    }
};

class ipc_gmq : protected ipc
{
public:
    ipc_gmq(char const* host, uint16_t port) : ipc(host, port)
    {
    }

    std::unique_ptr<gmq_message_view> pull()
    {
        return std::make_unique<gmq_message_view>(m_handle);
    }

    void push(uint32_t const* response, uint32_t count)
    {
        check_result(hl2ss::ulm::gmq_push(m_handle, response, count));
    }
};
}
}

//******************************************************************************
// Services
//******************************************************************************

namespace hl2ss
{
namespace svc
{
//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

HL2SS_INLINE
void initialize()
{
    hl2ss::shared::initialize();
}

HL2SS_INLINE
void cleanup()
{
    hl2ss::shared::cleanup();
}

template<typename T>
std::unique_ptr<T> open_stream(char const* host, uint16_t port, uint64_t buffer_size, void* configuration, uint8_t decoded)
{
    return std::make_unique<T>(host, port, buffer_size, configuration, decoded);
}

template<typename T>
std::unique_ptr<T> open_ipc(char const* host, uint16_t port)
{
    return std::make_unique<T>(host, port);
}

HL2SS_INLINE
void start_subsystem_pv(char const* host, uint16_t port, void const* configuration)
{
    hl2ss::shared::start_subsystem_pv(host, port, configuration);
}

HL2SS_INLINE
void stop_subsystem_pv(char const* host, uint16_t port)
{
    hl2ss::shared::stop_subsystem_pv(host, port);
}

template<typename T>
std::unique_ptr<hl2ss::shared::calibration_view<T>> download_calibration(char const* host, uint16_t port, void const* configuration)
{
    return std::make_unique<hl2ss::shared::calibration_view<T>>(host, port, configuration);
}

HL2SS_INLINE
std::unique_ptr<hl2ss::shared::device_list_view> download_device_list(char const* host, uint16_t port, void const* configuration)
{
    return std::make_unique<hl2ss::shared::device_list_view>(host, port, configuration);
}
}
}

#endif
