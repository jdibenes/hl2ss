
#include <stdint.h>
#include <vector>
#include <exception>
#include <stdexcept>
#include <memory>

#define HL2SS_SHARED

#include "hl2ss.h"
#include "hl2ss_mt.h"

#ifdef _WIN32
#define HL2SS_CLIENT_IMPORT extern "C" __declspec(dllimport)
#define HL2SS_CALL 
#else
#define HL2SS_CLIENT_IMPORT extern "C"
#define HL2SS_CALL
#endif

//******************************************************************************
// Unified Library Methods Module
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
    uint8_t fps;
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
    guid const* guid_list_data;
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

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL initialize();

//-----------------------------------------------------------------------------
// Interfaces
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL open_stream(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL open_ipc(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void HL2SS_CALL close_handle(void* h);

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
int32_t HL2SS_CALL start_subsystem_pv(char const* host, uint16_t port, configuration_pv_subsystem const& c);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL stop_subsystem_pv(char const* host, uint16_t port);

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL download_calibration(char const* host, uint16_t port, void const* configuration, void*& calibration);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL download_device_list(char const* host, uint16_t port, uint64_t& size, uint8_t*& query);

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_get_application_version(void* ipc, hl2ss::version& version);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_get_utc_offset(void* ipc, uint64_t& offset);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_hs_marker_state(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_get_pv_subsystem_status(void* ipc, uint32_t& status);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_wait_for_pv_subsystem(void* ipc, uint32_t status);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_video_temporal_denoising(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_white_balance_preset(void* ipc, uint32_t preset);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_white_balance_value(void* ipc, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_exposure(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_exposure_priority_video(void* ipc, uint32_t enabled);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_iso_speed(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_backlight_compensation(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_scene_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_flat_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_rm_eye_selection(void* ipc, uint32_t enable);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_desired_optimization(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_primary_use(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_optical_image_stabilization(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_hdr_video(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_pv_regions_of_interest(void* ipc, uint32_t clear, uint32_t set, uint32_t auto_exposure, uint32_t auto_focus, uint32_t bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_interface_priority(void* ipc, uint16_t port, int32_t priority);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL rc_set_quiet_mode(void* ipc, uint32_t mode);

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL sm_set_volumes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL sm_get_observed_surfaces(void* ipc, uint64_t& size, hl2ss::sm_surface_info*& data);

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL sm_get_meshes(void* ipc, uint32_t count, uint8_t const* data, uint64_t size);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL sm_unpack_mesh(void* reference, uint32_t index, hl2ss::ulm::sm_mesh& mesh);

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* HL2SS_CALL su_query(void* ipc, hl2ss::ulm::su_task const& task, hl2ss::ulm::su_result_header& header);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL su_unpack_item(void* reference, uint32_t index, hl2ss::ulm::su_item& item);

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL su_unpack_item_mesh(void* meshes, uint32_t index, hl2ss::ulm::su_mesh& mesh);

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
int32_t HL2SS_CALL umq_push(void* ipc, uint8_t const* data, uint64_t size);

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
// Library Helpers
//******************************************************************************

namespace hl2ss
{
namespace svc
{

//-----------------------------------------------------------------------------
// Handle
//-----------------------------------------------------------------------------

class handle
{
protected:
    void* m_handle;

    static void check_result(void* handle)
    {
        if (!handle) { throw std::runtime_error("ULM invalid handle"); }
    }

    handle(void* h) : m_handle(h)
    {
        check_result(h);
    }

public:
    static void check_result(int32_t result)
    {
        if (result < 0) { throw std::runtime_error("ULM operation error"); }
    }

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
// Stream
//-----------------------------------------------------------------------------

template<typename T>
class payload_map : public T
{
public:
    payload_map(uint8_t* payload, uint32_t)
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_vlc> : public hl2ss::map_rm_vlc
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_vlc{ hl2ss::unpack_rm_vlc(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_depth_ahat> : public hl2ss::map_rm_depth_ahat
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_depth_ahat{ hl2ss::unpack_rm_depth_ahat(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_depth_longthrow> : public hl2ss::map_rm_depth_longthrow
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_depth_longthrow{ hl2ss::unpack_rm_depth_longthrow(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_imu> : public hl2ss::map_rm_imu
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_rm_imu{ hl2ss::unpack_rm_imu(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_pv> : public hl2ss::map_pv
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_pv{ hl2ss::unpack_pv(payload, size) }
    {
    }
};

template<typename TD>
class payload_map<hl2ss::map_microphone<TD>> : public hl2ss::map_microphone<TD>
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_microphone<TD>{ hl2ss::unpack_microphone<TD>(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_si> : public hl2ss::map_si
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_si{ hl2ss::unpack_si(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_eet> : public hl2ss::map_eet
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_eet{ hl2ss::unpack_eet(payload, size) }
    {
    }
};

template<>
class payload_map<hl2ss::map_extended_depth> : public hl2ss::map_extended_depth
{
public:
    payload_map(uint8_t* payload, uint32_t size) : hl2ss::map_extended_depth{ hl2ss::unpack_extended_depth(payload, size) }
    {
    }
};

class packet : protected handle, public hl2ss::ulm::packet
{
public:
    packet(void* source_handle, int64_t frame_stamp) : handle(hl2ss::ulm::get_by_index(source_handle, frame_stamp, *this))
    {
    }

    packet(void* source_handle, uint64_t timestamp, int32_t time_preference, bool tiebreak_right) : handle(hl2ss::ulm::get_by_timestamp(source_handle, timestamp, time_preference, tiebreak_right, *this))
    {
    }

    template<typename T>
    payload_map<T> unpack()
    {
        return { payload, sz_payload };
    }
};

class source : protected handle
{
public:
    source(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration) : handle(hl2ss::ulm::open_stream(host, port, buffer_size, configuration))
    {
    }

    std::unique_ptr<packet> get_by_index(int64_t frame_stamp)
    {
        return std::make_unique<packet>(m_handle, frame_stamp);
    }

    std::unique_ptr<packet> get_by_timestamp(uint64_t timestamp, int32_t time_preference, bool tiebreak_right)
    {
        return std::make_unique<packet>(m_handle, timestamp, time_preference, tiebreak_right);
    }
};

template<typename T>
class calibration : protected handle, public buffer<T>
{
public:
    calibration(char const* host, uint16_t port, void const* configuration) : handle(hl2ss::ulm::download_calibration(host, port, configuration, (void*&)buffer<T>::data))
    {
        buffer<T>::size = 1;
    }
};

class device_list : protected handle, public buffer<uint8_t>
{
public:
    device_list(char const* host, uint16_t port) : handle(hl2ss::ulm::download_device_list(host, port, size, data))
    {
    }
};

//-----------------------------------------------------------------------------
// Remote Configuration
//-----------------------------------------------------------------------------

class ipc_rc : protected handle
{
public:
    ipc_rc(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    hl2ss::version get_application_version()
    {
        hl2ss::version version;
        check_result(hl2ss::ulm::rc_get_application_version(m_handle, version));
        return version;
    }

    uint64_t get_utc_offset()
    {
        uint64_t offset;
        check_result(hl2ss::ulm::rc_get_utc_offset(m_handle, offset));
        return offset;
    }

    void set_hs_marker_state(uint32_t state)
    {
        check_result(hl2ss::ulm::rc_set_hs_marker_state(m_handle, state));
    }

    uint32_t get_pv_subsystem_status()
    {
        uint32_t status;
        check_result(hl2ss::ulm::rc_get_pv_subsystem_status(m_handle, status));
        return status;
    }

    void wait_for_pv_subsystem(bool status)
    {
        check_result(hl2ss::ulm::rc_wait_for_pv_subsystem(m_handle, status));
    }

    void set_pv_focus(uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
    {
        check_result(hl2ss::ulm::rc_set_pv_focus(m_handle, mode, range, distance, value, driver_fallback));
    }

    void set_pv_video_temporal_denoising(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_video_temporal_denoising(m_handle, mode));
    }

    void set_pv_white_balance_preset(uint32_t preset)
    {
        check_result(hl2ss::ulm::rc_set_pv_white_balance_preset(m_handle, preset));
    }

    void set_pv_white_balance_value(uint32_t value)
    {
        check_result(hl2ss::ulm::rc_set_pv_white_balance_value(m_handle, value));
    }

    void set_pv_exposure(uint32_t mode, uint32_t value)
    {
        check_result(hl2ss::ulm::rc_set_pv_exposure(m_handle, mode, value));
    }

    void set_pv_exposure_priority_video(uint32_t enabled)
    {
        check_result(hl2ss::ulm::rc_set_pv_exposure_priority_video(m_handle, enabled));
    }

    void set_pv_iso_speed(uint32_t mode, uint32_t value)
    {
        check_result(hl2ss::ulm::rc_set_pv_iso_speed(m_handle, mode, value));
    }

    void set_pv_backlight_compensation(uint32_t state)
    {
        check_result(hl2ss::ulm::rc_set_pv_backlight_compensation(m_handle, state));
    }

    void set_pv_scene_mode(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_scene_mode(m_handle, mode));
    }

    void set_flat_mode(bool mode)
    {
        check_result(hl2ss::ulm::rc_set_flat_mode(m_handle, mode));
    }

    void set_rm_eye_selection(bool enable)
    {
        check_result(hl2ss::ulm::rc_set_rm_eye_selection(m_handle, enable));
    }

    void set_pv_desired_optimization(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_desired_optimization(m_handle, mode));
    }

    void set_pv_primary_use(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_primary_use(m_handle, mode));
    }

    void set_pv_optical_image_stabilization(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_optical_image_stabilization(m_handle, mode));
    }

    void set_pv_hdr_video(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_pv_hdr_video(m_handle, mode));
    }

    void set_pv_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
    {
        check_result(hl2ss::ulm::rc_set_pv_regions_of_interest(m_handle, clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h));
    }

    void set_interface_priority(uint16_t port, int32_t priority)
    {
        check_result(hl2ss::ulm::rc_set_interface_priority(m_handle, port, priority));
    }

    void set_quiet_mode(uint32_t mode)
    {
        check_result(hl2ss::ulm::rc_set_quiet_mode(m_handle, mode));
    }
};

//-----------------------------------------------------------------------------
// Spatial Mapping
//-----------------------------------------------------------------------------

class sm_surface_info_collection : protected handle, public buffer<hl2ss::sm_surface_info>
{
public:
    sm_surface_info_collection(void* ipc) : handle(hl2ss::ulm::sm_get_observed_surfaces(ipc, size, data))
    {
    }
};

class sm_mesh_collection : protected handle
{
public:
    std::vector<hl2ss::ulm::sm_mesh> meshes;

    sm_mesh_collection(void* ipc, uint32_t count, uint8_t const* data, uint64_t size) : handle(hl2ss::ulm::sm_get_meshes(ipc, count, data, size)), meshes{ count }
    {
        for (uint32_t i = 0; i < count; ++i) { check_result(hl2ss::ulm::sm_unpack_mesh(m_handle, i, meshes[i])); }
    }
};

class ipc_sm : protected handle
{
public:
    ipc_sm(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    void set_volumes(hl2ss::sm_bounding_volume const& volumes)
    {
        check_result(hl2ss::ulm::sm_set_volumes(m_handle, volumes.get_count(), volumes.get_data(), volumes.get_size()));
    }

    std::unique_ptr<sm_surface_info_collection> get_observed_surfaces()
    {
        return std::make_unique<sm_surface_info_collection>(m_handle);
    }

    std::unique_ptr<sm_mesh_collection> get_meshes(hl2ss::sm_mesh_task const& tasks)
    {
        return std::make_unique<sm_mesh_collection>(m_handle, tasks.get_count(), tasks.get_data(), tasks.get_size());
    }
};

//-----------------------------------------------------------------------------
// Scene Understanding
//-----------------------------------------------------------------------------

class su_item : public hl2ss::ulm::su_item
{
public:
    std::vector<hl2ss::ulm::su_mesh> unpacked_meshes;
    std::vector<hl2ss::ulm::su_mesh> unpacked_collider_meshes;
};

class su_result : protected handle, public hl2ss::ulm::su_result_header
{
private:
    void unpack_meshes(void* meshes, uint32_t count, std::vector<hl2ss::ulm::su_mesh>& out)
    {
        out.resize(count);
        for (uint32_t i = 0; i < count; ++i) { check_result(hl2ss::ulm::su_unpack_item_mesh(meshes, i, out[i])); }
    }

    void unpack_item(uint32_t index)
    {
        su_item& item = items[index];
        unpack_meshes(item.meshes,          item.meshes_count,          item.unpacked_meshes);
        unpack_meshes(item.collider_meshes, item.collider_meshes_count, item.unpacked_collider_meshes);
    }

public:
    std::vector<su_item> items;

    su_result(void* ipc, hl2ss::ulm::su_task const& task) : handle(hl2ss::ulm::su_query(ipc, task, *this))
    {
        if (status != 0) { return; }
        items.resize(count);
        for (uint32_t i = 0; i < count; ++i) { check_result(hl2ss::ulm::su_unpack_item(m_handle, i, items[i])); }
        for (uint32_t i = 0; i < count; ++i) { unpack_item(i); }
    }
};

class ipc_su : protected handle
{
public:
    ipc_su(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    std::unique_ptr<su_result> query(hl2ss::su_task const& task)
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

        return std::make_unique<su_result>(m_handle, t);
    }
};

//-----------------------------------------------------------------------------
// Voice Input
//-----------------------------------------------------------------------------

class vi_result : protected handle, public buffer<hl2ss::vi_result>
{
public:
    vi_result(void* ipc) : handle(hl2ss::ulm::vi_pop(ipc, size, data))
    {
    }
};

class ipc_vi : protected handle
{
public:
    ipc_vi(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    void start(char const* utf8_array)
    {
        check_result(hl2ss::ulm::vi_start(m_handle, utf8_array));
    }

    std::unique_ptr<vi_result> pop()
    {
        return std::make_unique<vi_result>(m_handle);
    }

    void stop()
    {
        check_result(hl2ss::ulm::vi_stop(m_handle));
    }
};

//-----------------------------------------------------------------------------
// Unity Message Queue
//-----------------------------------------------------------------------------

class ipc_umq : protected handle
{
public:
    ipc_umq(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    void push(uint8_t const* data, uint64_t size)
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

class gmq_message : protected handle, public hl2ss::ulm::gmq_message
{
public:
    gmq_message(void* ipc) : handle(hl2ss::ulm::gmq_pull(ipc, *this))
    {
    }
};

class ipc_gmq : protected handle
{
public:
    ipc_gmq(char const* host, uint16_t port) : handle(hl2ss::ulm::open_ipc(host, port))
    {
    }

    std::unique_ptr<gmq_message> pull()
    {
        return std::make_unique<gmq_message>(m_handle);
    }

    void push(uint32_t const* response, uint32_t count)
    {
        check_result(hl2ss::ulm::gmq_push(m_handle, response, count));
    }
};

//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

HL2SS_INLINE
void initialize()
{
    handle::check_result(hl2ss::ulm::initialize());
}

template<typename T>
T create_configuration()
{
    return T();
}

template<>
hl2ss::ulm::configuration_rm_vlc create_configuration()
{
    hl2ss::ulm::configuration_rm_vlc c;

    c.chunk = hl2ss::chunk_size::RM_VLC;
    c.mode = hl2ss::stream_mode::MODE_1;
    c.divisor = 1;
    c.profile = hl2ss::video_profile::H265_MAIN;
    c.level = hl2ss::h26x_level::DEFAULT;
    c.bitrate = 0;
    c.options_data = nullptr;
    c.options_size = -1;

    return c;
}

template<>
hl2ss::ulm::configuration_rm_depth_ahat create_configuration()
{
    hl2ss::ulm::configuration_rm_depth_ahat c;

    c.chunk = hl2ss::chunk_size::RM_DEPTH_AHAT;
    c.mode = hl2ss::stream_mode::MODE_1;
    c.divisor = 1;
    c.profile_z = hl2ss::depth_profile::SAME;
    c.profile_ab = hl2ss::video_profile::H265_MAIN;
    c.level = hl2ss::h26x_level::DEFAULT;
    c.bitrate = 0;
    c.options_data = nullptr;
    c.options_size = -1;

    return c;
}

template<>
hl2ss::ulm::configuration_rm_depth_longthrow create_configuration()
{
    hl2ss::ulm::configuration_rm_depth_longthrow c;

    c.chunk = hl2ss::chunk_size::RM_DEPTH_LONGTHROW;
    c.mode = hl2ss::stream_mode::MODE_1;
    c.divisor = 1;
    c.png_filter = hl2ss::png_filter_mode::PAETH;

    return c;
}

template<>
hl2ss::ulm::configuration_rm_imu create_configuration()
{
    hl2ss::ulm::configuration_rm_imu c;

    c.chunk = hl2ss::chunk_size::RM_IMU;
    c.mode = hl2ss::stream_mode::MODE_1;

    return c;
}

template<>
hl2ss::ulm::configuration_pv create_configuration()
{
    hl2ss::ulm::configuration_pv c;

    c.width = 1920;
    c.height = 1080;
    c.framerate = 30;
    c.chunk = hl2ss::chunk_size::PERSONAL_VIDEO;
    c.mode = hl2ss::stream_mode::MODE_1;
    c.divisor = 1;
    c.profile = hl2ss::video_profile::H265_MAIN;
    c.level = hl2ss::h26x_level::DEFAULT;
    c.bitrate = 0;
    c.options_data = nullptr;
    c.options_size = -1;
    c.decoded_format = hl2ss::pv_decoded_format::BGR;

    return c;
}

template<>
hl2ss::ulm::configuration_microphone create_configuration()
{
    hl2ss::ulm::configuration_microphone c;

    c.chunk = hl2ss::chunk_size::MICROPHONE;
    c.profile = hl2ss::audio_profile::AAC_24000;
    c.level = hl2ss::aac_level::L2;

    return c;
}

template<>
hl2ss::ulm::configuration_si create_configuration()
{
    hl2ss::ulm::configuration_si c;

    c.chunk = hl2ss::chunk_size::SPATIAL_INPUT;

    return c;
}

template<>
hl2ss::ulm::configuration_eet create_configuration()
{
    hl2ss::ulm::configuration_eet c;

    c.chunk = hl2ss::chunk_size::EXTENDED_EYE_TRACKER;
    c.fps = 30;

    return c;
}

template<>
hl2ss::ulm::configuration_extended_audio create_configuration()
{
    hl2ss::ulm::configuration_extended_audio c;

    c.chunk = hl2ss::chunk_size::EXTENDED_AUDIO;
    c.mixer_mode = hl2ss::mixer_mode::BOTH;
    c.loopback_gain = 1.0f;
    c.microphone_gain = 1.0f;
    c.profile = hl2ss::audio_profile::AAC_24000;
    c.level = hl2ss::aac_level::L2;

    return c;
}

template<>
hl2ss::ulm::configuration_extended_depth create_configuration()
{
    hl2ss::ulm::configuration_extended_depth c;

    c.chunk = hl2ss::chunk_size::EXTENDED_DEPTH;
    c.media_index = 0xFFFFFFFF;
    c.stride_mask = 0x3F;
    c.mode = hl2ss::stream_mode::MODE_0;
    c.divisor = 1;
    c.profile_z = hl2ss::depth_profile::ZDEPTH;

    return c;
}

template<>
hl2ss::ulm::configuration_pv_subsystem create_configuration()
{
    hl2ss::ulm::configuration_pv_subsystem c;

    c.enable_mrc = false;
    c.hologram_composition = true;
    c.recording_indicator = false;
    c.video_stabilization = false;
    c.blank_protected = false;
    c.show_mesh = false;
    c.shared = false;
    c.global_opacity = 0.9f;
    c.output_width = 0.0f;
    c.output_height = 0.0f;
    c.video_stabilization_length = 0;
    c.hologram_perspective = hl2ss::hologram_perspective::PV;

    return c;
}

HL2SS_INLINE
std::unique_ptr<source> open_stream(char const* host, uint16_t port, uint64_t buffer_size, void const* configuration)
{
    return std::make_unique<source>(host, port, buffer_size, configuration);
}

template<typename T>
std::unique_ptr<T> open_ipc(char const* host, uint16_t port)
{
    return std::make_unique<T>(host, port);
}

HL2SS_INLINE
void start_subsystem_pv(char const* host, uint16_t port, hl2ss::ulm::configuration_pv_subsystem const& c)
{
    handle::check_result(hl2ss::ulm::start_subsystem_pv(host, port, c));
}

HL2SS_INLINE
void stop_subsystem_pv(char const* host, uint16_t port)
{
    handle::check_result(hl2ss::ulm::stop_subsystem_pv(host, port));
}

template<typename T>
std::unique_ptr<calibration<T>> download_calibration(char const* host, uint16_t port, void const* configuration)
{
    return std::make_unique<calibration<T>>(host, port, configuration);
}

HL2SS_INLINE
std::unique_ptr<device_list> download_device_list(char const* host, uint16_t port)
{
    return std::make_unique<device_list>(host, port);
}

}
}
