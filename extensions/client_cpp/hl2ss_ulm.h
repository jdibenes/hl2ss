
#include <stdint.h>
#include <vector>
#include <exception>
#include <stdexcept>
#include <memory>

#ifdef _WIN32
#define HL2SS_CLIENT_IMPORT extern "C" __declspec(dllimport)
#define HL2SS_CALL 
#else
#define HL2SS_CLIENT_IMPORT extern "C"
#define HL2SS_CALL
#endif

#define HL2SS_INLINE inline

//******************************************************************************
// HL2SS Module
//******************************************************************************

namespace hl2ss
{

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

namespace stream_port
{
uint16_t const RM_VLC_LEFTFRONT     = 3800;
uint16_t const RM_VLC_LEFTLEFT      = 3801;
uint16_t const RM_VLC_RIGHTFRONT    = 3802;
uint16_t const RM_VLC_RIGHTRIGHT    = 3803;
uint16_t const RM_DEPTH_AHAT        = 3804;
uint16_t const RM_DEPTH_LONGTHROW   = 3805;
uint16_t const RM_IMU_ACCELEROMETER = 3806;
uint16_t const RM_IMU_GYROSCOPE     = 3807;
uint16_t const RM_IMU_MAGNETOMETER  = 3808;
uint16_t const PERSONAL_VIDEO       = 3810;
uint16_t const MICROPHONE           = 3811;
uint16_t const SPATIAL_INPUT        = 3812;
uint16_t const EXTENDED_EYE_TRACKER = 3817;
uint16_t const EXTENDED_AUDIO       = 3818;
uint16_t const EXTENDED_VIDEO       = 3819;
uint16_t const EXTENDED_DEPTH       = 3821;
}

namespace ipc_port
{
uint16_t const REMOTE_CONFIGURATION = 3809;
uint16_t const SPATIAL_MAPPING      = 3813;
uint16_t const SCENE_UNDERSTANDING  = 3814;
uint16_t const VOICE_INPUT          = 3815;
uint16_t const UNITY_MESSAGE_QUEUE  = 3816;
uint16_t const GUEST_MESSAGE_QUEUE  = 3820;
}

namespace chunk_size
{
uint64_t const RM_VLC               = 4096;
uint64_t const RM_DEPTH_AHAT        = 4096;
uint64_t const RM_DEPTH_LONGTHROW   = 4096;
uint64_t const RM_IMU               = 4096;
uint64_t const PERSONAL_VIDEO       = 4096;
uint64_t const MICROPHONE           = 512;
uint64_t const SPATIAL_INPUT        = 1024;
uint64_t const EXTENDED_EYE_TRACKER = 256;
uint64_t const EXTENDED_AUDIO       = 512;
uint64_t const EXTENDED_VIDEO       = 4096;
uint64_t const EXTENDED_DEPTH       = 4096;
uint64_t const SINGLE_TRANSFER      = 4096;
}

namespace stream_mode
{
uint8_t const MODE_0 = 0;
uint8_t const MODE_1 = 1;
uint8_t const MODE_2 = 2;
uint8_t const MODE_3 = 3;
}

namespace video_profile
{
uint8_t const H264_BASE = 0;
uint8_t const H264_MAIN = 1;
uint8_t const H264_HIGH = 2;
uint8_t const H265_MAIN = 3;
uint8_t const RAW       = 0xFF;
}

namespace h26x_level
{
uint8_t const H264_1   =  10;
uint8_t const H264_1_b =  11;
uint8_t const H264_1_1 =  11;
uint8_t const H264_1_2 =  12;
uint8_t const H264_1_3 =  13;
uint8_t const H264_2   =  20;
uint8_t const H264_2_1 =  21;
uint8_t const H264_2_2 =  22;
uint8_t const H264_3   =  30;
uint8_t const H264_3_1 =  31;
uint8_t const H264_3_2 =  32;
uint8_t const H264_4   =  40;
uint8_t const H264_4_1 =  41;
uint8_t const H264_4_2 =  42;
uint8_t const H264_5   =  50;
uint8_t const H264_5_1 =  51;
uint8_t const H264_5_2 =  52;
uint8_t const H265_1   =  30;
uint8_t const H265_2   =  60;
uint8_t const H265_2_1 =  63;
uint8_t const H265_3   =  90;
uint8_t const H265_3_1 =  93;
uint8_t const H265_4   = 120;
uint8_t const H265_4_1 = 123;
uint8_t const H265_5   = 150;
uint8_t const H265_5_1 = 153;
uint8_t const H265_5_2 = 156;
uint8_t const H265_6   = 180;
uint8_t const H265_6_1 = 183;
uint8_t const H265_6_2 = 186;
uint8_t const DEFAULT  = 255;
}

namespace depth_profile
{
uint8_t const SAME   = 0;
uint8_t const ZDEPTH = 1;
}

namespace audio_profile
{
uint8_t const AAC_12000 = 0;
uint8_t const AAC_16000 = 1;
uint8_t const AAC_20000 = 2;
uint8_t const AAC_24000 = 3;
uint8_t const RAW       = 0xFF;
}

namespace aac_level
{
uint8_t const L2      = 0x29;
uint8_t const L4      = 0x2A;
uint8_t const L5      = 0x2B;
uint8_t const HEV1L2  = 0x2C;
uint8_t const HEV1L4  = 0x2E;
uint8_t const HEV1L5  = 0x2F;
uint8_t const HEV2L2  = 0x30;
uint8_t const HEV2L3  = 0x31;
uint8_t const HEV2L4  = 0x32;
uint8_t const HEV2L5  = 0x33;
}

namespace png_filter_mode
{
uint8_t const AUTOMATIC = 0;
uint8_t const DISABLE   = 1;
uint8_t const SUB       = 2;
uint8_t const UP        = 3;
uint8_t const AVERAGE   = 4;
uint8_t const PAETH     = 5;
uint8_t const ADAPTIVE  = 6;
}

namespace h26x_encoder_property
{
uint64_t const CODECAPI_AVEncCommonRateControlMode     =  0;
uint64_t const CODECAPI_AVEncCommonQuality             =  1;
uint64_t const CODECAPI_AVEncAdaptiveMode              =  2;
uint64_t const CODECAPI_AVEncCommonBufferSize          =  3;
uint64_t const CODECAPI_AVEncCommonMaxBitRate          =  4;
uint64_t const CODECAPI_AVEncCommonMeanBitRate         =  5;
uint64_t const CODECAPI_AVEncCommonQualityVsSpeed      =  6;
uint64_t const CODECAPI_AVEncH264CABACEnable           =  7;
uint64_t const CODECAPI_AVEncH264SPSID                 =  8;
uint64_t const CODECAPI_AVEncMPVDefaultBPictureCount   =  9;
uint64_t const CODECAPI_AVEncMPVGOPSize                = 10;
uint64_t const CODECAPI_AVEncNumWorkerThreads          = 11;
uint64_t const CODECAPI_AVEncVideoContentType          = 12;
uint64_t const CODECAPI_AVEncVideoEncodeQP             = 13;
uint64_t const CODECAPI_AVEncVideoForceKeyFrame        = 14;
uint64_t const CODECAPI_AVEncVideoMinQP                = 15;
uint64_t const CODECAPI_AVLowLatencyMode               = 16;
uint64_t const CODECAPI_AVEncVideoMaxQP                = 17;
uint64_t const CODECAPI_VideoEncoderDisplayContentType = 18;
uint64_t const HL2SSAPI_VideoMediaIndex                = 0xFFFFFFFFFFFFFFFB;
uint64_t const HL2SSAPI_VideoStrideMask                = 0xFFFFFFFFFFFFFFFC;
uint64_t const HL2SSAPI_AcquisitionMode                = 0xFFFFFFFFFFFFFFFD;
uint64_t const HL2SSAPI_VLCHostTicksOffsetConstant     = 0xFFFFFFFFFFFFFFFE;
uint64_t const HL2SSAPI_VLCHostTicksOffsetExposure     = 0xFFFFFFFFFFFFFFFF;
}

namespace hologram_perspective
{
uint32_t const DISPLAY = 0;
uint32_t const PV      = 1;
}

namespace mixer_mode
{
uint32_t const MICROPHONE = 0;
uint32_t const SYSTEM     = 1;
uint32_t const BOTH       = 2;
uint32_t const QUERY      = 3;
}

namespace pv_decoded_format
{
uint8_t const BGR  = 0;
uint8_t const RGB  = 1;
uint8_t const BGRA = 2;
uint8_t const RGBA = 3;
uint8_t const GRAY = 4;
}

namespace eet_framerate
{
uint8_t const FPS_30 = 30;
uint8_t const FPS_60 = 60;
uint8_t const FPS_90 = 90;
}

namespace parameters_rm_vlc
{
uint16_t const WIDTH  = 640;
uint16_t const HEIGHT = 480;
uint8_t  const FPS    = 30;
uint32_t const PIXELS = WIDTH * HEIGHT;
}

namespace parameters_rm_depth_ahat
{
uint16_t const WIDTH  = 512;
uint16_t const HEIGHT = 512;
uint8_t  const FPS    = 45;
uint32_t const PIXELS = WIDTH * HEIGHT;
}

namespace parameters_rm_depth_longthrow
{
uint16_t const WIDTH  = 320;
uint16_t const HEIGHT = 288;
uint8_t  const FPS    = 5;
uint32_t const PIXELS = WIDTH * HEIGHT;
}

namespace parameters_rm_imu_accelerometer
{
uint16_t const BATCH_SIZE = 93;
}

namespace parameters_rm_imu_gyroscope
{
uint16_t const BATCH_SIZE = 315;
}

namespace parameters_rm_imu_magnetometer
{
uint16_t const BATCH_SIZE = 11;
}

namespace parameters_microphone
{
uint8_t const ARRAY_CHANNELS     = 5;
uint8_t const ARRAY_TOP_LEFT     = 0;
uint8_t const ARRAY_TOP_CENTER   = 1;
uint8_t const ARRAY_TOP_RIGHT    = 2;
uint8_t const ARRAY_BOTTOM_LEFT  = 3;
uint8_t const ARRAY_BOTTOM_RIGHT = 4;

uint32_t const SAMPLE_RATE    = 48000;
uint8_t  const CHANNELS       = 2;
uint16_t const GROUP_SIZE_RAW = 768;
uint16_t const GROUP_SIZE_AAC = 1024;
}

namespace parameters_si
{
uint8_t const SAMPLE_RATE = 30;
}

namespace parameters_extended_audio
{
uint32_t const SAMPLE_RATE    = 48000;
uint8_t  const CHANNELS       = 2;
uint16_t const GROUP_SIZE_AAC = 1024;
}

namespace time_base
{
uint64_t const HUNDREDS_OF_NANOSECONDS = 10000000ULL;
}

//------------------------------------------------------------------------------
// Geometry
//------------------------------------------------------------------------------

struct vector_2
{
    float x;
    float y;
};

struct vector_3
{
    float x;
    float y;
    float z;
};

struct vector_4
{
    float x;
    float y;
    float z;
    float w;
};

typedef vector_4 quaternion;
typedef vector_4 plane;

struct matrix_4x4
{
    float m[4][4];
};

struct ray
{
    vector_3 origin;
    vector_3 direction;
};

struct uint64x2
{
    uint64_t val[2];
};

//------------------------------------------------------------------------------
// Packer
//------------------------------------------------------------------------------

union v8  {                           uint8_t  b; int8_t  c; };
union v16 { struct { v8  b0, b1; } b; uint16_t w; int16_t s; };
union v32 { struct { v16 w0, w1; } w; uint32_t d; int32_t i; };
union v64 { struct { v32 d0, d1; } d; uint64_t q; int64_t l; };

HL2SS_INLINE
void push_u8(std::vector<uint8_t>& sc, uint8_t byte)
{
    sc.push_back(byte);
}

HL2SS_INLINE
void push_u16(std::vector<uint8_t>& sc, uint16_t word)
{
    v16 data;
    data.w = word;

    push_u8(sc, data.b.b0.b);
    push_u8(sc, data.b.b1.b);
}

HL2SS_INLINE
void push_u32(std::vector<uint8_t>& sc, uint32_t dword)
{
    v32 data;
    data.d = dword;

    push_u16(sc, data.w.w0.w);
    push_u16(sc, data.w.w1.w);
}

HL2SS_INLINE
void push_u64(std::vector<uint8_t>& sc, uint64_t qword)
{
    v64 data;
    data.q = qword;

    push_u32(sc, data.d.d0.d);
    push_u32(sc, data.d.d1.d);
}

HL2SS_INLINE
void push_float(std::vector<uint8_t>& sc, float f)
{
    push_u32(sc, *(uint32_t*)&f);
}

HL2SS_INLINE
void push_double(std::vector<uint8_t>& sc, double d)
{
    push_u64(sc, *(uint64_t*)&d);
}

HL2SS_INLINE
void push(std::vector<uint8_t>& sc, void const* data, uint64_t size)
{
    sc.insert(sc.end(), (uint8_t*)data, ((uint8_t*)data) + size);
}

//------------------------------------------------------------------------------
// Stream Configuration
//------------------------------------------------------------------------------

constexpr
uint32_t extended_audio_device_mixer_mode(uint32_t mixer_mode, uint32_t device)
{
    uint32_t const DEVICE_BASE = 0x00000004;
    return mixer_mode | (DEVICE_BASE * (device + 1));
}

//------------------------------------------------------------------------------
// Decoders
//------------------------------------------------------------------------------

struct rm_vlc_metadata
{
    uint64_t sensor_ticks;
    uint64_t exposure;
    uint32_t gain;
    uint32_t _reserved;
};

struct rm_depth_ahat_metadata
{
    uint64_t sensor_ticks;
};

struct rm_depth_longthrow_metadata
{
    uint64_t sensor_ticks;
};

struct pv_metadata
{
    vector_2 f;
    vector_2 c;
    uint64_t exposure_time;
    uint64x2 exposure_compensation;
    uint32_t lens_position;
    uint32_t focus_state;
    uint32_t iso_speed;
    uint32_t white_balance;
    vector_2 iso_gains;
    vector_3 white_balance_gains;
    uint16_t width;
    uint16_t height;
};

struct extended_depth_metadata
{
    uint16_t width;
    uint16_t height;
};

//------------------------------------------------------------------------------
// Mode 2 Data Acquisition
//------------------------------------------------------------------------------

struct calibration_rm_vlc
{
    float uv2xy[2][parameters_rm_vlc::HEIGHT][parameters_rm_vlc::WIDTH];
    float extrinsics[4][4];
    float undistort_map[2][parameters_rm_vlc::HEIGHT][parameters_rm_vlc::WIDTH];
    float intrinsics[4];
};

struct calibration_rm_depth_ahat
{
    float uv2xy[2][parameters_rm_depth_ahat::HEIGHT][parameters_rm_depth_ahat::WIDTH];
    float extrinsics[4][4];
    float scale;
    float alias;
    float undistort_map[2][parameters_rm_depth_ahat::HEIGHT][parameters_rm_depth_ahat::WIDTH];
    float intrinsics[4];
};

struct calibration_rm_depth_longthrow
{
    float uv2xy[2][parameters_rm_depth_longthrow::HEIGHT][parameters_rm_depth_longthrow::WIDTH];
    float extrinsics[4][4];
    float scale;
    float undistort_map[2][parameters_rm_depth_longthrow::HEIGHT][parameters_rm_depth_longthrow::WIDTH];
    float intrinsics[4];
};

struct calibration_rm_imu
{
    float extrinsics[4][4];
};

struct calibration_pv
{
    float focal_length[2];
    float principal_point[2];
    float radial_distortion[3];
    float tangential_distortion[2];
    float projection[4][4];
    float extrinsics[4][4];
    float intrinsics_mf[4];
    float extrinsics_mf[7];
};

//------------------------------------------------------------------------------
// Port Information
//------------------------------------------------------------------------------

constexpr
char const* get_port_name(uint16_t port)
{
    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:     return "rm_vlc_leftfront";
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:      return "rm_vlc_leftleft";
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:    return "rm_vlc_rightfront";
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    return "rm_vlc_rightright";
    case hl2ss::stream_port::RM_DEPTH_AHAT:        return "rm_depth_ahat";
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   return "rm_depth_longthrow";
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER: return "rm_imu_accelerometer";
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:     return "rm_imu_gyroscope";
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  return "rm_imu_magnetometer";
    case hl2ss::ipc_port::REMOTE_CONFIGURATION:    return "remote_configuration";
    case hl2ss::stream_port::PERSONAL_VIDEO:       return "personal_video";
    case hl2ss::stream_port::MICROPHONE:           return "microphone";
    case hl2ss::stream_port::SPATIAL_INPUT:        return "spatial_input";
    case hl2ss::ipc_port::SPATIAL_MAPPING:         return "spatial_mapping";
    case hl2ss::ipc_port::SCENE_UNDERSTANDING:     return "scene_understanding";
    case hl2ss::ipc_port::VOICE_INPUT:             return "voice_input";
    case hl2ss::ipc_port::UNITY_MESSAGE_QUEUE:     return "unity_message_queue";
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: return "extended_eye_tracker";
    case hl2ss::stream_port::EXTENDED_AUDIO:       return "extended_audio";
    case hl2ss::stream_port::EXTENDED_VIDEO:       return "extended_video";
    case hl2ss::ipc_port::GUEST_MESSAGE_QUEUE:     return "guest_message_queue";
    case hl2ss::stream_port::EXTENDED_DEPTH:       return "extended_depth";
    default:                                       return nullptr;
    }
}

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

namespace hs_marker_state
{
uint32_t const Disable = 0;
uint32_t const Enable  = 1;
}

namespace pv_focus_mode
{
uint32_t const Auto       = 0;
uint32_t const Single     = 1;
uint32_t const Continuous = 2;
uint32_t const Manual     = 3;
}

namespace pv_auto_focus_range
{
uint32_t const FullRange = 0;
uint32_t const Macro     = 1;
uint32_t const Normal    = 2;
}

namespace pv_manual_focus_distance
{
uint32_t const Infinity = 0;
uint32_t const Nearest  = 2;
}

namespace pv_focus_value
{
uint32_t const Min =   170;
uint32_t const Max = 10000;
}

namespace pv_driver_fallback
{
uint32_t const Enable  = 0;
uint32_t const Disable = 1;
}

namespace pv_video_temporal_denoising_mode
{
uint32_t const Off = 0;
uint32_t const On  = 1;
}

namespace pv_color_temperature_preset
{
uint32_t const Auto        = 0;
uint32_t const Manual      = 1;
uint32_t const Cloudy      = 2;
uint32_t const Daylight    = 3;
uint32_t const Flash       = 4;
uint32_t const Fluorescent = 5;
uint32_t const Tungsten    = 6;
uint32_t const Candlelight = 7;
}

namespace pv_white_balance_value
{
uint32_t const Min = 2300; // 25
uint32_t const Max = 7500; // 25
}

namespace pv_exposure_mode
{
uint32_t const Manual = 0;
uint32_t const Auto   = 1;
}

namespace pv_exposure_value
{
uint32_t const Min =   1000; // 10
uint32_t const Max = 660000; // 10
}

namespace pv_exposure_priority_video
{
uint32_t const Disabled = 0;
uint32_t const Enabled  = 1;
}

namespace pv_iso_speed_mode
{
uint32_t const Manual = 0;
uint32_t const Auto   = 1;
}

namespace pv_iso_speed_value
{
uint32_t const Min =  100;
uint32_t const Max = 3200;
}

namespace pv_backlight_compensation_state
{
uint32_t const Disable = 0;
uint32_t const Enable  = 1;
}

namespace pv_capture_scene_mode
{
uint32_t const Auto          =  0;
uint32_t const Macro         =  2;
uint32_t const Portrait      =  3;
uint32_t const Sport         =  4;
uint32_t const Snow          =  5;
uint32_t const Night         =  6;
uint32_t const Beach         =  7;
uint32_t const Sunset        =  8;
uint32_t const Candlelight   =  9;
uint32_t const Landscape     = 10;
uint32_t const NightPortrait = 11;
uint32_t const Backlit       = 12;
}

namespace pv_media_capture_optimization
{
uint32_t const Default            = 0;
uint32_t const Quality            = 1;
uint32_t const Latency            = 2;
uint32_t const Power              = 3;
uint32_t const LatencyThenQuality = 4;
uint32_t const LatencyThenPower   = 5;
uint32_t const PowerAndQuality    = 6;
}

namespace pv_capture_use
{
uint32_t const NotSet = 0;
uint32_t const Photo  = 1;
uint32_t const Video  = 2;
}

namespace pv_optical_image_stabilization_mode
{
uint32_t const Off = 0;
uint32_t const On  = 1;
}

namespace pv_hdr_video_mode
{
uint32_t const Off  = 0;
uint32_t const On   = 1;
uint32_t const Auto = 2;
}

namespace pv_region_of_interest_type
{
uint32_t const Unknown = 0;
uint32_t const Face    = 1;
}

namespace interface_priority
{
int32_t const LOWEST       = -2;
int32_t const BELOW_NORMAL = -1;
int32_t const NORMAL       = 0;
int32_t const ABOVE_NORMAL = 1;
int32_t const HIGHEST      = 2;
}

struct version
{
    uint16_t field[4];
};

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

namespace sm_vertex_position_format
{
uint32_t const R32G32B32A32Float         =  2;
uint32_t const R16G16B16A16IntNormalized = 13;
}

namespace sm_triangle_index_format
{
uint32_t const R16UInt = 57;
uint32_t const R32Uint = 42;
}

namespace sm_vertex_normal_format
{
uint32_t const R32G32B32A32Float     =  2;
uint32_t const R8G8B8A8IntNormalized = 31;
}

namespace sm_volume_type
{
uint32_t const Box         = 0;
uint32_t const Frustum     = 1;
uint32_t const OrientedBox = 2;
uint32_t const Sphere      = 3;
}

struct sm_box
{
    vector_3 center;
    vector_3 extents;
};

struct sm_frustum
{
    plane p_near;
    plane p_far;
    plane p_right;
    plane p_left;
    plane p_top;
    plane p_bottom;
};

struct sm_oriented_box
{
    vector_3 center;
    vector_3 extents;
    quaternion orientation;
};

struct sm_sphere
{
    vector_3 center;
    float radius;
};

struct guid
{
    uint64_t l;
    uint64_t h;
};

struct sm_surface_info
{
    guid id;
    uint64_t update_time;
};

class sm_bounding_volume
{
private:
    std::vector<uint8_t> m_data;
    uint32_t m_count;

public:
    sm_bounding_volume()
    {
        m_count = 0;
    }

    sm_bounding_volume(uint32_t count, uint8_t const* data, uint64_t size)
    {
        m_count = count;
        m_data  = { data, data + size };
    }

    void clear()
    {
        m_count = 0;
        m_data.clear();
    }

    void add_box(sm_box box)
    {
        m_count++;
        push_u32(m_data, sm_volume_type::Box);
        push(m_data, &box,  sizeof(box));
    }

    void add_frustum(sm_frustum frustum)
    {
        m_count++;
        push_u32(m_data, sm_volume_type::Frustum);
        push(m_data, &frustum, sizeof(frustum));
    }

    void add_oriented_box(sm_oriented_box oriented_box)
    {
        m_count++;
        push_u32(m_data, sm_volume_type::OrientedBox);
        push(m_data, &oriented_box, sizeof(oriented_box));
    }

    void add_sphere(sm_sphere sphere)
    {
        m_count++;
        push_u32(m_data, sm_volume_type::Sphere);
        push(m_data, &sphere, sizeof(sphere));
    }

    uint32_t get_count() const
    {
        return m_count;
    }

    uint8_t const* get_data() const
    {
        return m_data.data();
    }

    uint64_t get_size() const
    {
        return m_data.size();
    }
};

class sm_mesh_task
{
private:
    std::vector<uint8_t> m_data;
    uint32_t m_count;
    
public:
    sm_mesh_task()
    {
        m_count = 0;
    }

    sm_mesh_task(uint32_t count, uint8_t const* data, uint64_t size)
    {
        m_count = count;
        m_data  = { data, data + size };
    }

    void clear()
    {
        m_count = 0;
        m_data.clear();        
    }

    void add_task(guid id, double max_triangles_per_cubic_meter, uint32_t vertex_position_format, uint32_t triangle_index_format, uint32_t vertex_normal_format, bool include_vertex_normals, bool include_bounds)
    {
        m_count++;
        push_u64(m_data, id.l);
        push_u64(m_data, id.h);
        push_double(m_data, max_triangles_per_cubic_meter);
        push_u32(m_data, vertex_position_format);
        push_u32(m_data, triangle_index_format);
        push_u32(m_data, vertex_normal_format);
        push_u32(m_data, (1*include_vertex_normals) | (2*include_bounds));
    }

    uint32_t get_count() const
    {
        return m_count;
    }

    uint8_t const* get_data() const
    {
        return m_data.data();
    }

    uint64_t get_size() const
    {
        return m_data.size();
    }
};

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

namespace su_mesh_lod
{
uint32_t const Coarse    =   0;
uint32_t const Medium    =   1;
uint32_t const Fine      =   2;
uint32_t const Unlimited = 255;
}

namespace su_kind_flag
{
uint8_t const Background         =   1;
uint8_t const Wall               =   2;
uint8_t const Floor              =   4;
uint8_t const Ceiling            =   8;
uint8_t const Platform           =  16;
uint8_t const Unknown            =  32;
uint8_t const World              =  64;
uint8_t const CompletelyInferred = 128;
}

namespace su_create
{
uint8_t const New             = 0;
uint8_t const NewFromPrevious = 1;
}

namespace su_kind
{
int32_t const Background         =   0;
int32_t const Wall               =   1;
int32_t const Floor              =   2;
int32_t const Ceiling            =   3;
int32_t const Platform           =   4;
int32_t const Unknown            = 247;
int32_t const World              = 248;
int32_t const CompletelyInferred = 249;
}

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
    std::vector<guid> guid_list;
};

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

namespace vi_speech_recognition_confidence
{
uint32_t const High     = 0;
uint32_t const Medium   = 1;
uint32_t const Low      = 2;
uint32_t const Rejected = 3;
}

struct vi_result
{
    uint32_t index;
    uint32_t confidence;
    uint64_t phrase_duration;
    uint64_t phrase_start_time;
    double   raw_confidence;
};

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

class umq_command_buffer
{
private:
    std::vector<uint8_t> m_buffer;
    uint32_t m_count;

public:
    umq_command_buffer()
    {
        m_count = 0;
    }

    umq_command_buffer(uint32_t count, uint8_t const* data, uint64_t size)
    {
        m_count = count;
        m_buffer  = { data, data + size };
    }

    void clear()
    {
        m_count = 0;
        m_buffer.clear();
    }

    void add(uint32_t id, void const* data, uint64_t size)
    {
        push_u32(m_buffer, id);
        push_u32(m_buffer, (uint32_t)size);
        push(m_buffer, data, size);
        m_count++;
    }

    uint32_t get_count()
    {
        return m_count;
    }

    uint8_t const* get_data()
    {
        return m_buffer.data();
    }

    uint64_t get_size()
    {
        return m_buffer.size();
    }
};

//------------------------------------------------------------------------------
// Unpacking
//------------------------------------------------------------------------------

struct rm_imu_sample
{
    uint64_t sensor_timestamp;
    uint64_t timestamp;
    float x;
    float y;
    float z;
    float temperature;
};

namespace pv_focus_state
{
uint32_t const UNINITIALIZED = 0;
uint32_t const LOST          = 1;
uint32_t const SEARCHING     = 2;
uint32_t const FOCUSED       = 3;
uint32_t const FAILED        = 4;
}

namespace si_valid
{
uint32_t const HEAD  = 0x01;
uint32_t const EYE   = 0x02;
uint32_t const LEFT  = 0x04;
uint32_t const RIGHT = 0x08;
}

struct si_head_pose
{
    vector_3 position;
    vector_3 forward;
    vector_3 up;
};

namespace si_hand_joint_kind
{
uint8_t const Palm               =  0;
uint8_t const Wrist              =  1;
uint8_t const ThumbMetacarpal    =  2;
uint8_t const ThumbProximal      =  3;
uint8_t const ThumbDistal        =  4;
uint8_t const ThumbTip           =  5;
uint8_t const IndexMetacarpal    =  6;
uint8_t const IndexProximal      =  7;
uint8_t const IndexIntermediate  =  8;
uint8_t const IndexDistal        =  9;
uint8_t const IndexTip           = 10;
uint8_t const MiddleMetacarpal   = 11;
uint8_t const MiddleProximal     = 12;
uint8_t const MiddleIntermediate = 13;
uint8_t const MiddleDistal       = 14;
uint8_t const MiddleTip          = 15;
uint8_t const RingMetacarpal     = 16;
uint8_t const RingProximal       = 17;
uint8_t const RingIntermediate   = 18;
uint8_t const RingDistal         = 19;
uint8_t const RingTip            = 20;
uint8_t const LittleMetacarpal   = 21;
uint8_t const LittleProximal     = 22;
uint8_t const LittleIntermediate = 23;
uint8_t const LittleDistal       = 24;
uint8_t const LittleTip          = 25;
uint8_t const TOTAL              = 26;
}

struct si_hand_joint
{
    quaternion orientation;
    vector_3 position;
    float radius;
    int32_t accuracy;
};

struct si_frame
{
    uint32_t valid;
    si_head_pose head_pose;
    ray eye_ray;
    si_hand_joint left_hand[26];
    si_hand_joint right_hand[26];
};

namespace eet_valid
{
uint32_t const CALIBRATION       = 0x01;
uint32_t const COMBINED_RAY      = 0x02;
uint32_t const LEFT_RAY          = 0x04;
uint32_t const RIGHT_RAY         = 0x08;
uint32_t const LEFT_OPENNESS     = 0x10;
uint32_t const RIGHT_OPENNESS    = 0x20;
uint32_t const VERGENCE_DISTANCE = 0x40;
}

struct eet_frame
{
    uint32_t _reserved;
    ray combined_ray;
    ray left_ray;
    ray right_ray;
    float left_openness;
    float right_openness;
    float vergence_distance;
    uint32_t valid;
};

struct map_rm_vlc
{
    uint8_t* image;
    rm_vlc_metadata* metadata;
};

struct map_rm_depth_ahat
{
    uint16_t* depth;
    uint16_t* ab;
    rm_depth_ahat_metadata* metadata;
};

struct map_rm_depth_longthrow
{
    uint16_t* depth;
    uint16_t* ab;
    rm_depth_longthrow_metadata* metadata;
};

struct map_rm_imu
{
    hl2ss::rm_imu_sample* samples;
};

struct map_pv
{
    uint8_t* image;
    pv_metadata* metadata;
};

struct map_microphone_raw
{
    int16_t* samples;
};

struct map_microphone_aac
{
    float* samples;
};

struct map_microphone_array
{
    float* samples;
};

struct map_si
{
    hl2ss::si_frame* tracking;
};

struct map_eet
{
    hl2ss::eet_frame* tracking;
};

struct map_extended_audio_raw
{
    int16_t* samples;
};

struct map_extended_audio_aac
{
    float* samples;
};

struct map_extended_depth
{
    uint16_t* depth;
    extended_depth_metadata* metadata;
};

HL2SS_INLINE
map_rm_vlc unpack_rm_vlc(uint8_t* payload)
{
    return { payload, (rm_vlc_metadata*)(payload + parameters_rm_vlc::PIXELS) };
}

HL2SS_INLINE
map_rm_depth_ahat unpack_rm_depth_ahat(uint8_t* payload)
{
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t))), (rm_depth_ahat_metadata*)(payload + (2 * parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t))) };
}

HL2SS_INLINE
map_rm_depth_longthrow unpack_rm_depth_longthrow(uint8_t* payload)
{
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t))), (rm_depth_longthrow_metadata*)(payload + (2 * parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t))) };
}

HL2SS_INLINE
map_rm_imu unpack_rm_imu(uint8_t* payload)
{
    return { (rm_imu_sample*)payload };
}

HL2SS_INLINE
map_pv unpack_pv(uint8_t* payload, uint32_t size)
{
    return { payload, (pv_metadata*)(payload + size - sizeof(pv_metadata)) };
}

HL2SS_INLINE
map_microphone_raw unpack_microphone_raw(uint8_t* payload)
{
    return { (int16_t*)payload };
}

HL2SS_INLINE
map_microphone_aac unpack_microphone_aac(uint8_t* payload)
{
    return { (float*)payload };
}

HL2SS_INLINE
map_microphone_array unpack_microphone_array(uint8_t* payload)
{
    return { (float*)payload };
}

HL2SS_INLINE
map_si unpack_si(uint8_t* payload)
{
    return { (si_frame*)payload };
}

HL2SS_INLINE
map_eet unpack_eet(uint8_t* payload)
{
    return { (eet_frame*)payload };
}

HL2SS_INLINE
map_extended_audio_raw unpack_extended_audio_raw(uint8_t* payload)
{
    return { (int16_t*)payload };
}

HL2SS_INLINE
map_extended_audio_aac unpack_extended_audio_aac(uint8_t* payload)
{
    return { (float*)payload };
}

HL2SS_INLINE
map_extended_depth unpack_extended_depth(uint8_t* payload, uint32_t size)
{
    return { (uint16_t*)payload, (extended_depth_metadata*)(payload + size - sizeof(extended_depth_metadata)) };
}

}

//******************************************************************************
// MultiThreaded Module
//******************************************************************************

namespace hl2ss
{
namespace mt
{
namespace time_preference
{
int32_t const PREFER_PAST    = -1;
int32_t const PREFER_NEAREST =  0;
int32_t const PREFER_FUTURE  =  1;
}
}
}

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

HL2SS_CLIENT_IMPORT
int32_t HL2SS_CALL get_pv_dimensions(void* source, uint16_t& width, uint16_t& height);

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
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_rm_vlc{ hl2ss::unpack_rm_vlc(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_depth_ahat> : public hl2ss::map_rm_depth_ahat
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_rm_depth_ahat{ hl2ss::unpack_rm_depth_ahat(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_depth_longthrow> : public hl2ss::map_rm_depth_longthrow
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_rm_depth_longthrow{ hl2ss::unpack_rm_depth_longthrow(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_rm_imu> : public hl2ss::map_rm_imu
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_rm_imu{ hl2ss::unpack_rm_imu(payload) }
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

template<>
class payload_map<hl2ss::map_microphone_raw> : public hl2ss::map_microphone_raw
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_microphone_raw{ hl2ss::unpack_microphone_raw(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_microphone_aac> : public hl2ss::map_microphone_aac
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_microphone_aac{ hl2ss::unpack_microphone_aac(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_microphone_array> : public hl2ss::map_microphone_array
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_microphone_array{ hl2ss::unpack_microphone_array(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_si> : public hl2ss::map_si
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_si{ hl2ss::unpack_si(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_eet> : public hl2ss::map_eet
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_eet{ hl2ss::unpack_eet(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_extended_audio_raw> : public hl2ss::map_extended_audio_raw
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_extended_audio_raw{ hl2ss::unpack_extended_audio_raw(payload) }
    {
    }
};

template<>
class payload_map<hl2ss::map_extended_audio_aac> : public hl2ss::map_extended_audio_aac
{
public:
    payload_map(uint8_t* payload, uint32_t) : hl2ss::map_extended_audio_aac{ hl2ss::unpack_extended_audio_aac(payload) }
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

    void get_pv_dimensions(uint16_t& width, uint16_t& height)
    {
        handle::check_result(hl2ss::ulm::get_pv_dimensions(m_handle, width, height));
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
    c.framerate = hl2ss::eet_framerate::FPS_30;

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
