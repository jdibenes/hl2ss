
#pragma once

#include <stdint.h>
#include <vector>
#include <memory>

#define HL2SS_INLINE inline

//******************************************************************************
// "Enumerations" and Structures
//******************************************************************************

namespace hl2ss
{
//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------

union v8  { char x;                   uint8_t  b; int8_t  c; };
union v16 { struct { v8  b0, b1; } b; uint16_t w; int16_t s; };
union v32 { struct { v16 w0, w1; } w; uint32_t d; int32_t i; };
union v64 { struct { v32 d0, d1; } d; uint64_t q; int64_t l; };

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
uint64_t const MICROPHONE           = 4096;
uint64_t const SPATIAL_INPUT        = 4096;
uint64_t const EXTENDED_EYE_TRACKER = 4096;
uint64_t const EXTENDED_AUDIO       = 4096;
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
uint8_t const ANY  = 254;
uint8_t const NONE = 255;
}

namespace media_category
{
uint8_t const Other          = 0;
uint8_t const Communications = 1;
uint8_t const Media          = 2;
uint8_t const GameChat       = 3;
uint8_t const Speech         = 4;
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
uint8_t const CHANNELS = 2;

uint8_t const ARRAY_CHANNELS     = 5;
uint8_t const ARRAY_TOP_LEFT     = 0;
uint8_t const ARRAY_TOP_CENTER   = 1;
uint8_t const ARRAY_TOP_RIGHT    = 2;
uint8_t const ARRAY_BOTTOM_LEFT  = 3;
uint8_t const ARRAY_BOTTOM_RIGHT = 4;

uint32_t const SAMPLE_RATE    = 48000;
uint16_t const GROUP_SIZE_RAW = 768;
uint16_t const GROUP_SIZE_AAC = 1024;
}

namespace parameters_si
{
uint8_t const SAMPLE_RATE = 30;
}

namespace time_base
{
uint64_t const HUNDREDS_OF_NANOSECONDS = 10000000ULL;
int64_t  const FILETIME_EPOCH          = 0LL;
int64_t  const UNIX_EPOCH              = 116444736000000000LL;
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
// Decoder RM VLC
//------------------------------------------------------------------------------

struct rm_vlc_metadata
{
    uint64_t sensor_ticks;
    uint64_t exposure;
    uint32_t gain;
    uint32_t _reserved;
};

struct map_rm_vlc
{
    uint8_t* image;
    rm_vlc_metadata* metadata;
};

//------------------------------------------------------------------------------
// Decoder RM Depth
//------------------------------------------------------------------------------

struct rm_depth_ahat_metadata
{
    uint64_t sensor_ticks;
};

struct rm_depth_longthrow_metadata
{
    uint64_t sensor_ticks;
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

//------------------------------------------------------------------------------
// Decoder RM IMU
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

struct map_rm_imu
{
    rm_imu_sample* samples;
    uint32_t count;
};

//------------------------------------------------------------------------------
// Decoder PV
//------------------------------------------------------------------------------

namespace pv_focus_state
{
uint32_t const UNINITIALIZED = 0;
uint32_t const LOST          = 1;
uint32_t const SEARCHING     = 2;
uint32_t const FOCUSED       = 3;
uint32_t const FAILED        = 4;
}

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

struct map_pv
{
    uint8_t* image;
    pv_metadata* metadata;
};

//------------------------------------------------------------------------------
// Decoder Microphone
//------------------------------------------------------------------------------

template <typename T>
struct map_microphone
{
    T* samples;
    uint32_t count;
};

//------------------------------------------------------------------------------
// Decoder SI
//------------------------------------------------------------------------------

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
    si_hand_joint left_hand[si_hand_joint_kind::TOTAL];
    si_hand_joint right_hand[si_hand_joint_kind::TOTAL];
};

struct map_si
{
    si_frame* tracking;
};

//------------------------------------------------------------------------------
// Decoder EET
//------------------------------------------------------------------------------

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

struct map_eet
{
    eet_frame* tracking;
};

//------------------------------------------------------------------------------
// Decoder Extended Depth
//------------------------------------------------------------------------------

struct extended_depth_metadata
{
    uint16_t width;
    uint16_t height;
};

struct map_extended_depth
{
    uint16_t* depth;
    extended_depth_metadata* metadata;
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
uint32_t const Min = 2300 / 25;
uint32_t const Max = 7500 / 25;
}

namespace pv_exposure_mode
{
uint32_t const Manual = 0;
uint32_t const Auto   = 1;
}

namespace pv_exposure_value
{
uint32_t const Min =   1000 / 10;
uint32_t const Max = 660000 / 10;
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

namespace pv_region_of_interest_weight
{
uint32_t const Min = 0;
uint32_t const Max = 100;
}

namespace pv_region_of_interest_type
{
uint32_t const Unknown = 0;
uint32_t const Face    = 1;
}

namespace ee_interface_priority
{
int32_t const LOWEST       = -2;
int32_t const BELOW_NORMAL = -1;
int32_t const NORMAL       = 0;
int32_t const ABOVE_NORMAL = 1;
int32_t const HIGHEST      = 2;
}

namespace rm_map_camera_point_operation
{
uint32_t const ImagePointToCameraUnitPlane = 0;
uint32_t const CameraSpaceToImagePoint     = 1;
}

namespace ts_source
{
uint32_t const QPC = 0;
uint32_t const UTC = 1;
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
}

//******************************************************************************
// Inlines
//******************************************************************************

namespace hl2ss
{
//------------------------------------------------------------------------------
// Packer
//------------------------------------------------------------------------------

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
// Geometry
//------------------------------------------------------------------------------

HL2SS_INLINE
bool is_valid_pose(matrix_4x4 const& pose)
{
    return pose.m[3][3] != 0.0f;
}

//------------------------------------------------------------------------------
// Stream Configuration
//------------------------------------------------------------------------------

HL2SS_INLINE
uint32_t extended_audio_device_mixer_mode(uint32_t mixer_mode, uint32_t device_index, uint32_t source_index, uint32_t format_index)
{
    return mixer_mode | (((device_index + 1) & 0x3FF) << 2) | ((source_index & 0x3FF) << 12) | ((format_index & 0x3FF) << 22);
}

HL2SS_INLINE
uint8_t extended_audio_raw_configuration(uint8_t media_category, bool shared, bool audio_raw, bool disable_effect, bool enable_passthrough)
{
    uint8_t b0_2 = media_category & 7;
    uint8_t b3_3 = shared;
    uint8_t b4_4 = 0;
    uint8_t b5_5 = audio_raw;
    uint8_t b6_6 = disable_effect;
    uint8_t b7_7 = enable_passthrough;

    return (b7_7 << 7) | (b6_6 << 6) | (b5_5 << 5) | (b4_4 << 4) | (b3_3 << 3) | b0_2;
}

//------------------------------------------------------------------------------
// RM IMU
//------------------------------------------------------------------------------

HL2SS_INLINE
uint16_t rm_imu_get_batch_size(uint16_t port)
{
    switch (port)
    {
    case stream_port::RM_IMU_ACCELEROMETER: return parameters_rm_imu_accelerometer::BATCH_SIZE;
    case stream_port::RM_IMU_GYROSCOPE:     return parameters_rm_imu_gyroscope::BATCH_SIZE;
    case stream_port::RM_IMU_MAGNETOMETER:  return parameters_rm_imu_magnetometer::BATCH_SIZE;
    default:                                return 0;
    }
}

HL2SS_INLINE
void rm_imu_fix_soc_ticks(map_rm_imu const& map)
{
    for (uint32_t i = 0; i < map.count; ++i) { map.samples[i].timestamp += (map.samples[i].sensor_timestamp - map.samples[0].sensor_timestamp) / 100; }
}

//------------------------------------------------------------------------------
// PV
//------------------------------------------------------------------------------

HL2SS_INLINE
uint16_t pv_get_video_stride(uint16_t width)
{
    return (width + 63) & ~63;
}

//------------------------------------------------------------------------------
// Microphone
//------------------------------------------------------------------------------

template <typename T>
std::unique_ptr<T[]> microphone_planar_to_packed(map_microphone<T> const& map, uint32_t channels)
{
    std::unique_ptr<T[]> data = std::make_unique<T[]>(map.count);
    uint32_t frames = map.count / channels;
    for (uint32_t i = 0; i < frames; ++i) { for (uint32_t j = 0; j < channels; ++j) { data[(i*channels) + j] = map.samples[(j*frames) + i]; } }
    return data;
}

template <typename T>
std::unique_ptr<T[]> microphone_packed_to_planar(map_microphone<T> const& map, uint32_t channels)
{
    std::unique_ptr<T[]> data = std::make_unique<T[]>(map.count);
    uint32_t frames = map.count / channels;
    for (uint32_t i = 0; i < channels; ++i) { for (uint32_t j = 0; j < frames; ++j) { data[(i*frames) + j] = map.samples[(j*channels) + i]; } }
    return data;
}

//------------------------------------------------------------------------------
// SI
//------------------------------------------------------------------------------

HL2SS_INLINE
char const* si_get_joint_name(uint8_t joint_kind)
{
    switch (joint_kind)
    {
    case si_hand_joint_kind::Palm:               return "Palm";
    case si_hand_joint_kind::Wrist:              return "Wrist";
    case si_hand_joint_kind::ThumbMetacarpal:    return "ThumbMetacarpal";
    case si_hand_joint_kind::ThumbProximal:      return "ThumbProximal";
    case si_hand_joint_kind::ThumbDistal:        return "ThumbDistal";
    case si_hand_joint_kind::ThumbTip:           return "ThumbTip";
    case si_hand_joint_kind::IndexMetacarpal:    return "IndexMetacarpal";
    case si_hand_joint_kind::IndexProximal:      return "IndexProximal";
    case si_hand_joint_kind::IndexIntermediate:  return "IndexIntermediate";
    case si_hand_joint_kind::IndexDistal:        return "IndexDistal";
    case si_hand_joint_kind::IndexTip:           return "IndexTip";
    case si_hand_joint_kind::MiddleMetacarpal:   return "MiddleMetacarpal";
    case si_hand_joint_kind::MiddleProximal:     return "MiddleProximal";
    case si_hand_joint_kind::MiddleIntermediate: return "MiddleIntermediate";
    case si_hand_joint_kind::MiddleDistal:       return "MiddleDistal";
    case si_hand_joint_kind::MiddleTip:          return "MiddleTip";
    case si_hand_joint_kind::RingMetacarpal:     return "RingMetacarpal";
    case si_hand_joint_kind::RingProximal:       return "RingProximal";
    case si_hand_joint_kind::RingIntermediate:   return "RingIntermediate";
    case si_hand_joint_kind::RingDistal:         return "RingDistal";
    case si_hand_joint_kind::RingTip:            return "RingTip";
    case si_hand_joint_kind::LittleMetacarpal:   return "LittleMetacarpal";
    case si_hand_joint_kind::LittleProximal:     return "LittleProximal";
    case si_hand_joint_kind::LittleIntermediate: return "LittleIntermediate";
    case si_hand_joint_kind::LittleDistal:       return "LittleDistal";
    case si_hand_joint_kind::LittleTip:          return "LittleTip";
    default:                                     return nullptr;
    }
}

//------------------------------------------------------------------------------
// Port Information
//------------------------------------------------------------------------------

HL2SS_INLINE
char const* get_port_name(uint16_t port)
{
    switch (port)
    {
    case stream_port::RM_VLC_LEFTFRONT:     return "rm_vlc_leftfront";
    case stream_port::RM_VLC_LEFTLEFT:      return "rm_vlc_leftleft";
    case stream_port::RM_VLC_RIGHTFRONT:    return "rm_vlc_rightfront";
    case stream_port::RM_VLC_RIGHTRIGHT:    return "rm_vlc_rightright";
    case stream_port::RM_DEPTH_AHAT:        return "rm_depth_ahat";
    case stream_port::RM_DEPTH_LONGTHROW:   return "rm_depth_longthrow";
    case stream_port::RM_IMU_ACCELEROMETER: return "rm_imu_accelerometer";
    case stream_port::RM_IMU_GYROSCOPE:     return "rm_imu_gyroscope";
    case stream_port::RM_IMU_MAGNETOMETER:  return "rm_imu_magnetometer";
    case ipc_port::REMOTE_CONFIGURATION:    return "remote_configuration";
    case stream_port::PERSONAL_VIDEO:       return "personal_video";
    case stream_port::MICROPHONE:           return "microphone";
    case stream_port::SPATIAL_INPUT:        return "spatial_input";
    case ipc_port::SPATIAL_MAPPING:         return "spatial_mapping";
    case ipc_port::SCENE_UNDERSTANDING:     return "scene_understanding";
    case ipc_port::VOICE_INPUT:             return "voice_input";
    case ipc_port::UNITY_MESSAGE_QUEUE:     return "unity_message_queue";
    case stream_port::EXTENDED_EYE_TRACKER: return "extended_eye_tracker";
    case stream_port::EXTENDED_AUDIO:       return "extended_audio";
    case stream_port::EXTENDED_VIDEO:       return "extended_video";
    case ipc_port::GUEST_MESSAGE_QUEUE:     return "guest_message_queue";
    case stream_port::EXTENDED_DEPTH:       return "extended_depth";
    default:                                return nullptr;
    }
}

//------------------------------------------------------------------------------
// Unpacking
//------------------------------------------------------------------------------

HL2SS_INLINE
map_rm_vlc unpack_rm_vlc(uint8_t* payload, uint32_t size)
{
    (void)size;
    return { payload, (rm_vlc_metadata*)(payload + size - sizeof(rm_vlc_metadata)) };
}

HL2SS_INLINE
map_rm_depth_ahat unpack_rm_depth_ahat(uint8_t* payload, uint32_t size)
{
    (void)size;
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t))), (rm_depth_ahat_metadata*)(payload + size - sizeof(rm_depth_ahat_metadata)) };
}

HL2SS_INLINE
map_rm_depth_longthrow unpack_rm_depth_longthrow(uint8_t* payload, uint32_t size)
{
    (void)size;
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t))), (rm_depth_longthrow_metadata*)(payload + size - sizeof(rm_depth_longthrow_metadata)) };
}

HL2SS_INLINE
map_rm_imu unpack_rm_imu(uint8_t* payload, uint32_t size)
{
    return { (rm_imu_sample*)payload, size / (uint32_t)sizeof(rm_imu_sample) };
}

HL2SS_INLINE
map_pv unpack_pv(uint8_t* payload, uint32_t size)
{
    return { payload, (pv_metadata*)(payload + size - sizeof(pv_metadata)) };
}

template <typename T>
map_microphone<T> unpack_microphone(uint8_t* payload, uint32_t size)
{
    return { (T*)payload, size / (uint32_t)sizeof(T) };
}

HL2SS_INLINE
map_si unpack_si(uint8_t* payload, uint32_t size)
{
    (void)size;
    return { (si_frame*)payload };
}

HL2SS_INLINE
map_eet unpack_eet(uint8_t* payload, uint32_t size)
{
    (void)size;
    return { (eet_frame*)payload };
}

HL2SS_INLINE
map_extended_depth unpack_extended_depth(uint8_t* payload, uint32_t size)
{
    return { (uint16_t*)payload, (extended_depth_metadata*)(payload + size - sizeof(extended_depth_metadata)) };
}

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

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
        push(m_data, &box, sizeof(box));
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

    void add_task(guid id, double max_triangles_per_cubic_meter, uint32_t vertex_position_format, uint32_t triangle_index_format, uint32_t vertex_normal_format)
    {
        m_count++;
        push_u64(m_data, id.l);
        push_u64(m_data, id.h);
        push_double(m_data, max_triangles_per_cubic_meter);
        push_u32(m_data, vertex_position_format);
        push_u32(m_data, triangle_index_format);
        push_u32(m_data, vertex_normal_format);
        push_u32(m_data, 0);
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

    umq_command_buffer(uint32_t count, uint8_t const* data, uint32_t size)
    {
        m_count = count;
        m_buffer  = { data, data + size };
    }

    void clear()
    {
        m_count = 0;
        m_buffer.clear();
    }

    void add(uint32_t id, void const* data, uint32_t size)
    {
        push_u32(m_buffer, id);
        push_u32(m_buffer, size);
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

    uint32_t get_size()
    {
        return (uint32_t)m_buffer.size();
    }
};

//------------------------------------------------------------------------------
// Timestamps
//------------------------------------------------------------------------------

HL2SS_INLINE
int64_t ts_qpc_to_filetime(int64_t timestamp, int64_t utc_offset)
{
    return timestamp + utc_offset;
}

HL2SS_INLINE
int64_t ts_filetime_to_unix_hns(int64_t timestamp_filetime)
{
    return timestamp_filetime - time_base::UNIX_EPOCH;
}

HL2SS_INLINE
double ts_unix_hns_to_unix(int64_t timestamp_unix_hns)
{
    return timestamp_unix_hns / (double)time_base::HUNDREDS_OF_NANOSECONDS;
}

HL2SS_INLINE
int64_t ts_unix_to_unix_hns(double timestamp_unix)
{
    return (int64_t)(timestamp_unix * time_base::HUNDREDS_OF_NANOSECONDS);
}

HL2SS_INLINE
int64_t ts_unix_hns_to_filetime(int64_t timestamp_unix_hns)
{
    return timestamp_unix_hns + time_base::UNIX_EPOCH;
}

HL2SS_INLINE
int64_t ts_filetime_to_qpc(int64_t timestamp_filetime, int64_t utc_offset)
{
    return timestamp_filetime - utc_offset;
}
}

//******************************************************************************
// Implementation
//******************************************************************************

#ifndef HL2SS_SHARED

extern "C"
{ 
#include <libavcodec/avcodec.h>
}

#include <string>
#include <zdepth.hpp>

namespace hl2ss
{
//------------------------------------------------------------------------------
// Client
//------------------------------------------------------------------------------

class client
{
private:
#ifdef _WIN32
    uint64_t m_socket;
#else
    int m_socket;
#endif

public:
    static void initialize();
    static void cleanup();

    client();
    ~client();

    void open(char const* host, uint16_t port);
    void sendall(void const* data, uint64_t count);
    uint64_t recv(void* buffer, uint64_t count);
    void download(void* buffer, uint64_t total, uint64_t chunk);
    void close();
};

//------------------------------------------------------------------------------
// Packet
//------------------------------------------------------------------------------

class packet
{
public:
    static uint64_t const SZ_POSE = sizeof(matrix_4x4); 

    uint64_t timestamp;
    uint32_t sz_payload;
    std::unique_ptr<uint8_t[]> payload;
    std::unique_ptr<matrix_4x4> pose;

    packet();

    void init_payload(uint32_t size);
    void set_payload(uint32_t size, std::unique_ptr<uint8_t[]> new_payload);
    void init_pose();
};

//------------------------------------------------------------------------------
// Packet Gatherer
//------------------------------------------------------------------------------

class gatherer
{
private:
    client m_client;
    uint64_t m_chunk;
    uint8_t m_mode;

public:
    void open(char const* host, uint16_t port, uint64_t chunk, uint8_t mode);
    void sendall(void const* data, uint64_t count);
    std::unique_ptr<packet> get_next_packet();
    void close();
};

//------------------------------------------------------------------------------
// PV Control
//------------------------------------------------------------------------------

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc, bool hologram_composition, bool recording_indicator, bool video_stabilization, bool blank_protected, bool show_mesh, bool shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective);
void stop_subsystem_pv(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// Modes 0, 1 Data Acquisition
//------------------------------------------------------------------------------

class rx
{
private:
    gatherer m_client;
    std::vector<uint8_t> m_sc;

protected:
    virtual void create_configuration(std::vector<uint8_t>& sc) = 0;
    rx(char const* host, uint16_t port, uint64_t chunk, uint8_t mode);

public:
    std::string host;
    uint16_t port;
    uint64_t chunk;
    uint8_t mode;

    virtual ~rx();

    virtual void open();
    virtual std::unique_ptr<packet> get_next_packet();
    virtual void close();
};

class rx_rm_vlc : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    std::unique_ptr<packet> get_next_packet() override;
};

class rx_rm_depth_ahat : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t divisor;
    uint8_t profile_z;
    uint8_t profile_ab;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    std::unique_ptr<packet> get_next_packet() override;
};

class rx_rm_depth_longthrow : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t divisor;
    uint8_t png_filter;

    rx_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
};

class rx_rm_imu : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:

    rx_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode);
};

class rx_pv : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint16_t width;
    uint16_t height;
    uint8_t framerate;
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_pv(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    std::unique_ptr<packet> get_next_packet() override;
};

class rx_microphone : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t profile;
    uint8_t level;

    rx_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level);
};

class rx_si : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:

    rx_si(char const* host, uint16_t port, uint64_t chunk);
};

class rx_eet : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t fps;

    rx_eet(char const* host, uint16_t port, uint64_t chunk, uint8_t fps);
};

class rx_extended_audio : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint32_t mixer_mode;
    float loopback_gain;
    float microphone_gain;
    uint8_t profile;
    uint8_t level;

    rx_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level);
};

class rx_extended_depth : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t divisor;
    uint8_t profile_z;
    std::vector<uint64_t> options;

    rx_extended_depth(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, std::vector<uint64_t> const& options);
};

//------------------------------------------------------------------------------
// Frame
//------------------------------------------------------------------------------

class frame
{
public:
    AVFrame* av_frame;

    frame();
    ~frame();
};

//------------------------------------------------------------------------------
// Codec
//------------------------------------------------------------------------------

AVCodecID get_video_codec_id(uint8_t profile);
AVCodecID get_audio_codec_id(uint8_t profile);
uint32_t  get_audio_codec_bitrate(uint8_t profile);

class codec
{
private:
    AVCodecContext* m_c;
    AVPacket* m_avpkt;

public:
    codec();
    ~codec();

    void open(AVCodecID id);
    std::unique_ptr<frame> decode(uint8_t* payload, uint32_t size);
    void close();
};

//------------------------------------------------------------------------------
// Decoders
//------------------------------------------------------------------------------

void trim_plane(uint8_t* dst, uint8_t const* src, uint16_t height, uint16_t width, uint16_t stride);
void collect_i420(uint8_t* dst, int width, int height, uint8_t* data[8], int linesize[8]);
void collect_nv12(uint8_t* dst, uint16_t width, uint16_t height, uint8_t const* src, uint16_t stride);

class decoder_rm_vlc
{
private:
    uint8_t m_profile;
    codec m_codec;
    
public:
    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint32_t& decoded_size);
    void close();
};

class decoder_rm_depth_ahat
{
private:
    uint8_t m_profile_z;
    uint8_t m_profile_ab;
    zdepth::DepthCompressor m_zdc;
    codec m_codec_ab;

public:
    void open(uint8_t profile_z, uint8_t profile_ab);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint32_t& decoded_size);
    void close();
};

class decoder_rm_depth_longthrow
{
public:
    void open();
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint32_t& decoded_size);
    void close();
};

class decoder_pv
{
private:
    uint8_t m_profile;
    codec m_codec;
    
public:
    static int decoded_bpp(uint8_t decoded_format);
    static int decoded_cv_type(uint8_t decoded_format);
    static int decoded_cv_i420(uint8_t decoded_format);
    static int decoded_cv_nv12(uint8_t decoded_format);

    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size);
    void close();
};

class decoder_microphone
{
private:
    uint8_t m_profile;
    codec m_codec;

public:
    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint32_t& decoded_size);
    void close();
};

class decoder_extended_depth
{
private:
    uint8_t m_profile_z;
    zdepth::DepthCompressor m_zdc;

public:
    void open(uint8_t profile_z);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint32_t& decoded_size);
    void close();
};

//------------------------------------------------------------------------------
// Modes 0, 1 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

class rx_decoded_rm_vlc : public rx_rm_vlc
{
protected:
    decoder_rm_vlc m_decoder;

public:
    rx_decoded_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    
    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_ahat : public rx_rm_depth_ahat
{
protected:
    decoder_rm_depth_ahat m_decoder;

public:
    rx_decoded_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_longthrow : public rx_rm_depth_longthrow
{
protected:
    decoder_rm_depth_longthrow m_decoder;

public:
    rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
    
    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_imu : public rx_rm_imu
{
public:
    rx_decoded_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_pv : public rx_pv
{
protected:
    decoder_pv m_decoder;

public:
    uint8_t decoded_format;

    rx_decoded_pv(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_microphone : public rx_microphone
{
protected:
    decoder_microphone m_decoder;

public:
    rx_decoded_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_extended_audio : public rx_extended_audio
{
protected:
    decoder_microphone m_decoder;

public:
    rx_decoded_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_extended_depth : public rx_extended_depth
{
protected:
    decoder_extended_depth m_decoder;

public:
    rx_decoded_extended_depth(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, std::vector<uint64_t> const& options);

    void open() override;
    std::unique_ptr<packet> get_next_packet() override;
    void close() override;
};

//------------------------------------------------------------------------------
// Mode 2 Data Acquisition
//------------------------------------------------------------------------------

std::unique_ptr<calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port);
std::unique_ptr<calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port);
std::unique_ptr<calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port);
std::unique_ptr<calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port);
std::unique_ptr<calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate);
std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_audio(char const* host, uint16_t port, uint8_t profile, uint8_t level);
std::unique_ptr<std::vector<uint8_t>> download_devicelist_extended_video(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------

class ipc
{
protected:
    client m_client;

    ipc(char const* host, uint16_t port);

public:
    std::string host;
    uint16_t port;

    virtual ~ipc();

    virtual void open();
    virtual void close();
};

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

class ipc_rc : public ipc
{
protected:
    std::vector<uint8_t> m_sc;

    void send(uint8_t command, std::initializer_list<uint32_t> list);
    void send(void const* buffer, uint64_t size);
    void recv(void* buffer, uint64_t size);

public:
    ipc_rc(char const* host, uint16_t port);

    version ee_get_application_version();
    uint64_t ts_get_utc_offset();
    void hs_set_marker_state(uint32_t state);
    bool pv_get_subsystem_status();
    void pv_wait_for_subsystem(bool status);
    void pv_set_focus(uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback);
    void pv_set_video_temporal_denoising(uint32_t mode);
    void pv_set_white_balance_preset(uint32_t preset);
    void pv_set_white_balance_value(uint32_t value);
    void pv_set_exposure(uint32_t mode, uint32_t value);
    void pv_set_exposure_priority_video(uint32_t enabled);
    void pv_set_iso_speed(uint32_t mode, uint32_t value);
    void pv_set_backlight_compensation(uint32_t state);
    void pv_set_scene_mode(uint32_t mode);
    void ee_set_flat_mode(uint32_t mode);
    void rm_set_eye_selection(uint32_t enable);
    void pv_set_desired_optimization(uint32_t mode);
    void pv_set_primary_use(uint32_t mode);
    void pv_set_optical_image_stabilization(uint32_t mode);
    void pv_set_hdr_video(uint32_t mode);
    void pv_set_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h);
    void ee_set_interface_priority(uint16_t port, int32_t priority);
    void ee_set_quiet_mode(uint32_t mode);
    std::unique_ptr<vector_2[]> rm_map_camera_points(uint16_t port, uint32_t operation, vector_2 const* points, uint32_t count);
    std::unique_ptr<matrix_4x4[]> rm_get_rignode_world_poses(uint64_t const* timestamps, uint32_t count);
    uint64_t ts_get_current_time(uint32_t source);
    void si_set_sampling_delay(int64_t delay);
    void ee_set_encoder_buffering(bool enable);
    void ee_set_reader_buffering(bool enable);
    void rm_set_loop_control(uint16_t port, bool enable);
};

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

struct sm_mesh
{
    uint32_t status;
    vector_3 vertex_position_scale;
    matrix_4x4 pose;
    std::vector<uint8_t> bounds;
    std::vector<uint8_t> vertex_positions;
    std::vector<uint8_t> triangle_indices;
    std::vector<uint8_t> vertex_normals;
};

class ipc_sm : public ipc
{
public:
    ipc_sm(char const* host, uint16_t port);

    void set_volumes(sm_bounding_volume const& volumes);
    void get_observed_surfaces(std::vector<sm_surface_info>& surfaces);
    void get_meshes(sm_mesh_task const& tasks, std::vector<sm_mesh>& meshes);
};

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

struct su_mesh
{
    std::vector<uint8_t> vertex_positions;
    std::vector<uint8_t> triangle_indices;
};

struct su_item
{
    guid id;
    int32_t kind;
    quaternion orientation;
    vector_3 position;
    matrix_4x4 location;
    int32_t alignment;
    vector_2 extents;
    std::vector<su_mesh> meshes;
    std::vector<su_mesh> collider_meshes;
};

struct su_result
{
    uint32_t status;
    matrix_4x4 extrinsics;
    matrix_4x4 pose;
    std::vector<su_item> items;
};

class ipc_su : public ipc
{
protected:
    void pack_task(std::vector<uint8_t>& sc, su_task const& task);
    void download_meshes(std::vector<su_mesh>& meshes);

public:
    ipc_su(char const* host, uint16_t port);

    void query(su_task const& task, su_result& result);
};

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

class ipc_vi : public ipc
{
public:
    ipc_vi(char const* host, uint16_t port);

    void start(std::vector<std::u16string> const& strings);
    void pop(std::vector<vi_result>& results);
    void stop();
};

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

class ipc_umq : public ipc
{
public:
    ipc_umq(char const* host, uint16_t port);

    void push(uint8_t const* data, uint32_t size);
    void pull(uint32_t* data, uint32_t count);
};

//------------------------------------------------------------------------------
// Guest Message Queue
//------------------------------------------------------------------------------

struct gmq_message
{
    uint32_t command;
    uint32_t size;
    std::unique_ptr<uint8_t[]> data;
};

class ipc_gmq : public ipc
{
public:
    ipc_gmq(char const* host, uint16_t port);

    void pull(gmq_message& message);
    void push(uint32_t const* response, uint32_t count);
};
}

#endif
