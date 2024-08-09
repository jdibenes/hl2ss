
#include <stdint.h>

#ifdef WIN32
#define HL2SS_CLIENT_IMPORT extern "C" __declspec(dllimport)
#else
#define HL2SS_CLIENT_IMPORT extern "C"
#endif

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
uint64_t const SINGLE_TRANSFER      = 4096;
}

namespace stream_mode
{
uint8_t const MODE_0 = 0;
uint8_t const MODE_1 = 1;
uint8_t const MODE_2 = 2;
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
uint32_t const QUERY      = 0x80000000;
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

//------------------------------------------------------------------------------
// Unpacking
//------------------------------------------------------------------------------

struct pv_intrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};

struct rm_imu_sample
{
    uint64_t sensor_timestamp;
    uint64_t timestamp;
    float x;
    float y;
    float z;
    float temperature;
};

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

//------------------------------------------------------------------------------
// Calibration
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

namespace pv_backlight_compensation_state
{
uint32_t const Disable = 0;
uint32_t const Enable  = 1;
}

struct version
{
    uint16_t field[4];
};

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

// TODO

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

// TODO

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
// Stream Configuration
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
    default:                                       return nullptr;
    }
}

constexpr
uint32_t extended_audio_device_mixer_mode(uint32_t mixer_mode, uint32_t device)
{
    uint32_t const DEVICE_BASE = 0x00000004;
    return mixer_mode | (DEVICE_BASE * (device + 1));
}

//------------------------------------------------------------------------------
// Unpacking
//------------------------------------------------------------------------------

constexpr
void unpack_rm_vlc(uint8_t* payload, uint8_t*& image)
{
    image = payload;
}

constexpr
void unpack_rm_depth_ahat(uint8_t* payload, uint16_t*& depth, uint16_t*& ab)
{
    depth = (uint16_t*)(payload);
    ab    = (uint16_t*)(payload + (parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t)));
}

constexpr
void unpack_rm_depth_longthrow(uint8_t* payload, uint16_t*& depth, uint16_t*& ab)
{
    depth = (uint16_t*)(payload);
    ab    = (uint16_t*)(payload + (parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t)));
}

constexpr
void unpack_rm_imu(uint8_t* payload, rm_imu_sample*& samples)
{
    samples = (rm_imu_sample*)payload;
}

constexpr
void unpack_pv(uint8_t* payload, uint64_t size, uint8_t*& image, pv_intrinsics*& intrinsics)
{
    image = payload;
    intrinsics = (pv_intrinsics*)(payload + size - sizeof(pv_intrinsics));
}

constexpr
void unpack_microphone_raw(uint8_t* payload, int16_t*& samples)
{
    samples = (int16_t*)payload;
}

constexpr
void unpack_microphone_aac(uint8_t* payload, float*& samples)
{
    samples = (float*)payload;
}

constexpr
void unpack_si(uint8_t* payload, si_frame*& si)
{
    si = (si_frame*)payload;
}

constexpr
void unpack_eet(uint8_t* payload, eet_frame*& eet)
{
    eet = (eet_frame*)payload;
}

constexpr
void unpack_extended_audio_raw(uint8_t* payload, int16_t*& samples)
{
    samples = (int16_t*)payload;
}

constexpr
void unpack_extended_audio_aac(uint8_t* payload, float*& samples)
{
    samples = (float*)payload;
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

struct packet
{
    uint64_t timestamp;
    uint32_t sz_payload;
    uint32_t _reserved;
    uint8_t* payload;
    matrix_4x4* pose;
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

HL2SS_CLIENT_IMPORT
int initialize();

//-----------------------------------------------------------------------------
// Open
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* open_rm_vlc(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_VLC, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, uint64_t const* options_data=nullptr, uint64_t options_size=0, uint64_t buffer_size=hl2ss::parameters_rm_vlc::FPS*10);

HL2SS_CLIENT_IMPORT
void* open_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_DEPTH_AHAT, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile_z=hl2ss::depth_profile::SAME, uint8_t profile_ab=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, uint64_t const* options_data=nullptr, uint64_t options_size=0, uint64_t buffer_size=hl2ss::parameters_rm_depth_ahat::FPS*10);

HL2SS_CLIENT_IMPORT
void* open_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_DEPTH_LONGTHROW, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t png_filter=hl2ss::png_filter_mode::PAETH, uint64_t buffer_size=hl2ss::parameters_rm_depth_longthrow::FPS*10);

HL2SS_CLIENT_IMPORT
void* open_rm_imu(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::RM_IMU, uint8_t mode=hl2ss::stream_mode::MODE_1, uint64_t buffer_size=300);

HL2SS_CLIENT_IMPORT
void* open_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, uint64_t chunk=hl2ss::chunk_size::PERSONAL_VIDEO, uint8_t mode=hl2ss::stream_mode::MODE_1, uint8_t divisor=1, uint8_t profile=hl2ss::video_profile::H265_MAIN, uint8_t level=hl2ss::h26x_level::DEFAULT, uint32_t bitrate=0, uint64_t const* options_data=nullptr, uint64_t options_size=0, uint8_t decoded_format=hl2ss::pv_decoded_format::BGR, uint64_t buffer_size=300);

HL2SS_CLIENT_IMPORT
void* open_microphone(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::MICROPHONE, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, uint64_t buffer_size=500);

HL2SS_CLIENT_IMPORT
void* open_si(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::SPATIAL_INPUT, uint64_t buffer_size=300);

HL2SS_CLIENT_IMPORT
void* open_eet(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::EXTENDED_EYE_TRACKER, uint8_t framerate=hl2ss::eet_framerate::FPS_30, uint64_t buffer_size=900);

HL2SS_CLIENT_IMPORT
void* open_extended_audio(char const* host, uint16_t port, uint64_t chunk=hl2ss::chunk_size::EXTENDED_AUDIO, uint32_t mixer_mode=hl2ss::mixer_mode::BOTH, float loopback_gain=1.0f, float microphone_gain=1.0f, uint8_t profile=hl2ss::audio_profile::AAC_24000, uint8_t level=hl2ss::aac_level::L2, uint64_t buffer_size=500);

HL2SS_CLIENT_IMPORT
void* open_rc(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void* open_sm(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void* open_su(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void* open_vi(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void* open_umq(char const* host, uint16_t port);

HL2SS_CLIENT_IMPORT
void* open_gmq(char const* host, uint16_t port);

//-----------------------------------------------------------------------------
// Close
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void close_source(void* source);

HL2SS_CLIENT_IMPORT
void close_ipc(void* ipc);

//-----------------------------------------------------------------------------
// Grab
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t get_by_index(void* source, int64_t& frame_stamp, int32_t& status, void*& frame, hl2ss::ulm::packet& packet);

HL2SS_CLIENT_IMPORT
int32_t get_by_timestamp(void* source, uint64_t timestamp, int32_t time_preference, int32_t tiebreak_right, int64_t& frame_stamp, int32_t& status, void*& frame, hl2ss::ulm::packet& packet);

HL2SS_CLIENT_IMPORT
void release_frame(void* frame);

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t start_subsystem_pv(char const* host, uint16_t port, uint8_t enable_mrc=false, uint8_t hologram_composition=true, uint8_t recording_indicator=false, uint8_t video_stabilization=false, uint8_t blank_protected=false, uint8_t show_mesh=false, uint8_t shared=false, float global_opacity=0.9f, float output_width=0.0f, float output_height=0.0f, uint32_t video_stabilization_length=0, uint32_t hologram_perspective=hl2ss::hologram_perspective::PV);

HL2SS_CLIENT_IMPORT
int32_t stop_subsystem_pv(char const* host, uint16_t port);

//-----------------------------------------------------------------------------
// Calibration
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t download_calibration_rm_vlc(char const* host, uint16_t port, calibration_rm_vlc& calibration);

HL2SS_CLIENT_IMPORT
int32_t download_calibration_rm_depth_ahat(char const* host, uint16_t port, calibration_rm_depth_ahat& calibration);

HL2SS_CLIENT_IMPORT
int32_t download_calibration_rm_depth_longthrow(char const* host, uint16_t port, calibration_rm_depth_longthrow& calibration);

HL2SS_CLIENT_IMPORT
int32_t download_calibration_rm_imu(char const* host, uint16_t port, calibration_rm_imu& calibration);

HL2SS_CLIENT_IMPORT
int32_t download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate, calibration_pv& calibration);

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t rc_get_application_version(void* ipc, hl2ss::version& version);

HL2SS_CLIENT_IMPORT
int32_t rc_get_utc_offset(void* ipc, uint32_t samples, uint64_t& offset);

HL2SS_CLIENT_IMPORT
int32_t rc_set_hs_marker_state(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t rc_get_pv_subsystem_status(void* ipc, uint32_t& status);

HL2SS_CLIENT_IMPORT
int32_t rc_wait_for_pv_subsystem(void* ipc, uint32_t status);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_focus(void* ipc, uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_video_temporal_denoising(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_white_balance_preset(void* ipc, uint32_t preset);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_white_balance_value(void* ipc, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_exposure(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_exposure_priority_video(void* ipc, uint32_t enabled);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_iso_speed(void* ipc, uint32_t mode, uint32_t value);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_backlight_compensation(void* ipc, uint32_t state);

HL2SS_CLIENT_IMPORT
int32_t rc_set_pv_scene_mode(void* ipc, uint32_t mode);

HL2SS_CLIENT_IMPORT
int32_t rc_set_flat_mode(void* ipc, uint32_t mode);

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

// TODO:

//------------------------------------------------------------------------------
// Scene Understanding
//------------------------------------------------------------------------------

// TODO:

//------------------------------------------------------------------------------
// Voice Input
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t vi_create_recognizer(void* ipc);

HL2SS_CLIENT_IMPORT
int32_t vi_register_commands(void* ipc, bool clear, char const* utf8_array, uint32_t& status);

HL2SS_CLIENT_IMPORT
int32_t vi_start(void* ipc);

HL2SS_CLIENT_IMPORT
void* vi_pop(void* ipc, uint64_t& size, hl2ss::vi_result*& data);

HL2SS_CLIENT_IMPORT
void vi_release(void* results);

HL2SS_CLIENT_IMPORT
int32_t vi_clear(void* ipc);

HL2SS_CLIENT_IMPORT
int32_t vi_stop(void* ipc);

//------------------------------------------------------------------------------
// Unity Message Queue
//------------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
int32_t umq_push(void* ipc, uint8_t const* data, uint64_t size);

HL2SS_CLIENT_IMPORT
int32_t umq_pull(void* ipc, uint32_t* data, uint32_t count);

//-----------------------------------------------------------------------------
// Guest Message Queue
//-----------------------------------------------------------------------------

HL2SS_CLIENT_IMPORT
void* gmq_pull(void *ipc, hl2ss::ulm::gmq_message& result);

HL2SS_CLIENT_IMPORT
int32_t gmq_push(void* ipc, uint32_t const* response, uint32_t count);

HL2SS_CLIENT_IMPORT
void gmq_release(void* message);

}
}
