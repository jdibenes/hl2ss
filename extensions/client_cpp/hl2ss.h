
#pragma once

#include <string>
#include <vector>
#include <memory>

extern "C" { 
#include <libavcodec/avcodec.h>
}

#ifdef HL2SS_ENABLE_ZDEPTH
#include <zdepth.hpp>
#endif

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
};

namespace ipc_port
{
uint16_t const REMOTE_CONFIGURATION = 3809;
uint16_t const SPATIAL_MAPPING      = 3813;
uint16_t const SCENE_UNDERSTANDING  = 3814;
uint16_t const VOICE_INPUT          = 3815;
uint16_t const UNITY_MESSAGE_QUEUE  = 3816;
};

namespace chunk_size
{
size_t const RM_VLC               = 4096;
size_t const RM_DEPTH_AHAT        = 4096;
size_t const RM_DEPTH_LONGTHROW   = 4096;
size_t const RM_IMU               = 4096;
size_t const PERSONAL_VIDEO       = 4096;
size_t const MICROPHONE           = 512;
size_t const SPATIAL_INPUT        = 1024;
size_t const EXTENDED_EYE_TRACKER = 256;
size_t const EXTENDED_AUDIO       = 512;
size_t const SINGLE_TRANSFER      = 4096;
}

namespace stream_mode
{
uint8_t const MODE_0 = 0;
uint8_t const MODE_1 = 1;
uint8_t const MODE_2 = 2;
};

namespace video_profile
{
uint8_t const H264_BASE = 0;
uint8_t const H264_MAIN = 1;
uint8_t const H264_HIGH = 2;
uint8_t const H265_MAIN = 3;
uint8_t const RAW       = 0xFF;
};

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
};

namespace depth_profile
{
uint8_t const SAME   = 0;
uint8_t const ZDEPTH = 1;
};

namespace audio_profile
{
uint8_t const AAC_12000 = 0;
uint8_t const AAC_16000 = 1;
uint8_t const AAC_20000 = 2;
uint8_t const AAC_24000 = 3;
uint8_t const RAW       = 0xFF;
};

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
};

namespace png_filter_mode
{
uint8_t const AUTOMATIC = 0;
uint8_t const DISABLE   = 1;
uint8_t const SUB       = 2;
uint8_t const UP        = 3;
uint8_t const AVERAGE   = 4;
uint8_t const PAETH     = 5;
uint8_t const ADAPTIVE  = 6;
};

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
};

namespace hologram_perspective
{
uint32_t const DISPLAY = 0;
uint32_t const PV      = 1;
};

namespace mixer_mode
{
uint32_t const MICROPHONE = 0;
uint32_t const SYSTEM     = 1;
uint32_t const BOTH       = 2;
};

namespace pv_decoded_format
{
uint8_t const BGR  = 0;
uint8_t const RGB  = 1;
uint8_t const BGRA = 2;
uint8_t const RGBA = 3;
uint8_t const GRAY = 4;
};

namespace eet_framerate
{
uint8_t const FPS_30 = 30;
uint8_t const FPS_60 = 60;
uint8_t const FPS_90 = 90;
};

namespace parameters_rm_vlc
{
uint16_t const WIDTH  = 640;
uint16_t const HEIGHT = 480;
uint8_t  const FPS    = 30;
uint32_t const PIXELS = WIDTH * HEIGHT;
};

namespace parameters_rm_depth_ahat
{
uint16_t const WIDTH  = 512;
uint16_t const HEIGHT = 512;
uint8_t  const FPS    = 45;
uint32_t const PIXELS = WIDTH * HEIGHT;
};

namespace parameters_rm_depth_longthrow
{
uint16_t const WIDTH  = 320;
uint16_t const HEIGHT = 288;
uint8_t  const FPS    = 5;
uint32_t const PIXELS = WIDTH * HEIGHT;
};

namespace parameters_rm_imu_accelerometer
{
uint16_t const BATCH_SIZE = 93;
};

namespace parameters_rm_imu_gyroscope
{
uint16_t const BATCH_SIZE = 315;
};

namespace parameters_rm_imu_magnetometer
{
uint16_t const BATCH_SIZE = 11;
};

namespace parameters_microphone
{
uint32_t const SAMPLE_RATE    = 48000;
uint8_t  const CHANNELS       = 2;
uint16_t const GROUP_SIZE_RAW = 768;
uint16_t const GROUP_SIZE_AAC = 1024;
};

namespace parameters_si
{
uint8_t const SAMPLE_RATE = 30;
};

namespace parameters_extended_audio
{
uint32_t const SAMPLE_RATE    = 48000;
uint8_t  const CHANNELS       = 2;
uint16_t const GROUP_SIZE_AAC = 1024;
};

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
// * Client
//------------------------------------------------------------------------------

class client
{
private:
#ifdef WIN32
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
    void sendall(void const* data, size_t count);
    size_t recv(void* buffer, size_t count);
    void download(void* buffer, size_t total, size_t chunk);
    void close();
};

//------------------------------------------------------------------------------
// * Packet
//------------------------------------------------------------------------------

class packet
{
public:
    static size_t const SZ_POSE = sizeof(matrix_4x4); 

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
// * Packet Gatherer
//------------------------------------------------------------------------------

class gatherer
{
private:
    client m_client;
    size_t m_chunk;
    uint8_t m_mode;

public:
    void open(char const* host, uint16_t port, size_t chunk, uint8_t mode);
    void sendall(void const* data, size_t count);
    std::shared_ptr<packet> get_next_packet();
    void close();
};

//------------------------------------------------------------------------------
// * PV Control
//------------------------------------------------------------------------------

struct pv_intrinsics
{
    float fx;
    float fy;
    float cx;
    float cy;
};

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc, bool hologram_composition, bool recording_indicator, bool video_stabilization, bool blank_protected, bool show_mesh, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective);
void stop_subsystem_pv(char const* host, uint16_t port);

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition
//------------------------------------------------------------------------------

class rx
{
private:
    gatherer m_client;
    std::vector<uint8_t> m_sc;

protected:
    virtual void create_configuration(std::vector<uint8_t>& sc) = 0;
    rx(char const* host, uint16_t port, size_t chunk, uint8_t mode);

public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;

    virtual ~rx();

    virtual void open();
    virtual std::shared_ptr<packet> get_next_packet();
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

    rx_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
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

    rx_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
};

class rx_rm_depth_longthrow : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t divisor;
    uint8_t png_filter;

    rx_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
};

class rx_rm_imu : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:

    rx_rm_imu(char const* host, uint16_t port, size_t chunk, uint8_t mode);
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

    rx_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
};

class rx_microphone : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t profile;
    uint8_t level;

    rx_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level);
};

class rx_si : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:

    rx_si(char const* host, uint16_t port, size_t chunk);
};

class rx_eet : public rx
{
protected:
    void create_configuration(std::vector<uint8_t>& sc) override;

public:
    uint8_t framerate;

    rx_eet(char const* host, uint16_t port, size_t chunk, uint8_t framerate);
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

    rx_extended_audio(char const* host, uint16_t port, size_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level);
};

//------------------------------------------------------------------------------
// * Frame
//------------------------------------------------------------------------------

class frame
{
public:
    AVFrame* av_frame;

    frame();
    ~frame();
};

//------------------------------------------------------------------------------
// * Codec
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
    std::shared_ptr<frame> decode(uint8_t* payload, uint32_t size);
    void close();
};

//------------------------------------------------------------------------------
// * Decoders
//------------------------------------------------------------------------------

class decoder_rm_vlc
{
private:
    codec m_codec;
    uint8_t m_profile;
    
public:
    static uint32_t const DECODED_SIZE = parameters_rm_vlc::PIXELS * sizeof(uint8_t);

    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

class decoder_rm_depth_ahat
{
private:
    codec m_codec;
#ifdef HL2SS_ENABLE_ZDEPTH
    zdepth::DepthCompressor m_zdc;
#endif
    uint8_t m_profile_z;
    uint8_t m_profile_ab;

public:
    static uint32_t const DECODED_SIZE = 2 * parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t);

    void open(uint8_t profile_z, uint8_t profile_ab);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

class decoder_rm_depth_longthrow
{
public:
    static uint32_t const DECODED_SIZE = 2 * parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t);

    void open();
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

class decoder_pv
{
private:
    codec m_codec;
    uint16_t m_width;
    uint16_t m_height;
    uint8_t m_profile;

public:
    static uint32_t const K_SIZE = sizeof(pv_intrinsics);

    static uint8_t decoded_bpp(uint8_t decoded_format);

    void open(uint16_t width, uint16_t height, uint8_t profile);
    uint32_t decoded_size(uint8_t decoded_format);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint8_t decoded_format);
    void close();
};

class decoder_microphone
{
private:
    codec m_codec;
    uint8_t m_profile;

public:
    static uint32_t const DECODED_SIZE = parameters_microphone::GROUP_SIZE_AAC * parameters_microphone::CHANNELS * sizeof(float);

    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

class rx_decoded_rm_vlc : public rx_rm_vlc
{
protected:
    decoder_rm_vlc m_decoder;

public:
    rx_decoded_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    
    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_ahat : public rx_rm_depth_ahat
{
protected:
    decoder_rm_depth_ahat m_decoder;

public:
    rx_decoded_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_longthrow : public rx_rm_depth_longthrow
{
protected:
    decoder_rm_depth_longthrow m_decoder;

public:
    rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
    
    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_pv : public rx_pv
{
protected:
    decoder_pv m_decoder;

public:
    uint8_t decoded_format;

    rx_decoded_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_microphone : public rx_microphone
{
protected:
    decoder_microphone m_decoder;

public:
    rx_decoded_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_extended_audio : public rx_extended_audio
{
protected:
    decoder_microphone m_decoder;

public:
    rx_decoded_extended_audio(char const* host, uint16_t port, size_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

//------------------------------------------------------------------------------
// * Mode 2 Data Acquisition
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
};

std::shared_ptr<calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port);
std::shared_ptr<calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port);
std::shared_ptr<calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port);
std::shared_ptr<calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port);
std::shared_ptr<calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate);

//------------------------------------------------------------------------------
// * Port Information
//------------------------------------------------------------------------------

char const* get_port_name(uint16_t port);

//------------------------------------------------------------------------------
// * IPC
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
// * Remote Configuration
//------------------------------------------------------------------------------

namespace hs_marker_state
{
uint32_t const Disable = 0;
uint32_t const Enable  = 1;
};

namespace pv_focus_mode
{
uint32_t const Auto       = 0;
uint32_t const Single     = 1;
uint32_t const Continuous = 2;
uint32_t const Manual     = 3;
};

namespace pv_auto_focus_range
{
uint32_t const FullRange = 0;
uint32_t const Macro     = 1;
uint32_t const Normal    = 2;
};

namespace pv_manual_focus_distance
{
uint32_t const Infinity = 0;
uint32_t const Nearest  = 2;
};

namespace pv_focus_value
{
uint32_t const Min =   170;
uint32_t const Max = 10000;
};

namespace pv_driver_fallback
{
uint32_t const Enable  = 0;
uint32_t const Disable = 1;
};

namespace pv_video_temporal_denoising_mode
{
uint32_t const Off = 0;
uint32_t const On  = 1;
};

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
};

namespace pv_white_balance_value
{
uint32_t const Min = 2300; // 25
uint32_t const Max = 7500; // 25
};

namespace pv_exposure_mode
{
uint32_t const Manual = 0;
uint32_t const Auto   = 1;
};   

namespace pv_exposure_value
{
uint32_t const Min =   1000; // 10
uint32_t const Max = 660000; // 10
};

namespace pv_exposure_priority_video
{
uint32_t const Disabled = 0;
uint32_t const Enabled  = 1;
};

namespace pv_iso_speed_mode
{
uint32_t const Manual = 0;
uint32_t const Auto   = 1;
};

namespace pv_iso_speed_value
{
uint32_t const Min =  100;
uint32_t const Max = 3200;
};

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
};

namespace pv_backlight_compensation_state
{
uint32_t const Disable = 0;
uint32_t const Enable  = 1;
};

struct version
{
    uint16_t field[4];
};

class ipc_rc : public ipc
{
protected:
    std::vector<uint8_t> m_sc;

    void send(uint8_t command, std::initializer_list<uint32_t> list);
    void recv(void* buffer, size_t size);

public:
    ipc_rc(char const* host, uint16_t port);

    version get_application_version();
    uint64_t get_utc_offset(uint32_t samples);
    void set_hs_marker_state(uint32_t state);
    bool get_pv_subsystem_status();
    void wait_for_pv_subsystem(bool status);
    void set_pv_focus(uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback);
    void set_pv_video_temporal_denoising(uint32_t mode);
    void set_pv_white_balance_preset(uint32_t preset);
    void set_pv_white_balance_value(uint32_t value);
    void set_pv_exposure(uint32_t mode, uint32_t value);
    void set_pv_exposure_priority_video(uint32_t enabled);
    void set_pv_iso_speed(uint32_t mode, uint32_t value);
    void set_pv_backlight_compensation(uint32_t state);
    void set_pv_scene_mode(uint32_t mode);
};

//------------------------------------------------------------------------------
// * Spatial Mapping
//------------------------------------------------------------------------------

namespace sm_vertex_position_format
{
uint32_t const R32G32B32A32Float         =  2;
uint32_t const R16G16B16A16IntNormalized = 13;
};

namespace sm_triangle_index_format
{
uint32_t const R16UInt = 57;
uint32_t const R32Uint = 42;
};

namespace sm_vertex_normal_format
{
uint32_t const R32G32B32A32Float     =  2;
uint32_t const R8G8B8A8IntNormalized = 31;
};

namespace sm_volume_type
{
uint32_t const Box         = 0;
uint32_t const Frustum     = 1;
uint32_t const OrientedBox = 2;
uint32_t const Sphere      = 3;
};

class sm_bounding_volume
{
    friend class ipc_sm;

private:
    std::vector<uint8_t> m_data;
    uint32_t m_count;

public:
    sm_bounding_volume();

    void add_box(vector_3 center, vector_3 extents);
    void add_frustum(plane p_near, plane p_far, plane p_right, plane p_left, plane p_top, plane p_bottom);
    void add_oriented_box(vector_3 center, vector_3 extents, quaternion orientation);
    void add_sphere(vector_3 center, float radius);
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

class sm_mesh_task
{
    friend class ipc_sm;

private:
    std::vector<uint8_t> m_data;
    uint32_t m_count;
    
public:
    sm_mesh_task();

    void add_task(guid id, double max_triangles_per_cubic_meter, uint32_t vertex_position_format, uint32_t triangle_index_format, uint32_t vertex_normal_format, bool include_vertex_normals, bool include_bounds);
};

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

    void create_observer();
    void set_volumes(sm_bounding_volume const& volumes);
    void get_observed_surfaces(std::vector<sm_surface_info>& surfaces);
    void get_meshes(sm_mesh_task const& tasks, uint32_t threads, std::vector<sm_mesh>& meshes);
};

//------------------------------------------------------------------------------
// * Scene Understanding
//------------------------------------------------------------------------------

namespace su_mesh_lod
{
uint32_t const Coarse    =   0;
uint32_t const Medium    =   1;
uint32_t const Fine      =   2;
uint32_t const Unlimited = 255;
};

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
};

namespace su_create
{
uint8_t const New             = 0;
uint8_t const NewFromPrevious = 1;
};

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
};

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

class ipc_su : public ipc
{
protected:
    void download_meshes(std::vector<su_mesh>& meshes);

public:
    ipc_su(char const* host, uint16_t port);

    void query(su_task const& task, su_result& result);
};

//------------------------------------------------------------------------------
// * Voice Input
//------------------------------------------------------------------------------

namespace vi_speech_recognition_confidence
{
uint32_t const High     = 0;
uint32_t const Medium   = 1;
uint32_t const Low      = 2;
uint32_t const Rejected = 3;
};

struct vi_result
{
    uint32_t index;
    uint32_t confidence;
    uint64_t phrase_duration;
    uint64_t phrase_start_time;
    double   raw_confidence;
};

class ipc_vi : public ipc
{
public:
    ipc_vi(char const* host, uint16_t port);

    void create_recognizer();
    bool register_commands(bool clear, std::vector<std::u16string> const& strings);
    void start();
    void pop(std::vector<vi_result>& results);
    void clear();
    void stop();
};

//------------------------------------------------------------------------------
// * Unity Message Queue
//------------------------------------------------------------------------------

class umq_command_buffer
{
private:
    std::vector<uint8_t> m_buffer;
    uint32_t m_count;

public:
    umq_command_buffer();

    void add(uint32_t id, void const* data, size_t size);
    void clear();
    uint8_t const* data();
    size_t size();
    uint32_t count();
};

class ipc_umq : public ipc
{
public:
    ipc_umq(char const* host, uint16_t port);

    void push(uint8_t const* data, size_t size);
    void pull(uint32_t* data, uint32_t count);
};

//------------------------------------------------------------------------------
// * Unpacking
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

namespace si_valid
{
uint32_t const HEAD  = 0x01;
uint32_t const EYE   = 0x02;
uint32_t const LEFT  = 0x04;
uint32_t const RIGHT = 0x08;
};

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
};

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

void unpack_rm_vlc(uint8_t* payload, uint8_t** image);
void unpack_rm_depth_ahat(uint8_t* payload, uint16_t** depth, uint16_t** ab);
void unpack_rm_depth_longthrow(uint8_t* payload, uint16_t** depth, uint16_t** ab);
void unpack_rm_imu(uint8_t* payload, rm_imu_sample** samples);
void unpack_pv(uint8_t* payload, size_t size, uint8_t** image, pv_intrinsics** intrinsics);
void unpack_microphone_raw(uint8_t* payload, int16_t** samples);
void unpack_microphone_aac(uint8_t* payload, float** samples);
void unpack_si(uint8_t* payload, si_frame** si);
void unpack_eet(uint8_t* payload, eet_frame** eet);
void unpack_extended_audio_raw(uint8_t* payload, int16_t** samples);
void unpack_extended_audio_aac(uint8_t* payload, float** samples);
}
