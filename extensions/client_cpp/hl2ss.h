
#pragma once

#include <string>
#include <vector>
#include <memory>

extern "C" { 
#include <libavcodec/avcodec.h>
}

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
uint64_t const CODECAPI_AVEncCommonRateControlMode = 0;
uint64_t const CODECAPI_AVEncCommonQuality = 1;
uint64_t const CODECAPI_AVEncAdaptiveMode = 2;
uint64_t const CODECAPI_AVEncCommonBufferSize = 3;
uint64_t const CODECAPI_AVEncCommonMaxBitRate = 4;
uint64_t const CODECAPI_AVEncCommonMeanBitRate = 5;
uint64_t const CODECAPI_AVEncCommonQualityVsSpeed = 6;
uint64_t const CODECAPI_AVEncH264CABACEnable = 7;
uint64_t const CODECAPI_AVEncH264SPSID = 8;
uint64_t const CODECAPI_AVEncMPVDefaultBPictureCount = 9;
uint64_t const CODECAPI_AVEncMPVGOPSize = 10;
uint64_t const CODECAPI_AVEncNumWorkerThreads = 11;
uint64_t const CODECAPI_AVEncVideoContentType = 12;
uint64_t const CODECAPI_AVEncVideoEncodeQP = 13;
uint64_t const CODECAPI_AVEncVideoForceKeyFrame = 14; 
uint64_t const CODECAPI_AVEncVideoMinQP = 15;
uint64_t const CODECAPI_AVLowLatencyMode = 16;
uint64_t const CODECAPI_AVEncVideoMaxQP = 17;
uint64_t const CODECAPI_VideoEncoderDisplayContentType = 18;
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
    static size_t const SZ_POSE = 4*4*sizeof(float); 

    uint64_t timestamp;
    uint32_t sz_payload;
    std::unique_ptr<uint8_t[]> payload;
    std::unique_ptr<float[]> pose;

    packet();

    void init_payload(uint32_t size);
    void swap_payload(uint32_t size, std::unique_ptr<uint8_t[]> new_payload);
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
    void sendall(void const *data, size_t count);
    std::shared_ptr<packet> get_next_packet();
    void close();
};

//------------------------------------------------------------------------------
// * PV Control
//------------------------------------------------------------------------------

void start_subsystem_pv(char const* host, uint16_t port);
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

public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;

    rx(char const* host, uint16_t port, size_t chunk, uint8_t mode);
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

//------------------------------------------------------------------------------
// * Frame
//------------------------------------------------------------------------------

class frame
{
public:
    AVFrame *av_frame;

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
    AVCodecContext *m_c;
    AVPacket *m_avpkt;

public:
    codec();
    ~codec();

    void open(AVCodecID id);
    std::shared_ptr<frame> decode(uint8_t *payload, uint32_t size);
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
    static uint32_t const decoded_size = parameters_rm_vlc::PIXELS * sizeof(uint8_t);

    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

class decoder_rm_depth_ahat
{
private:
    codec m_codec;
    uint8_t m_profile_z;
    uint8_t m_profile_ab;

public:
    static uint32_t const decoded_size = 2 * parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t);

    void open(uint8_t profile_z, uint8_t profile_ab);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

class decoder_rm_depth_longthrow
{
public:
    static uint32_t const decoded_size = 2 * parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t);

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
    static uint32_t const K_SIZE = 4 * sizeof(float);

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
    static uint32_t const decoded_size = parameters_microphone::GROUP_SIZE_AAC * parameters_microphone::CHANNELS * sizeof(float);
    static uint32_t const raw_size     = parameters_microphone::GROUP_SIZE_RAW * parameters_microphone::CHANNELS * sizeof(int16_t);

    void open(uint8_t profile);
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size);
    void close();
};

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

class rx_decoded_rm_vlc : public rx_rm_vlc
{
private:
    decoder_rm_vlc m_decoder;

public:
    rx_decoded_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    
    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_ahat : public rx_rm_depth_ahat
{
private:
    decoder_rm_depth_ahat m_decoder;

public:
    rx_decoded_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_rm_depth_longthrow : public rx_rm_depth_longthrow
{
private:
    decoder_rm_depth_longthrow m_decoder;

public:
    rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
    
    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

class rx_decoded_pv : public rx_pv
{
private:
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
private:
    decoder_microphone m_decoder;

public:
    rx_decoded_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level);

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

uint16_t get_port_index(uint16_t port);
char const* get_port_name(uint16_t port);

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------








class ipc
{
protected:
    client m_client;
    std::vector<uint8_t> m_sc;

protected:
    void send(uint8_t command, std::initializer_list<uint32_t> list);
    void recv(void* buffer, size_t size);

    ipc(char const* host, uint16_t port);

public:
    std::string host;
    uint16_t port;

    void open();
    void close();
};




struct hs_marker_state
{
    static uint32_t const Disable = 0;
    static uint32_t const Enable = 1;
};

struct pv_focus_mode
{
    static uint32_t const Auto = 0;
    static uint32_t const Single = 1;
    static uint32_t const Continuous = 2;
    static uint32_t const Manual = 3;
};

struct pv_auto_focus_range
{
    static uint32_t const FullRange = 0;
    static uint32_t const Macro = 1;
    static uint32_t const Normal = 2;
};

struct pv_manual_focus_distance
{
    static uint32_t const Infinity = 0;
    static uint32_t const Nearest = 2;
};

struct pv_focus_value
{
    static uint32_t const Min = 170;
    static uint32_t const Max = 10000;
};

struct pv_driver_fallback
{
    static uint32_t const Enable = 0;
    static uint32_t const Disable = 1;
};

struct pv_video_temporal_denoising_mode
{
    static uint32_t const Off = 0;
    static uint32_t const On = 1;
};

struct pv_color_temperature_preset
{
    static uint32_t const Auto = 0;
    static uint32_t const Manual = 1;
    static uint32_t const Cloudy = 2;
    static uint32_t const Daylight = 3;
    static uint32_t const Flash = 4;
    static uint32_t const Fluorescent = 5;
    static uint32_t const Tungsten = 6;
    static uint32_t const Candlelight = 7;
};

struct pv_white_balance_value
{
    static uint32_t const Min = 2300; // 25
    static uint32_t const Max = 7500; // 25
};

struct pv_exposure_mode
{
    static uint32_t const Manual = 0;
    static uint32_t const Auto = 1;
};   

struct pv_exposure_value
{
    static uint32_t const Min = 1000; // 10
    static uint32_t const Max = 660000; // 10
};

struct pv_exposure_priority_video
{
    static uint32_t const Disabled = 0;
    static uint32_t const Enabled = 1;
};

struct pv_iso_speed_mode
{
    static uint32_t const Manual = 0;
    static uint32_t const Auto = 1;
};

struct pv_iso_speed_value
{
    static uint32_t const Min = 100;
    static uint32_t const Max = 3200;
};

struct pv_capture_scene_mode
{
    static uint32_t const Auto = 0;
    static uint32_t const Macro = 2;
    static uint32_t const Portrait = 3;
    static uint32_t const Sport = 4;
    static uint32_t const Snow = 5;
    static uint32_t const Night = 6;
    static uint32_t const Beach = 7;
    static uint32_t const Sunset = 8;
    static uint32_t const Candlelight = 9;
    static uint32_t const Landscape = 10;
    static uint32_t const NightPortrait = 11;
    static uint32_t const Backlit = 12;
};

struct pv_backlight_compensation_state
{
    static uint32_t const Disable = 0;
    static uint32_t const Enable = 1;
};





struct version
{
    uint16_t field[4];
};

class ipc_rc : public ipc
{
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
}
