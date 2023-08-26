
#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <memory>

extern "C" { 
#include <libavcodec/avcodec.h>
}

namespace hl2ss
{
enum stream_port : uint16_t
{
    RM_VLC_LEFTFRONT     = 3800,
    RM_VLC_LEFTLEFT      = 3801,
    RM_VLC_RIGHTFRONT    = 3802,
    RM_VLC_RIGHTRIGHT    = 3803,
    RM_DEPTH_AHAT        = 3804,
    RM_DEPTH_LONGTHROW   = 3805,
    RM_IMU_ACCELEROMETER = 3806,
    RM_IMU_GYROSCOPE     = 3807,
    RM_IMU_MAGNETOMETER  = 3808,
    PERSONAL_VIDEO       = 3810,
    MICROPHONE           = 3811,
    SPATIAL_INPUT        = 3812,
    EXTENDED_EYE_TRACKER = 3817,
};

enum ipc_port : uint16_t
{
    REMOTE_CONFIGURATION = 3809,
    SPATIAL_MAPPING      = 3813,
    SCENE_UNDERSTANDING  = 3814,
    VOICE_INPUT          = 3815,
    UNITY_MESSAGE_QUEUE  = 3816,
};

enum stream_mode : uint8_t
{
    MODE_0 = 0,
    MODE_1 = 1,
    MODE_2 = 2,
};

enum video_profile : uint8_t
{
    H264_BASE = 0,
    H264_MAIN = 1,
    H264_HIGH = 2,
    H265_MAIN = 3,
    VIDEO_RAW = 0xFF,
};

enum h26x_level : uint8_t
{
    H264_1   =  10,
    H264_1_b =  11,   
    H264_1_1 =  11,
    H264_1_2 =  12,
    H264_1_3 =  13,
    H264_2   =  20,
    H264_2_1 =  21,
    H264_2_2 =  22,
    H264_3   =  30,
    H264_3_1 =  31,
    H264_3_2 =  32,
    H264_4   =  40,
    H264_4_1 =  41,
    H264_4_2 =  42,
    H264_5   =  50,
    H264_5_1 =  51,
    H264_5_2 =  52,
    H265_1   =  30,
    H265_2   =  60,
    H265_2_1 =  63,
    H265_3   =  90,
    H265_3_1 =  93,
    H265_4   = 120,
    H265_4_1 = 123,
    H265_5   = 150,
    H265_5_1 = 153,
    H265_5_2 = 156,
    H265_6   = 180,
    H265_6_1 = 183,
    H265_6_2 = 186,
    DEFAULT  = 255,
};

enum depth_profile : uint8_t
{
    SAME   = 0,
    ZDEPTH = 1,
};

enum audio_profile : uint8_t
{
    AAC_12000 = 0,
    AAC_16000 = 1,
    AAC_20000 = 2,
    AAC_24000 = 3,
    AUDIO_RAW = 0xFF,
};

enum aac_level : uint8_t
{
    L2      = 0x29,
    L4      = 0x2A,
    L5      = 0x2B,
    HEV1L2  = 0x2C,
    HEV1L4  = 0x2E,
    HEV1L5  = 0x2F,
    HEV2L2  = 0x30,
    HEV2L3  = 0x31,
    HEV2L4  = 0x32,
    HEV2L5  = 0x33,
};

enum png_filter_mode : uint8_t
{
    AUTOMATIC = 0,
    DISABLE   = 1,
    SUB       = 2,
    UP        = 3,
    AVERAGE   = 4,
    PAETH     = 5,
    ADAPTIVE  = 6,
};

class parameters_rm_vlc
{
public:
    static uint16_t const WIDTH  = 640;
    static uint16_t const HEIGHT = 480;
    static uint8_t  const FPS    = 30;
    static uint32_t const PIXELS = WIDTH * HEIGHT;
};

class parameters_rm_depth_ahat
{
public:
    static uint16_t const WIDTH  = 512;
    static uint16_t const HEIGHT = 512;
    static uint8_t  const FPS    = 45;
    static uint32_t const PIXELS = WIDTH * HEIGHT;
};

class parameters_rm_depth_longthrow
{
public:
    static uint16_t const WIDTH  = 320;
    static uint16_t const HEIGHT = 288;
    static uint8_t  const FPS    = 5;
    static uint32_t const PIXELS = WIDTH * HEIGHT;
};

class parameters_rm_imu_accelerometer
{
public:
    static uint16_t const BATCH_SIZE = 93;
};

class parameters_rm_imu_gyroscope
{
public:
    static uint16_t const BATCH_SIZE = 315;
};

class parameters_rm_imu_magnetometer
{
public:
    static uint16_t const BATCH_SIZE = 11;
};

class parameters_microphone
{
public:
    static uint32_t const SAMPLE_RATE    = 48000;
    static uint8_t  const CHANNELS       = 2;
    static uint16_t const GROUP_SIZE_RAW = 768;
    static uint16_t const GROUP_SIZE_AAC = 1024;
};

class parameters_si
{
public:
    static uint8_t  const SAMPLE_RATE = 30;
};

class client
{
private:
    int m_socket;

public:
    client();
    ~client();

    void open(char const* host, uint16_t port);
    void sendall(char const* data, size_t count);
    size_t recv(char* buffer, size_t count);
    void download(char* buffer, size_t total, size_t chunk_size);
    void close();
};

class packet
{
public:
    static size_t const SZ_POSE = 4*4*sizeof(float); 

    uint64_t timestamp;
    uint32_t sz_payload;
    uint8_t* payload;
    float* pose;

    packet();
    ~packet();

    void init_payload();
    void init_pose();
    void free_payload();
    void free_pose();
};

class gatherer
{
private:
    client m_client;
    size_t m_chunk_size;
    uint8_t m_mode;

public:
    void open(char const* host, uint16_t port, size_t chunk_size, uint8_t mode);
    void sendall(char const *data, size_t count);
    std::shared_ptr<packet> get_next_packet();
    void close();
};

enum h26x_encoder_property : uint64_t
{
    CODECAPI_AVEncCommonRateControlMode = 0,
    CODECAPI_AVEncCommonQuality = 1,
    CODECAPI_AVEncAdaptiveMode = 2,
    CODECAPI_AVEncCommonBufferSize = 3,
    CODECAPI_AVEncCommonMaxBitRate = 4,
    CODECAPI_AVEncCommonMeanBitRate = 5,
    CODECAPI_AVEncCommonQualityVsSpeed = 6,
    CODECAPI_AVEncH264CABACEnable = 7,
    CODECAPI_AVEncH264SPSID = 8,
    CODECAPI_AVEncMPVDefaultBPictureCount = 9,
    CODECAPI_AVEncMPVGOPSize = 10,
    CODECAPI_AVEncNumWorkerThreads = 11,
    CODECAPI_AVEncVideoContentType = 12,
    CODECAPI_AVEncVideoEncodeQP = 13,
    CODECAPI_AVEncVideoForceKeyFrame = 14, 
    CODECAPI_AVEncVideoMinQP = 15,
    CODECAPI_AVLowLatencyMode = 16,
    CODECAPI_AVEncVideoMaxQP = 17,
    CODECAPI_VideoEncoderDisplayContentType = 18,
};

void start_subsystem_pv(char const* host, uint16_t port);
void stop_subsystem_pv(char const* host, uint16_t port);

class rx
{
protected:
    gatherer m_client;

public:
    virtual void open() = 0;
    virtual std::shared_ptr<packet> get_next_packet();
    virtual void close();
};

class rx_rm_vlc : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    void open();
};

class rx_rm_depth_ahat : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t profile_z;
    uint8_t profile_ab;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    void open();
};

class rx_rm_depth_longthrow : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;
    uint8_t divisor;
    uint8_t png_filter;

    rx_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
    void open();
};

class rx_rm_imu : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;

    rx_rm_imu(char const* host, uint16_t port, size_t chunk, uint8_t mode);
    void open();
};

class rx_pv : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t mode;
    uint16_t width;
    uint16_t height;
    uint8_t framerate;
    uint8_t divisor;
    uint8_t profile;
    uint8_t level;
    uint32_t bitrate;
    std::vector<uint64_t> options;

    rx_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    void open();
};

class rx_microphone : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t profile;
    uint8_t level;

    rx_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level);
    void open();
};

class rx_si : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;

    rx_si(char const* host, uint16_t port, size_t chunk);
    void open();
};

class rx_eet : public rx
{
public:
    std::string host;
    uint16_t port;
    size_t chunk;
    uint8_t fps;

    rx_eet(char const* host, uint16_t port, size_t chunk, uint8_t fps);
    void open();
};

class frame
{
public:
    AVFrame *av_frame;

    frame();
    ~frame();

    void init_frame();
    void free_frame();
};

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

class rx_decoded_rm_vlc : public rx_rm_vlc
{
private:
    codec m_codec;

public:
    rx_decoded_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);
    
    void open();
    std::shared_ptr<packet> get_next_packet();
    void close();
};

class rx_decoded_rm_depth_ahat : public rx_rm_depth_ahat
{
private:
    codec m_codec;

public:
    rx_decoded_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options);

    void open();
    std::shared_ptr<packet> get_next_packet();
    void close();
};

class rx_decoded_rm_depth_longthrow : public rx_rm_depth_longthrow
{
public:
    rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter);
    
    void open();
    std::shared_ptr<packet> get_next_packet();
    void close();
};

class rx_decoded_pv : public rx_pv
{
private:
    codec m_codec;

public:
    static int const cv_format[5][4];

    uint8_t decoded_format;

    rx_decoded_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format);
    
    void open();
    std::shared_ptr<packet> get_next_packet();
    void close();
};

class rx_decoded_microphone : public rx_microphone
{
private:
    codec m_codec;

public:
    rx_decoded_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level);

    void open();
    std::shared_ptr<packet> get_next_packet();
    void close();
};
}
