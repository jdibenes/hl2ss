
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "hl2ss.h"

namespace hl2ss
{
union v8  {                           uint8_t  b; int8_t  c; };
union v16 { struct { v8  b0, b1; } b; uint16_t w; int16_t s; };
union v32 { struct { v16 w0, w1; } w; uint32_t d; int32_t i; };
union v64 { struct { v32 d0, d1; } d; uint64_t q; int64_t l; };

//------------------------------------------------------------------------------
// * Client
//------------------------------------------------------------------------------

void client::initialize()
{
#ifdef _WIN32
    WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) { throw std::runtime_error("hl2ss::client::initialize : WSAStartup failed"); }
#endif
}

void client::cleanup()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

client::client()
{
#ifdef _WIN32
    m_socket = INVALID_SOCKET;
#else
    m_socket = -1;
#endif
}

client::~client()
{
    close();
}

void client::open(char const* host, uint16_t port)
{
    m_socket = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (m_socket == INVALID_SOCKET)
#else
    if (m_socket < 0)
#endif
    { throw std::runtime_error("hl2ss::client::open : socket failed"); }

    sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    int status = inet_pton(AF_INET, host, &addr.sin_addr);
    if (status <= 0) { throw std::runtime_error("hl2ss::client::open : inet_pton failed"); }

    status = connect(m_socket, (sockaddr*)&addr, sizeof(addr));
    if (status < 0) { throw std::runtime_error("hl2ss::client::open : connect failed"); }
}

void client::sendall(void const* data, uint64_t count)
{
    int64_t sent = 0;
    while (count > 0)
    {
    int64_t bytes = send(m_socket, (char*)data + sent, (int)count, 0);
    if (bytes <= 0) { throw std::runtime_error("hl2ss::client::sendall : send failed"); }
    count -= bytes;
    sent  += bytes;
    }
}

uint64_t client::recv(void* buffer, uint64_t count)
{
    int64_t bytes = ::recv(m_socket, (char*)buffer, (int)count, 0);
    if ((bytes <= 0) && (count > 0)) { throw std::runtime_error("hl2ss::client::recv : recv failed"); }
    return bytes;
}

void client::download(void* buffer, uint64_t total, uint64_t chunk)
{
    uint64_t received = 0;
    while (total > 0)
    {
    if (chunk > total) { chunk = total; }
    uint64_t bytes = recv((char*)buffer + received, chunk);
    total    -= bytes;
    received += bytes;
    }
}

void client::close()
{
#ifdef _WIN32
    if (m_socket == INVALID_SOCKET) { return; }
    closesocket(m_socket);
    m_socket = INVALID_SOCKET;
#else
    if (m_socket < 0) { return; }
    ::close(m_socket);
    m_socket = -1;
#endif
}

//------------------------------------------------------------------------------
// * Packet
//------------------------------------------------------------------------------

packet::packet()
{
    timestamp = 0;
    sz_payload = 0;
    payload = nullptr;
    pose = nullptr;
}

void packet::init_payload(uint32_t size)
{
    sz_payload = size;    
    payload = (size > 0) ? std::make_unique<uint8_t[]>(size) : nullptr;
}

void packet::set_payload(uint32_t size, std::unique_ptr<uint8_t[]> new_payload)
{
    payload = std::move(new_payload);
    sz_payload = size;
}

void packet::init_pose()
{
    pose = std::make_unique<matrix_4x4>();
}

//------------------------------------------------------------------------------
// * Packet Gatherer
//------------------------------------------------------------------------------

void gatherer::open(char const* host, uint16_t port, uint64_t chunk, uint8_t mode)
{
    m_client.open(host, port);
    m_chunk = chunk;
    m_mode = mode;
}

void gatherer::sendall(void const* data, uint64_t count)
{
    m_client.sendall(data, count);
}

std::shared_ptr<packet> gatherer::get_next_packet()
{
    std::shared_ptr<packet> p = std::make_shared<packet>();

    m_client.download(&p->timestamp, sizeof(p->timestamp), m_chunk);
    m_client.download(&p->sz_payload, sizeof(p->sz_payload), m_chunk);
    p->init_payload(p->sz_payload);
    m_client.download(p->payload.get(), p->sz_payload, m_chunk);
    if (m_mode == stream_mode::MODE_1)
    {
    p->init_pose();
    m_client.download(p->pose.get(), packet::SZ_POSE, m_chunk);
    }

    return p;
}

void gatherer::close()
{
    m_client.close();
}

//------------------------------------------------------------------------------
// * Packer
//------------------------------------------------------------------------------

void push_u8(std::vector<uint8_t>& sc, uint8_t byte)
{
    sc.push_back(byte);
}

void push_u16(std::vector<uint8_t>& sc, uint16_t word)
{
    v16 data;

    data.w = word;

    push_u8(sc, data.b.b0.b);
    push_u8(sc, data.b.b1.b);
}

void push_u32(std::vector<uint8_t>& sc, uint32_t dword)
{
    v32 data;

    data.d = dword;

    push_u16(sc, data.w.w0.w);
    push_u16(sc, data.w.w1.w);
}

void push_u64(std::vector<uint8_t>& sc, uint64_t qword)
{
    v64 data;

    data.q = qword;

    push_u32(sc, data.d.d0.d);
    push_u32(sc, data.d.d1.d);
}

void push_float(std::vector<uint8_t>& sc, float f)
{
    push_u32(sc, *(uint32_t*)&f);
}

void push_double(std::vector<uint8_t>& sc, double d)
{
    push_u64(sc, *(uint64_t*)&d);
}

void push(std::vector<uint8_t>& sc, void const* data, uint64_t size)
{
    sc.insert(sc.end(), (uint8_t*)data, ((uint8_t*)data) + size);
}

//------------------------------------------------------------------------------
// * Configuration Primitives
//------------------------------------------------------------------------------

void create_configuration_for_mode(std::vector<uint8_t>& sc, uint8_t mode)
{
    push_u8(sc, mode);
}

void create_configuration_for_video_resolution(std::vector<uint8_t>& sc, uint16_t width, uint16_t height)
{
    push_u16(sc, width);
    push_u16(sc, height);
}

void create_configuration_for_framerate(std::vector<uint8_t>& sc, uint8_t framerate)
{
    push_u8(sc, framerate);
}

void create_configuration_for_video_divisor(std::vector<uint8_t>& sc, uint8_t divisor)
{
    push_u8(sc, divisor);
}

void create_configuration_for_video_encoding(std::vector<uint8_t>& sc, uint8_t profile, uint8_t level, uint32_t bitrate)
{
    push_u8(sc, profile);
    push_u8(sc, level);
    push_u32(sc, bitrate);
}

void create_configuration_for_depth_encoding(std::vector<uint8_t>& sc, uint8_t profile)
{
    push_u8(sc, profile);
}

void create_configuration_for_audio_encoding(std::vector<uint8_t>& sc, uint8_t profile, uint8_t level)
{
    push_u8(sc, profile);
    push_u8(sc, level);
}

void create_configuration_for_png_encoding(std::vector<uint8_t>& sc, uint32_t png_filter)
{
    push_u32(sc, png_filter);
}

void create_configuration_for_h26x_encoding(std::vector<uint8_t>& sc, std::vector<uint64_t> const& options)
{
    push_u8(sc, (uint8_t)(options.size() / 2));

    for (uint64_t i = 0; i < (options.size() & ~1ULL); i += 2)
    {
    push_u64(sc, options[i]);
    push_u64(sc, options[i+1]);
    }
}

void create_configuration_for_mrc_video(std::vector<uint8_t>& sc, bool enable_mrc, bool hologram_composition, bool recording_indicator, bool video_stabilization, bool blank_protected, bool show_mesh, bool shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective)
{
    push_u8(sc, enable_mrc);
    push_u8(sc, hologram_composition);
    push_u8(sc, recording_indicator);
    push_u8(sc, video_stabilization);
    push_u8(sc, blank_protected);
    push_u8(sc, show_mesh);
    push_u8(sc, shared);
    push_float(sc, global_opacity);
    push_float(sc, output_width);
    push_float(sc, output_height);
    push_u32(sc, video_stabilization_length);
    push_u32(sc, hologram_perspective);
}

void create_configuration_for_mrc_audio(std::vector<uint8_t>& sc, uint32_t mixer_mode, float loopback_gain, float microphone_gain)
{
    push_u32(sc, mixer_mode);
    push_float(sc, loopback_gain);
    push_float(sc, microphone_gain);
}

//------------------------------------------------------------------------------
// * Stream Configuration
//------------------------------------------------------------------------------

void create_configuration_for_rm_vlc(std::vector<uint8_t>& sc, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_divisor(sc, divisor);
    create_configuration_for_video_encoding(sc, profile, level, bitrate);
    create_configuration_for_h26x_encoding(sc, options);
}

void create_configuration_for_rm_depth_ahat(std::vector<uint8_t>& sc, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_divisor(sc, divisor);
    create_configuration_for_depth_encoding(sc, profile_z);
    create_configuration_for_video_encoding(sc, profile_ab, level, bitrate);
    create_configuration_for_h26x_encoding(sc, options);
}

void create_configuration_for_rm_depth_longthrow(std::vector<uint8_t>& sc, uint8_t mode, uint8_t divisor, uint8_t png_filter)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_divisor(sc, divisor);
    create_configuration_for_png_encoding(sc, png_filter);
}

void create_configuration_for_rm_imu(std::vector<uint8_t>& sc, uint8_t mode)
{
    create_configuration_for_mode(sc, mode);
}

void create_configuration_for_pv(std::vector<uint8_t>& sc, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_resolution(sc, width, height);
    create_configuration_for_framerate(sc, framerate);
    create_configuration_for_video_divisor(sc, divisor);
    create_configuration_for_video_encoding(sc, profile, level, bitrate);
    create_configuration_for_h26x_encoding(sc, options);
}

void create_configuration_for_microphone(std::vector<uint8_t>& sc, uint8_t profile, uint8_t level)
{
    create_configuration_for_audio_encoding(sc, profile, level);
}

void create_configuration_for_si(std::vector<uint8_t>& sc)
{
    (void)sc;
}

void create_configuration_for_eet(std::vector<uint8_t>& sc, uint8_t framerate)
{
    create_configuration_for_framerate(sc, framerate);
}

void create_configuration_for_extended_audio(std::vector<uint8_t>& sc, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level)
{
    create_configuration_for_mrc_audio(sc, mixer_mode, loopback_gain, microphone_gain);
    create_configuration_for_audio_encoding(sc, profile, level);
}

void create_configuration_for_extended_depth(std::vector<uint8_t>& sc, uint8_t mode, uint8_t divisor, uint8_t profile_z, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_divisor(sc, divisor);
    create_configuration_for_depth_encoding(sc, profile_z);
    create_configuration_for_h26x_encoding(sc, options);
}

void create_configuration_for_rm_mode_2(std::vector<uint8_t>& sc, uint8_t mode)
{
    create_configuration_for_mode(sc, mode);
}

void create_configuration_for_pv_mode_2(std::vector<uint8_t>& sc, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate)
{
    create_configuration_for_mode(sc, mode);
    create_configuration_for_video_resolution(sc, width, height);
    create_configuration_for_framerate(sc, framerate);
}

uint32_t extended_audio_device_mixer_mode(uint32_t mixer_mode, uint32_t device)
{
    uint32_t const DEVICE_BASE = 0x00000004;
    return mixer_mode | (DEVICE_BASE * (device + 1));
}

//------------------------------------------------------------------------------
// * PV Control
//------------------------------------------------------------------------------

namespace pv_control
{
uint8_t const START = 0x04;
uint8_t const STOP  = 0x08;
};

void start_subsystem_pv(char const* host, uint16_t port, bool enable_mrc, bool hologram_composition, bool recording_indicator, bool video_stabilization, bool blank_protected, bool show_mesh, bool shared, float global_opacity, float output_width, float output_height, uint32_t video_stabilization_length, uint32_t hologram_perspective)
{
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_mode(sc, pv_control::START | stream_mode::MODE_3);
    create_configuration_for_mrc_video(sc, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.close();
}

void stop_subsystem_pv(char const* host, uint16_t port)
{
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_mode(sc, pv_control::STOP | stream_mode::MODE_3);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.close();
}

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition
//------------------------------------------------------------------------------

rx::rx(char const* host, uint16_t port, uint64_t chunk, uint8_t mode)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
}

rx::~rx()
{
}

void rx::open()
{
    create_configuration(m_sc);
    m_client.open(host.c_str(), port, chunk, mode);
    m_client.sendall(m_sc.data(), m_sc.size());
    m_sc.clear();
}

std::shared_ptr<packet> rx::get_next_packet()
{
    return m_client.get_next_packet();
}

void rx::close()
{
    return m_client.close();
}

rx_rm_vlc::rx_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
{
    this->divisor = divisor;
    this->profile = profile;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}

void rx_rm_vlc::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_vlc(sc, mode, divisor, profile, level, bitrate, options);
}

rx_rm_depth_ahat::rx_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
{
    this->divisor = divisor;
    this->profile_z = profile_z;
    this->profile_ab = profile_ab;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}

void rx_rm_depth_ahat::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_depth_ahat(sc, mode, divisor, profile_z, profile_ab, level, bitrate, options);
}

rx_rm_depth_longthrow::rx_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter) : rx(host, port, chunk, mode)
{
    this->divisor = divisor;
    this->png_filter = png_filter;
}

void rx_rm_depth_longthrow::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_depth_longthrow(sc, mode, divisor, png_filter);
}

rx_rm_imu::rx_rm_imu(char const* host, uint16_t port, uint64_t chunk, uint8_t mode) : rx(host, port, chunk, mode)
{
}

void rx_rm_imu::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_imu(sc, mode);
}

rx_pv::rx_pv(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
{
    this->width = width;
    this->height = height;
    this->framerate = framerate;
    this->divisor = divisor;
    this->profile = profile;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}

void rx_pv::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_pv(sc, mode, width, height, framerate, divisor, profile, level, bitrate, options);
}

rx_microphone::rx_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level) : rx(host, port, chunk, stream_mode::MODE_0)
{
    this->profile = profile;
    this->level = level;
}

void rx_microphone::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_microphone(sc, profile, level);
}

rx_si::rx_si(char const* host, uint16_t port, uint64_t chunk) : rx(host, port, chunk, stream_mode::MODE_0)
{
}

void rx_si::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_si(sc);
}

rx_eet::rx_eet(char const* host, uint16_t port, uint64_t chunk, uint8_t framerate) : rx(host, port, chunk, stream_mode::MODE_1)
{
    this->framerate = framerate;
}

void rx_eet::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_eet(sc, framerate);
}

rx_extended_audio::rx_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level) : rx(host, port, chunk, stream_mode::MODE_0)
{
    this->mixer_mode = mixer_mode;
    this->loopback_gain = loopback_gain;
    this->microphone_gain = microphone_gain;
    this->profile = profile;
    this->level = level;
}

void rx_extended_audio::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_extended_audio(sc, mixer_mode, loopback_gain, microphone_gain, profile, level);
}

rx_extended_depth::rx_extended_depth(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
{
    this->divisor = divisor;
    this->profile_z = profile_z;
    this->options = options;
}

void rx_extended_depth::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_extended_depth(sc, mode, divisor, profile_z, options);
}

//------------------------------------------------------------------------------
// * Frame
//------------------------------------------------------------------------------

frame::frame()
{
    av_frame = av_frame_alloc();
    if (av_frame == NULL) { throw std::runtime_error("hl2ss::frame::init_frame : av_frame_alloc failed"); }
}

frame::~frame()
{
    if (av_frame != NULL) { av_frame_free(&av_frame); }
}

//------------------------------------------------------------------------------
// * Codec
//------------------------------------------------------------------------------

AVCodecID get_video_codec_id(uint8_t profile)
{
    switch (profile)
    {
    case video_profile::H264_BASE:
    case video_profile::H264_MAIN:
    case video_profile::H264_HIGH: return AV_CODEC_ID_H264;
    case video_profile::H265_MAIN: return AV_CODEC_ID_HEVC;
    default:                       return AV_CODEC_ID_NONE;
    }
}

AVCodecID get_audio_codec_id(uint8_t profile)
{
    switch (profile)
    {
    case audio_profile::AAC_12000:
    case audio_profile::AAC_16000:
    case audio_profile::AAC_20000:
    case audio_profile::AAC_24000: return AV_CODEC_ID_AAC;
    default:                       return AV_CODEC_ID_NONE;
    }
}

uint32_t get_audio_codec_bitrate(uint8_t profile)
{
    switch (profile)
    {
    case audio_profile::AAC_12000: return 12000UL*8UL;
    case audio_profile::AAC_16000: return 16000UL*8UL;
    case audio_profile::AAC_20000: return 20000UL*8UL;
    case audio_profile::AAC_24000: return 24000UL*8UL;
    default:                       return 48000UL*2UL*2UL*8UL;
    }
}

codec::codec()
{
    m_c = NULL; 
    m_avpkt = NULL;  
}

codec::~codec()
{
    close();    
}

void codec::open(AVCodecID id)
{
    m_avpkt = av_packet_alloc();
    if (m_avpkt == NULL) { throw std::runtime_error("hl2ss::codec::codec : av_packet_alloc failed"); }

    AVCodec const* decoder = avcodec_find_decoder(id);
    if (decoder == NULL) { throw std::runtime_error("hl2ss::codec::codec : avcodec_find_decoder failed"); }

    m_c = avcodec_alloc_context3(decoder);
    if (m_c == NULL) { throw std::runtime_error("hl2ss::codec::codec : avcodec_alloc_context3 failed"); }

    int status = avcodec_open2(m_c, decoder, NULL);
    if (status < 0) { throw std::runtime_error("hl2ss::codec::codec : avcodec_open2 failed"); }
}

std::shared_ptr<frame> codec::decode(uint8_t* payload, uint32_t size)
{
    std::shared_ptr<frame> f = std::make_shared<frame>();

    m_avpkt->size = size;
    m_avpkt->data = payload;

    int status = avcodec_send_packet(m_c, m_avpkt);
    if (status < 0) { throw std::runtime_error("hl2ss::codec::codec : avcodec_send_packet failed"); }

    status = avcodec_receive_frame(m_c, f->av_frame);
    if (status < 0) { throw std::runtime_error("hl2ss::codec::codec : avcodec_receive_frame failed"); }

    return f;
}

void codec::close()
{
    if (m_c != NULL) { avcodec_free_context(&m_c); }
    if (m_avpkt != NULL) { av_packet_free(&m_avpkt); }
}

//------------------------------------------------------------------------------
// * Decoders
//------------------------------------------------------------------------------

void trim_plane(uint8_t* dst, uint8_t const* src, uint16_t height, uint16_t width, uint16_t stride)
{
    if (width == stride)
    {
    memcpy(dst, src, height * width);
    return;
    }

    for (uint16_t h = 0; h < height; ++h)
    {
    memcpy(dst, src + (h * stride), width);
    dst += width;
    }
}

void collect_i420(uint8_t* dst, int width, int height, uint8_t* data[8], int linesize[8])
{
    trim_plane(dst,                           data[0], height,     width,     linesize[0]);
    trim_plane(dst + (   height * width),     data[1], height / 2, width / 2, linesize[1]);
    trim_plane(dst + (5 *height * width) / 4, data[2], height / 2, width / 2, linesize[2]);
}

void collect_nv12(uint8_t* dst, uint16_t width, uint16_t height, uint8_t const* src, uint16_t stride)
{
    trim_plane(dst,                    src,                     height,     width, stride);
    trim_plane(dst + (height * width), src + (height * stride), height / 2, width, stride);
}

constexpr uint16_t get_video_stride(uint16_t width)
{
    return (width + 63) & ~63;
}

int const cv_format[5][4] = 
{
    { 3, CV_8UC3, cv::COLOR_YUV2BGR_I420,  cv::COLOR_YUV2BGR_NV12  },
    { 3, CV_8UC3, cv::COLOR_YUV2RGB_I420,  cv::COLOR_YUV2RGB_NV12  },
    { 4, CV_8UC4, cv::COLOR_YUV2BGRA_I420, cv::COLOR_YUV2BGRA_NV12 },
    { 4, CV_8UC4, cv::COLOR_YUV2RGBA_I420, cv::COLOR_YUV2RGBA_NV12 },
    { 1, CV_8UC1, cv::COLOR_YUV2GRAY_I420, cv::COLOR_YUV2GRAY_NV12 },
};

void decoder_rm_vlc::open(uint8_t profile)
{
    m_profile = profile;
    if (m_profile != video_profile::RAW) { m_codec.open(get_video_codec_id(m_profile)); }
}

std::unique_ptr<uint8_t[]> decoder_rm_vlc::decode(uint8_t* data, uint32_t size)
{
    uint32_t const h26x_size = size - METADATA_SIZE;
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(DECODED_SIZE + METADATA_SIZE);
    if (m_profile != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data, h26x_size);
    memcpy(out.get(), f->av_frame->data[0], DECODED_SIZE);
    memcpy(out.get() + DECODED_SIZE, data + h26x_size, METADATA_SIZE);
    }
    else
    {
    memcpy(out.get(), data, size);
    }
    return out;
}

void decoder_rm_vlc::close()
{
    if (m_profile != video_profile::RAW) { m_codec.close(); }
}

void decoder_rm_depth_ahat::open(uint8_t profile_z, uint8_t profile_ab)
{
    m_profile_z = profile_z;
    m_profile_ab = profile_ab;
    if (m_profile_ab != video_profile::RAW) { m_codec.open(get_video_codec_id(profile_ab)); }
}

std::unique_ptr<uint8_t[]> decoder_rm_depth_ahat::decode(uint8_t* data, uint32_t size)
{
    uint32_t const base = 2 * sizeof(uint32_t);
    uint32_t const data_size = size - METADATA_SIZE - base;
    
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(DECODED_SIZE + METADATA_SIZE);
    uint32_t sz = DECODED_SIZE / 2;
    uint8_t* dst_z  = out.get();
    uint8_t* dst_ab = dst_z + sz;
    uint8_t* dst_md = dst_ab + sz;
    
    if (m_profile_z == depth_profile::SAME)
    {
    if (m_profile_ab != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data + base, data_size);

    cv::Mat depth = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, dst_z);
    cv::Mat ab    = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, dst_ab);

    cv::Mat y  = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH,     CV_8UC1, f->av_frame->data[0]);
    cv::Mat u  = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[1]);
    cv::Mat v  = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[2]);
    cv::Mat uv = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC2);
    cv::Mat cc = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, uv.data);
    cv::Mat bb = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH / 2, CV_16UC1);
    
    y.convertTo(depth, CV_16UC1, 4);
    cv::merge(std::vector<cv::Mat>{u, v}, uv);
    cc.convertTo(bb, CV_16UC1);
    cv::multiply(bb, bb, bb);
    cv::resize(bb, ab, ab.size(), 0, 0, cv::INTER_NEAREST);

    memcpy(dst_md, data + base + data_size, METADATA_SIZE);
    }
    else
    {
    memcpy(out.get(), data + base, size - base);
    }
    }
    else
    {
#ifdef HL2SS_ENABLE_ZDEPTH
    uint32_t sz_depth;
    uint32_t sz_ab;

    memcpy(&sz_depth, data,                    sizeof(sz_depth));
    memcpy(&sz_ab,    data + sizeof(uint32_t), sizeof(sz_ab));

    uint8_t* data_z  = data    + base;
    uint8_t* data_ab = data_z  + sz_depth;
    uint8_t* data_md = data_ab + sz_ab;

    if (sz_depth > 0)
    {
    std::vector<uint8_t> v_z(data_z, data_ab);
    int w;
    int h;
    std::vector<uint16_t> v_d;

    zdepth::DepthResult result = m_zdc.Decompress(v_z, w, h, v_d);
    if (result != zdepth::DepthResult::Success) { throw std::runtime_error("hl2ss::decoder_rm_depth_ahat::decode : zdepth::DepthCompressor::Decompress failed"); }
    
    memcpy(dst_z, v_d.data(), sz);
    }
    else
    {
    memset(dst_z, 0, sz);
    }

    if (m_profile_ab != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data_ab, sz_ab);

    cv::Mat y  = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_8UC1,  f->av_frame->data[0]);
    cv::Mat ab = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, dst_ab);

    y.convertTo(ab, CV_16UC1);
    cv::multiply(ab, ab, ab);
    }
    else
    {
    memcpy(dst_ab, data_ab, sz);
    }

    memcpy(dst_md, data_md, METADATA_SIZE);
#else
    throw std::runtime_error("hl2ss::decoder_rm_depth_ahat::decode : ZDEPTH decompression not implemented");
#endif
    }
    return out;
}

void decoder_rm_depth_ahat::close()
{
    if (m_profile_ab != video_profile::RAW) { m_codec.close(); }
}

void decoder_rm_depth_longthrow::open()
{
}

std::unique_ptr<uint8_t[]> decoder_rm_depth_longthrow::decode(uint8_t* data, uint32_t size)
{
    uint32_t const png_size = size - METADATA_SIZE;
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(DECODED_SIZE + METADATA_SIZE);
    cv::Mat output = cv::Mat(parameters_rm_depth_longthrow::HEIGHT, parameters_rm_depth_longthrow::WIDTH, CV_8UC4, out.get());
    cv::imdecode(cv::Mat(1, png_size, CV_8UC1, data), cv::IMREAD_UNCHANGED, &output);
    memcpy(out.get() + DECODED_SIZE, data + png_size, METADATA_SIZE);
    return out;
}

void decoder_rm_depth_longthrow::close()
{
}

uint8_t decoder_pv::decoded_bpp(uint8_t decoded_format)
{
    return cv_format[decoded_format][0];
}

int decoder_pv::decoded_cv_type(uint8_t decoded_format)
{
    return cv_format[decoded_format][1];
}

int decoder_pv::decoded_cv_i420(uint8_t decoded_format)
{
    return cv_format[decoded_format][2];
}

int decoder_pv::decoded_cv_nv12(uint8_t decoded_format)
{
    return cv_format[decoded_format][3];
}

void decoder_pv::resolution(uint32_t bytes, uint16_t& width, uint16_t& height, uint16_t& stride)
{
    switch (bytes)
    {
    case get_video_stride(1952)*1100: width = 1952; height = 1100; stride = get_video_stride(1952); break;
    case get_video_stride(1504)* 846: width = 1504; height =  846; stride = get_video_stride(1504); break;
    case get_video_stride(1920)*1080: width = 1920; height = 1080; stride = get_video_stride(1920); break;
    case get_video_stride(1280)* 720: width = 1280; height =  720; stride = get_video_stride(1280); break;
    case get_video_stride( 640)* 360: width =  640; height =  360; stride = get_video_stride( 640); break;
    case get_video_stride( 760)* 428: width =  760; height =  428; stride = get_video_stride( 760); break;
    case get_video_stride( 960)* 540: width =  960; height =  540; stride = get_video_stride( 960); break;
    case get_video_stride(1128)* 636: width = 1128; height =  636; stride = get_video_stride(1128); break;
    case get_video_stride( 424)* 240: width =  424; height =  240; stride = get_video_stride( 424); break;
    case get_video_stride( 500)* 282: width =  500; height =  282; stride = get_video_stride( 500); break;    
    case 1952*1100: width = 1952; height = 1100; stride = 1952; break;
    case 1504* 846: width = 1504; height =  846; stride = 1504; break;
    case  760* 428: width =  760; height =  428; stride =  760; break;
    case 1128* 636: width = 1128; height =  636; stride = 1128; break;
    case  424* 240: width =  424; height =  240; stride =  424; break;
    case  500* 282: width =  500; height =  282; stride =  500; break;
    default: throw std::runtime_error("hl2ss::decoder_pv::resolution : unknown resolution");
    }
}

void decoder_pv::resolution_decoded(uint32_t payload_size, uint8_t decoded_format, uint16_t& width, uint16_t& height, uint8_t& channels)
{
    uint16_t stride;
    channels = decoded_bpp(decoded_format);
    resolution((payload_size - METADATA_SIZE) / channels, width, height, stride);
}

void decoder_pv::open(uint16_t width, uint16_t height, uint8_t profile)
{
    m_width = width;
    m_height = height;
    m_profile = profile;

    if (m_profile != video_profile::RAW) { m_codec.open(get_video_codec_id(m_profile)); }
}

std::unique_ptr<uint8_t[]> decoder_pv::decode(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size, uint16_t& width, uint16_t& height)
{
    uint32_t const h26x_size = size - METADATA_SIZE;
    uint16_t stride;
    cv::Mat input; 
    int code;

    if (m_profile != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data, h26x_size);
    width = f->av_frame->width;
    height = f->av_frame->height;
    input = cv::Mat((height * 3) / 2, width, CV_8UC1);
    collect_i420(input.data, width, height, f->av_frame->data, f->av_frame->linesize);
    code = cv_format[decoded_format][2];
    }
    else
    {
    resolution((h26x_size * 2) / 3, width, height, stride);
    input = cv::Mat((height * 3) / 2, width, CV_8UC1);
    collect_nv12(input.data, width, height, data, stride);
    code = cv_format[decoded_format][3];
    }

    decoded_size = width * height * decoded_bpp(decoded_format);
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(decoded_size + METADATA_SIZE);
    cv::Mat output = cv::Mat(height, width, cv_format[decoded_format][1], out.get());
    cv::cvtColor(input, output, code);
    memcpy(out.get() + decoded_size, data + h26x_size, METADATA_SIZE);

    return out;
}

void decoder_pv::close()
{
    if (m_profile != video_profile::RAW) { m_codec.close(); }
}

void decoder_microphone::open(uint8_t profile)
{
    m_profile = profile;
    if (m_profile != audio_profile::RAW) { m_codec.open(get_audio_codec_id(m_profile)); }
}

std::unique_ptr<uint8_t[]> decoder_microphone::decode(uint8_t* data, uint32_t size)
{
    std::unique_ptr<uint8_t[]> out;
    if (m_profile != audio_profile::RAW)
    { 
    out = std::make_unique<uint8_t[]>(DECODED_SIZE);
    std::shared_ptr<frame> f = m_codec.decode(data, size);
    uint32_t offset = f->av_frame->linesize[0] / 2;
    memcpy(out.get(),          f->av_frame->data[0], offset);
    memcpy(out.get() + offset, f->av_frame->data[1], offset);
    }
    else
    {
    out = std::make_unique<uint8_t[]>(size);
    memcpy(out.get(), data, size);
    }
    return out;
}

void decoder_microphone::close()
{
    if (m_profile != audio_profile::RAW) { m_codec.close(); }
}

void decoder_extended_depth::open(uint8_t profile_z)
{
    m_profile_z = profile_z;
}

std::unique_ptr<uint8_t[]> decoder_extended_depth::decode(uint8_t* data, uint32_t size, uint32_t& decoded_size, uint16_t& width, uint16_t& height)
{
    uint8_t* data_z  = data;
    uint8_t* data_md = data + (size - METADATA_SIZE);
    uint16_t* md = (uint16_t*)data_md; 

    std::unique_ptr<uint8_t[]> out;

    if (m_profile_z == depth_profile::ZDEPTH)
    {
#ifdef HL2SS_ENABLE_ZDEPTH
    std::vector<uint8_t> v_z(data_z, data_md);
    int w;
    int h;
    std::vector<uint16_t> v_d;

    zdepth::DepthResult result = m_zdc.Decompress(v_z, w, h, v_d);
    if (result != zdepth::DepthResult::Success) { throw std::runtime_error("hl2ss::decoder_extended_depth::decode : zdepth::DepthCompressor::Decompress failed"); }
    
    decoded_size = (uint32_t)(v_d.size() * sizeof(uint16_t));
    width = w;
    height = h;

    out = std::make_unique<uint8_t[]>(decoded_size + METADATA_SIZE);
    memcpy(out.get(), v_d.data(), decoded_size);
    memcpy(out.get() + decoded_size, data_md, METADATA_SIZE);
#else
    throw std::runtime_error("hl2ss::decoder_extended_depth::decode : ZDEPTH decompression not implemented");
#endif
    }
    else
    {
    decoded_size = size - METADATA_SIZE;
    width = md[0];
    height = md[1];

    out = std::make_unique<uint8_t[]>(size);
    memcpy(out.get(), data, size);
    }
    
    return out;
}

void decoder_extended_depth::close()
{
}

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

rx_decoded_rm_vlc::rx_decoded_rm_vlc(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)
{
}

void rx_decoded_rm_vlc::open()
{
    m_decoder.open(profile);
    rx_rm_vlc::open();
}

std::shared_ptr<packet> rx_decoded_rm_vlc::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_vlc::get_next_packet();
    if (profile != video_profile::RAW) { p->set_payload(decoder_rm_vlc::DECODED_SIZE + decoder_rm_vlc::METADATA_SIZE, m_decoder.decode(p->payload.get(), p->sz_payload)); }    
    return p;
}

void rx_decoded_rm_vlc::close()
{
    rx_rm_vlc::close();
    m_decoder.close();
}

rx_decoded_rm_depth_ahat::rx_decoded_rm_depth_ahat(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)
{
}

void rx_decoded_rm_depth_ahat::open()
{
    m_decoder.open(profile_z, profile_ab);
    rx_rm_depth_ahat::open();
}

std::shared_ptr<packet> rx_decoded_rm_depth_ahat::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_depth_ahat::get_next_packet();
    if ((profile_z != depth_profile::SAME) || (profile_ab != video_profile::RAW)) { p->set_payload(decoder_rm_depth_ahat::DECODED_SIZE + decoder_rm_depth_ahat::METADATA_SIZE, m_decoder.decode(p->payload.get(), p->sz_payload)); }
    return p;
}

void rx_decoded_rm_depth_ahat::close()
{
    rx_rm_depth_ahat::close();
    m_decoder.close();
}

rx_decoded_rm_depth_longthrow::rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter) : rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)
{
}

void rx_decoded_rm_depth_longthrow::open()
{
    m_decoder.open();
    rx_rm_depth_longthrow::open();
}

std::shared_ptr<packet> rx_decoded_rm_depth_longthrow::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_depth_longthrow::get_next_packet();
    p->set_payload(decoder_rm_depth_longthrow::DECODED_SIZE + decoder_rm_depth_longthrow::METADATA_SIZE, m_decoder.decode(p->payload.get(), p->sz_payload));
    return p;
}

void rx_decoded_rm_depth_longthrow::close()
{
    rx_rm_depth_longthrow::close();
    m_decoder.close();
}

rx_decoded_pv::rx_decoded_pv(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format) : rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)
{
    this->decoded_format = decoded_format;
}

void rx_decoded_pv::open()
{
    m_decoder.open(width, height, profile);
    rx_pv::open();
}

std::shared_ptr<packet> rx_decoded_pv::get_next_packet()
{
    std::shared_ptr<packet> p = rx_pv::get_next_packet();
    uint32_t size;
    std::unique_ptr<uint8_t[]> payload = m_decoder.decode(p->payload.get(), p->sz_payload, decoded_format, size, width, height);
    p->set_payload(size + decoder_pv::METADATA_SIZE, std::move(payload));
    return p;
}

void rx_decoded_pv::close()
{
    rx_pv::close();
    m_decoder.close();
}

rx_decoded_microphone::rx_decoded_microphone(char const* host, uint16_t port, uint64_t chunk, uint8_t profile, uint8_t level) : rx_microphone(host, port, chunk, profile, level)
{
}

void rx_decoded_microphone::open()
{
    m_decoder.open(profile);
    rx_microphone::open();
}

std::shared_ptr<packet> rx_decoded_microphone::get_next_packet()
{
    std::shared_ptr<packet> p = rx_microphone::get_next_packet();
    if (profile != audio_profile::RAW) { p->set_payload(decoder_microphone::DECODED_SIZE, m_decoder.decode(p->payload.get(), p->sz_payload)); }
    return p;
}

void rx_decoded_microphone::close()
{
    rx_microphone::close();
    m_decoder.close();
}

rx_decoded_extended_audio::rx_decoded_extended_audio(char const* host, uint16_t port, uint64_t chunk, uint32_t mixer_mode, float loopback_gain, float microphone_gain, uint8_t profile, uint8_t level) : rx_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level)
{
}

void rx_decoded_extended_audio::open()
{
    m_decoder.open(profile);
    rx_extended_audio::open();
}

std::shared_ptr<packet> rx_decoded_extended_audio::get_next_packet()
{
    std::shared_ptr<packet> p = rx_extended_audio::get_next_packet();
    if (profile != audio_profile::RAW) { p->set_payload(decoder_microphone::DECODED_SIZE, m_decoder.decode(p->payload.get(), p->sz_payload)); }
    return p;
}

void rx_decoded_extended_audio::close()
{
    rx_extended_audio::close();
    m_decoder.close();
}

rx_decoded_extended_depth::rx_decoded_extended_depth(char const* host, uint16_t port, uint64_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, std::vector<uint64_t> const& options) : rx_extended_depth(host, port, chunk, mode, divisor, profile_z, options)
{
    this->width = 0;
    this->height = 0;
}

void rx_decoded_extended_depth::open()
{
    m_decoder.open(profile_z);
    rx_extended_depth::open();
}

std::shared_ptr<packet> rx_decoded_extended_depth::get_next_packet()
{
    std::shared_ptr<packet> p = rx_extended_depth::get_next_packet();
    uint32_t size;
    std::unique_ptr<uint8_t[]> payload = m_decoder.decode(p->payload.get(), p->sz_payload, size, width, height);
    p->set_payload(size + decoder_extended_depth::METADATA_SIZE, std::move(payload));
    return p;
}

void rx_decoded_extended_depth::close()
{
    rx_extended_depth::close();
    m_decoder.close();
}

//------------------------------------------------------------------------------
// * Mode 2 Data Acquisition
//------------------------------------------------------------------------------

std::shared_ptr<calibration_rm_vlc> download_calibration_rm_vlc(char const* host, uint16_t port)
{
    std::shared_ptr<calibration_rm_vlc> data = std::make_shared<calibration_rm_vlc>();
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_rm_mode_2(sc, stream_mode::MODE_2);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(data.get(), sizeof(calibration_rm_vlc), chunk_size::SINGLE_TRANSFER);    
    c.close();

    return data;
}

std::shared_ptr<calibration_rm_depth_ahat> download_calibration_rm_depth_ahat(char const* host, uint16_t port)
{
    std::shared_ptr<calibration_rm_depth_ahat> data = std::make_shared<calibration_rm_depth_ahat>();
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_rm_mode_2(sc, stream_mode::MODE_2);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(data.get(), sizeof(calibration_rm_depth_ahat), chunk_size::SINGLE_TRANSFER);    
    c.close();

    return data;
}

std::shared_ptr<calibration_rm_depth_longthrow> download_calibration_rm_depth_longthrow(char const* host, uint16_t port)
{
    std::shared_ptr<calibration_rm_depth_longthrow> data = std::make_shared<calibration_rm_depth_longthrow>();
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_rm_mode_2(sc, stream_mode::MODE_2);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(data.get(), sizeof(calibration_rm_depth_longthrow), chunk_size::SINGLE_TRANSFER);    
    c.close();

    return data;
}

std::shared_ptr<calibration_rm_imu> download_calibration_rm_imu(char const* host, uint16_t port)
{
    std::shared_ptr<calibration_rm_imu> data = std::make_shared<calibration_rm_imu>();
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_rm_mode_2(sc, stream_mode::MODE_2);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(data.get(), sizeof(calibration_rm_imu), chunk_size::SINGLE_TRANSFER);
    c.close();

    return data;
}

std::shared_ptr<calibration_pv> download_calibration_pv(char const* host, uint16_t port, uint16_t width, uint16_t height, uint8_t framerate)
{
    std::shared_ptr<calibration_pv> data = std::make_shared<calibration_pv>();
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_pv_mode_2(sc, stream_mode::MODE_2, width, height, framerate);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(data.get(), sizeof(calibration_pv), chunk_size::SINGLE_TRANSFER);
    c.close();

    return data;
}

std::shared_ptr<std::vector<uint8_t>> download_devicelist_extended_audio(char const* host, uint16_t port)
{
    std::shared_ptr<std::vector<uint8_t>> data = std::make_shared<std::vector<uint8_t>>();
    uint32_t size;
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_mrc_audio(sc, mixer_mode::QUERY, 1.0, 1.0);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(&size, sizeof(size), chunk_size::SINGLE_TRANSFER);
    data->resize(size);
    c.download(data->data(), size, chunk_size::SINGLE_TRANSFER);
    c.close();

    return data;
}

std::shared_ptr<std::vector<uint8_t>> download_devicelist_extended_video(char const* host, uint16_t port)
{
    std::shared_ptr<std::vector<uint8_t>> data = std::make_shared<std::vector<uint8_t>>();
    uint32_t size;
    std::vector<uint8_t> sc;
    client c;

    create_configuration_for_mode(sc, stream_mode::MODE_2);

    c.open(host, port);
    c.sendall(sc.data(), sc.size());
    c.download(&size, sizeof(size), chunk_size::SINGLE_TRANSFER);
    data->resize(size);
    c.download(data->data(), size, chunk_size::SINGLE_TRANSFER);
    c.close();

    return data;
}

//------------------------------------------------------------------------------
// * Port Information
//------------------------------------------------------------------------------

char const* const port_name[] = {
    "rm_vlc_leftfront",
    "rm_vlc_leftleft",
    "rm_vlc_rightfront", 
    "rm_vlc_rightright",
    "rm_depth_ahat",
    "rm_depth_longthrow", 
    "rm_imu_accelerometer", 
    "rm_imu_gyroscope", 
    "rm_imu_magnetometer", 
    "remote_configuration", 
    "personal_video", 
    "microphone", 
    "spatial_input", 
    "spatial_mapping", 
    "scene_understanding",
    "voice_input",
    "unity_message_queue",
    "extended_eye_tracker",
    "extended_audio",
    "extended_video",
    "guest_message_queue",
    "extended_depth",
};

char const* get_port_name(uint16_t port)
{
    uint16_t index = port - stream_port::RM_VLC_LEFTFRONT;
    return (index < (sizeof(port_name) / sizeof(char*))) ? port_name[index] : NULL;
}

//------------------------------------------------------------------------------
// * IPC
//------------------------------------------------------------------------------

ipc::ipc(char const* host, uint16_t port)
{
    this->host = host;
    this->port = port;
}

ipc::~ipc()
{
}

void ipc::open()
{
    m_client.open(host.c_str(), port);
}

void ipc::close()
{
    m_client.close();
}

//------------------------------------------------------------------------------
// * Remote Configuration
//------------------------------------------------------------------------------

namespace cmd_ipc_rc
{
uint8_t const GET_APPLICATION_VERSION            = 0x00;
uint8_t const GET_UTC_OFFSET                     = 0x01;
uint8_t const SET_HS_MARKER_STATE                = 0x02;
uint8_t const GET_PV_SUBSYSTEM_STATUS            = 0x03;
uint8_t const SET_PV_FOCUS                       = 0x04;
uint8_t const SET_PV_VIDEO_TEMPORAL_DENOISING    = 0x05;
uint8_t const SET_PV_WHITE_BALANCE_PRESET        = 0x06;
uint8_t const SET_PV_WHITE_BALANCE_VALUE         = 0x07;
uint8_t const SET_PV_EXPOSURE                    = 0x08;
uint8_t const SET_PV_EXPOSURE_PRIORITY_VIDEO     = 0x09;
uint8_t const SET_PV_ISO_SPEED                   = 0x0A;
uint8_t const SET_PV_BACKLIGHT_COMPENSATION      = 0x0B;
uint8_t const SET_PV_SCENE_MODE                  = 0x0C;
uint8_t const SET_FLAT_MODE                      = 0x0D;
uint8_t const SET_RM_EYE_SELECTION               = 0x0E;
uint8_t const SET_PV_DESIRED_OPTIMIZATION        = 0x0F;
uint8_t const SET_PV_PRIMARY_USE                 = 0x10;
uint8_t const SET_PV_OPTICAL_IMAGE_STABILIZATION = 0x11;
uint8_t const SET_PV_HDR_VIDEO                   = 0x12;
uint8_t const SET_PV_REGIONS_OF_INTEREST         = 0x13;
uint8_t const SET_INTERFACE_PRIORITY             = 0x14;
uint8_t const SET_QUIET_MODE                     = 0x15;
};

ipc_rc::ipc_rc(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_rc::send(uint8_t command, std::initializer_list<uint32_t> list)
{
    m_sc.clear();
    push_u8(m_sc, command);
    for (uint32_t argument : list) { push_u32(m_sc, argument); }
    m_client.sendall(m_sc.data(), m_sc.size());
}

void ipc_rc::recv(void* buffer, uint64_t size)
{
    m_client.download(buffer, size, chunk_size::SINGLE_TRANSFER);
}

version ipc_rc::get_application_version()
{
    send(cmd_ipc_rc::GET_APPLICATION_VERSION, {});
    version data;
    recv(data.field, sizeof(data.field));
    return data;
}

uint64_t ipc_rc::get_utc_offset()
{
    send(cmd_ipc_rc::GET_UTC_OFFSET, {});
    uint64_t data;
    recv(&data, sizeof(data));
    return data;
}

void ipc_rc::set_hs_marker_state(uint32_t state)
{
    send(cmd_ipc_rc::SET_HS_MARKER_STATE, {state});
}

bool ipc_rc::get_pv_subsystem_status()
{
    send(cmd_ipc_rc::GET_PV_SUBSYSTEM_STATUS, {});
    uint8_t status;
    recv(&status, sizeof(status));
    return status != 0;
}

void ipc_rc::wait_for_pv_subsystem(bool status)
{
    while (get_pv_subsystem_status() != status);
}

void ipc_rc::set_pv_focus(uint32_t mode, uint32_t range, uint32_t distance, uint32_t value, uint32_t driver_fallback)
{
    send(cmd_ipc_rc::SET_PV_FOCUS, {mode, range, distance, value, driver_fallback});
}

void ipc_rc::set_pv_video_temporal_denoising(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_VIDEO_TEMPORAL_DENOISING, {mode});
}

void ipc_rc::set_pv_white_balance_preset(uint32_t preset)
{
    send(cmd_ipc_rc::SET_PV_WHITE_BALANCE_PRESET, {preset});
}

void ipc_rc::set_pv_white_balance_value(uint32_t value)
{
    send(cmd_ipc_rc::SET_PV_WHITE_BALANCE_VALUE, {value});
}

void ipc_rc::set_pv_exposure(uint32_t mode, uint32_t value)
{
    send(cmd_ipc_rc::SET_PV_EXPOSURE, {mode, value});
}

void ipc_rc::set_pv_exposure_priority_video(uint32_t enabled)
{
    send(cmd_ipc_rc::SET_PV_EXPOSURE_PRIORITY_VIDEO, {enabled});
}

void ipc_rc::set_pv_iso_speed(uint32_t mode, uint32_t value)
{
    send(cmd_ipc_rc::SET_PV_ISO_SPEED, {mode, value});
}

void ipc_rc::set_pv_backlight_compensation(uint32_t state)
{
    send(cmd_ipc_rc::SET_PV_BACKLIGHT_COMPENSATION, {state});
}

void ipc_rc::set_pv_scene_mode(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_SCENE_MODE, {mode});
}

void ipc_rc::set_flat_mode(uint32_t mode)
{
    send(cmd_ipc_rc::SET_FLAT_MODE, {mode});
}

void ipc_rc::set_rm_eye_selection(uint32_t enable)
{
    send(cmd_ipc_rc::SET_RM_EYE_SELECTION, {enable});
}

void ipc_rc::set_pv_desired_optimization(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_DESIRED_OPTIMIZATION, {mode});
}

void ipc_rc::set_pv_primary_use(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_PRIMARY_USE, {mode});
}

void ipc_rc::set_pv_optical_image_stabilization(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_OPTICAL_IMAGE_STABILIZATION, {mode});
}

void ipc_rc::set_pv_hdr_video(uint32_t mode)
{
    send(cmd_ipc_rc::SET_PV_HDR_VIDEO, {mode});
}

void ipc_rc::set_pv_regions_of_interest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, uint32_t type, uint32_t weight, float x, float y, float w, float h)
{
    uint32_t mode = (0x1000 * clear) | (0x0800 * set) | (0x0400 * auto_exposure) | (0x0200 * auto_focus) | (0x0100 * bounds_normalized) | ((type & 1) << 7) | (weight & 0x7F);
    send(cmd_ipc_rc::SET_PV_REGIONS_OF_INTEREST, {mode, *(uint32_t*)&x, *(uint32_t*)&y, *(uint32_t*)&w, *(uint32_t*)&h});
}

void ipc_rc::set_interface_priority(uint16_t port, int32_t priority)
{
    send(cmd_ipc_rc::SET_INTERFACE_PRIORITY, { (uint32_t)port, *(uint32_t*)&priority });
}

void ipc_rc::set_quiet_mode(uint32_t mode)
{
    send(cmd_ipc_rc::SET_QUIET_MODE, {mode});
}

//------------------------------------------------------------------------------
// * Spatial Mapping
//------------------------------------------------------------------------------

namespace commmand_ipc_sm
{
uint8_t const SET_VOLUMES           = 0x00;
uint8_t const GET_OBSERVED_SURFACES = 0x01;
uint8_t const GET_MESHES            = 0x02;
};

sm_bounding_volume::sm_bounding_volume()
{
    m_count = 0;
}

sm_bounding_volume::sm_bounding_volume(uint32_t count, uint8_t const* data, uint64_t size)
{
    m_count = count;
    m_data  = { data, data + size };
}

void sm_bounding_volume::clear()
{
    m_count = 0;
    m_data.clear();
}

void sm_bounding_volume::add_box(sm_box box)
{
    m_count++;
    push_u32(m_data, sm_volume_type::Box);
    push(m_data, &box,  sizeof(box));
}

void sm_bounding_volume::add_frustum(sm_frustum frustum)
{
    m_count++;
    push_u32(m_data, sm_volume_type::Frustum);
    push(m_data, &frustum, sizeof(frustum));
}

void sm_bounding_volume::add_oriented_box(sm_oriented_box oriented_box)
{
    m_count++;
    push_u32(m_data, sm_volume_type::OrientedBox);
    push(m_data, &oriented_box, sizeof(oriented_box));
}

void sm_bounding_volume::add_sphere(sm_sphere sphere)
{
    m_count++;
    push_u32(m_data, sm_volume_type::Sphere);
    push(m_data, &sphere, sizeof(sphere));
}

uint32_t sm_bounding_volume::get_count() const
{
    return m_count;
}

uint8_t const* sm_bounding_volume::get_data() const
{
    return m_data.data();
}

uint64_t sm_bounding_volume::get_size() const
{
    return m_data.size();
}

sm_mesh_task::sm_mesh_task()
{
    m_count = 0;
}

sm_mesh_task::sm_mesh_task(uint32_t count, uint8_t const* data, uint64_t size)
{
    m_count = count;
    m_data  = { data, data + size };
}

void sm_mesh_task::clear()
{
    m_count = 0;
    m_data.clear();
}

void sm_mesh_task::add_task(guid id, double max_triangles_per_cubic_meter, uint32_t vertex_position_format, uint32_t triangle_index_format, uint32_t vertex_normal_format)
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

uint32_t sm_mesh_task::get_count() const
{
    return m_count;
}

uint8_t const* sm_mesh_task::get_data() const
{
    return m_data.data();
}

uint64_t sm_mesh_task::get_size() const
{
    return m_data.size();
}

ipc_sm::ipc_sm(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_sm::set_volumes(sm_bounding_volume const& volumes)
{
    std::vector<uint8_t> sc;
    push_u8(sc, commmand_ipc_sm::SET_VOLUMES);
    push_u8(sc, (uint8_t)volumes.get_count());
    push(sc, volumes.get_data(), volumes.get_size());
    m_client.sendall(sc.data(), sc.size());
}

void ipc_sm::get_observed_surfaces(std::vector<sm_surface_info>& surfaces)
{
    uint8_t c = commmand_ipc_sm::GET_OBSERVED_SURFACES;
    m_client.sendall(&c, sizeof(c));
    uint32_t count;
    m_client.download(&count, sizeof(count), chunk_size::SINGLE_TRANSFER);
    surfaces.resize(count);
    m_client.download(surfaces.data(), count * sizeof(sm_surface_info), chunk_size::SINGLE_TRANSFER);
}

void ipc_sm::get_meshes(sm_mesh_task const& tasks, std::vector<sm_mesh>& meshes)
{
    std::vector<uint8_t> sc;
    push_u8(sc, commmand_ipc_sm::GET_MESHES);
    push_u32(sc, tasks.get_count());
    push(sc, tasks.get_data(), tasks.get_size());
    m_client.sendall(sc.data(), sc.size());

    meshes.resize(tasks.get_count());

    for (uint32_t i = 0; i < tasks.get_count(); ++i)
    {
    uint32_t index;
    
    m_client.download(&index,                       sizeof(index),                      chunk_size::SINGLE_TRANSFER);

    sm_mesh& mesh = meshes[index];

    m_client.download(&mesh.status,                 sizeof(mesh.status),                chunk_size::SINGLE_TRANSFER);

    uint32_t vpl;
    uint32_t til;
    uint32_t vnl;
    uint32_t bsz = 40;

    m_client.download(&vpl,                         sizeof(vpl),                        chunk_size::SINGLE_TRANSFER);
    m_client.download(&til,                         sizeof(til),                        chunk_size::SINGLE_TRANSFER);
    m_client.download(&vnl,                         sizeof(vnl),                        chunk_size::SINGLE_TRANSFER);
    m_client.download(&mesh.vertex_position_scale,  sizeof(mesh.vertex_position_scale), chunk_size::SINGLE_TRANSFER);
    m_client.download(&mesh.pose,                   sizeof(mesh.pose),                  chunk_size::SINGLE_TRANSFER);

    mesh.bounds.resize(bsz);
    mesh.vertex_positions.resize(vpl);
    mesh.triangle_indices.resize(til);
    mesh.vertex_normals.resize(vnl);

    m_client.download(mesh.bounds.data(),           bsz,                                chunk_size::SINGLE_TRANSFER);
    m_client.download(mesh.vertex_positions.data(), vpl,                                chunk_size::SINGLE_TRANSFER);
    m_client.download(mesh.triangle_indices.data(), til,                                chunk_size::SINGLE_TRANSFER);
    m_client.download(mesh.vertex_normals.data(),   vnl,                                chunk_size::SINGLE_TRANSFER);
    }
}

//------------------------------------------------------------------------------
// * Scene Understanding
//------------------------------------------------------------------------------

void su_pack_task(std::vector<uint8_t>& sc, su_task const& task)
{
    push_u8(sc, task.enable_quads);
    push_u8(sc, task.enable_meshes);
    push_u8(sc, task.enable_only_observed);
    push_u8(sc, task.enable_world_mesh);
    push_u32(sc, task.mesh_lod);
    push_float(sc, task.query_radius);
    push_u8(sc, task.create_mode);
    push_u8(sc, task.kind_flags);
    push_u8(sc, task.get_orientation);
    push_u8(sc, task.get_position);
    push_u8(sc, task.get_location_matrix);
    push_u8(sc, task.get_quad);
    push_u8(sc, task.get_meshes);
    push_u8(sc, task.get_collider_meshes);
    push_u32(sc, (uint32_t)task.guid_list.size());
    push(sc, task.guid_list.data(), task.guid_list.size() * sizeof(guid));
}

ipc_su::ipc_su(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_su::download_meshes(std::vector<su_mesh>& meshes)
{
    uint32_t count;
    m_client.download(&count, sizeof(count), chunk_size::SINGLE_TRANSFER);
    meshes.resize(count);
    
    for (uint32_t i = 0; i < count; ++i)
    {
    su_mesh& mesh = meshes[i];

    uint32_t elements[2];
    m_client.download(elements, sizeof(elements), chunk_size::SINGLE_TRANSFER);

    uint32_t vpl = elements[0] * sizeof(float);
    uint32_t til = elements[1] * sizeof(uint32_t);

    mesh.vertex_positions.resize(vpl);
    mesh.triangle_indices.resize(til);

    m_client.download(mesh.vertex_positions.data(), vpl, chunk_size::SINGLE_TRANSFER);
    m_client.download(mesh.triangle_indices.data(), til, chunk_size::SINGLE_TRANSFER);
    }
}

void ipc_su::query(su_task const& task, su_result& result)
{
    std::vector<uint8_t> sc;
    su_pack_task(sc, task);
    m_client.sendall(sc.data(), sc.size());

    m_client.download(&result.status,     sizeof(result.status),     chunk_size::SINGLE_TRANSFER);
    if (result.status != 0) { return; }

    uint32_t items;

    m_client.download(&result.extrinsics, sizeof(result.extrinsics), chunk_size::SINGLE_TRANSFER);
    m_client.download(&result.pose,       sizeof(result.pose),       chunk_size::SINGLE_TRANSFER);
    m_client.download(&items,             sizeof(items),             chunk_size::SINGLE_TRANSFER);

    result.items.resize(items);

    for (uint32_t i = 0; i < items; ++i)
    {
    su_item& item = result.items[i];

    m_client.download(&item.id,          sizeof(item.id),                                     chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.kind,        sizeof(item.kind),                                   chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.orientation, sizeof(item.orientation) * task.get_orientation,     chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.position,    sizeof(item.position)    * task.get_position,        chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.location,    sizeof(item.location)    * task.get_location_matrix, chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.alignment,   sizeof(item.alignment)   * task.get_quad,            chunk_size::SINGLE_TRANSFER);
    m_client.download(&item.extents,     sizeof(item.extents)     * task.get_quad,            chunk_size::SINGLE_TRANSFER);

    if (task.get_meshes)          { download_meshes(item.meshes); }
    if (task.get_collider_meshes) { download_meshes(item.collider_meshes); }
    }
}

//------------------------------------------------------------------------------
// * Voice Input
//------------------------------------------------------------------------------

namespace commmand_ipc_vi
{
uint8_t const POP  = 0x01;
uint8_t const STOP = 0x00;
};

ipc_vi::ipc_vi(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_vi::start(std::vector<std::u16string> const& strings)
{
    std::vector<uint8_t> data;

    push_u16(data, (uint16_t)strings.size());

    for (std::u16string string : strings)
    {
    uint16_t len = (uint16_t)(string.length() * sizeof(char16_t));
    push_u16(data, len);
    push(data, string.c_str(), len);
    }

    m_client.sendall(data.data(), data.size());
}

void ipc_vi::pop(std::vector<vi_result>& results)
{
    uint8_t c = commmand_ipc_vi::POP;
    uint32_t count;

    m_client.sendall(&c, sizeof(c));
    m_client.download(&count, sizeof(count), chunk_size::SINGLE_TRANSFER);
    results.resize(count);
    m_client.download(results.data(), count * sizeof(vi_result), chunk_size::SINGLE_TRANSFER);
}

void ipc_vi::stop()
{
    uint8_t c = commmand_ipc_vi::STOP;
    m_client.sendall(&c, sizeof(c));
}

//------------------------------------------------------------------------------
// * Unity Message Queue
//------------------------------------------------------------------------------

umq_command_buffer::umq_command_buffer()
{
    m_count = 0;
}

umq_command_buffer::umq_command_buffer(uint32_t count, uint8_t const* data, uint64_t size)
{
    m_count = count;
    m_buffer = { data, data + size };
}

void umq_command_buffer::clear()
{
    m_count = 0;
    m_buffer.clear();
}

void umq_command_buffer::add(uint32_t id, void const* data, uint64_t size)
{
    push_u32(m_buffer, id);
    push_u32(m_buffer, (uint32_t)size);
    push(m_buffer, data, size);
    m_count++;
}

uint32_t umq_command_buffer::get_count()
{
    return m_count;
}

uint8_t const* umq_command_buffer::get_data()
{
    return m_buffer.data();
}

uint64_t umq_command_buffer::get_size()
{
    return m_buffer.size();
}

ipc_umq::ipc_umq(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_umq::push(uint8_t const* data, uint64_t size)
{
    m_client.sendall(data, size);
}

void ipc_umq::pull(uint32_t* data, uint32_t count)
{
    m_client.download(data, count * sizeof(uint32_t), chunk_size::SINGLE_TRANSFER);
}

//------------------------------------------------------------------------------
// * Guest Message Queue
//------------------------------------------------------------------------------

ipc_gmq::ipc_gmq(char const* host, uint16_t port) : ipc(host, port)
{
}

void ipc_gmq::pull(gmq_message& message)
{
    uint32_t peek = ~0U;
    m_client.sendall(&peek, sizeof(peek));
    m_client.download(&message, 2 * sizeof(uint32_t), chunk_size::SINGLE_TRANSFER);
    message.data = nullptr;
    if (message.size <= 0) { return; }
    message.data = std::make_unique<uint8_t[]>(message.size);
    m_client.download(message.data.get(), message.size, chunk_size::SINGLE_TRANSFER);
}

void ipc_gmq::push(uint32_t const* response, uint32_t count)
{
    m_client.sendall(response, count * sizeof(uint32_t));
}

//------------------------------------------------------------------------------
// * Unpacking
//------------------------------------------------------------------------------

map_rm_vlc unpack_rm_vlc(uint8_t* payload)
{
    return { payload, (rm_vlc_metadata*)(payload + parameters_rm_vlc::PIXELS) };
}

map_rm_depth_ahat unpack_rm_depth_ahat(uint8_t* payload)
{
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t))), (rm_depth_ahat_metadata*)(payload + (2 * parameters_rm_depth_ahat::PIXELS * sizeof(uint16_t))) };
}

map_rm_depth_longthrow unpack_rm_depth_longthrow(uint8_t* payload)
{
    return { (uint16_t*)(payload), (uint16_t*)(payload + (parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t))), (rm_depth_longthrow_metadata*)(payload + (2 * parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t))) };
}

map_rm_imu unpack_rm_imu(uint8_t* payload)
{
    return { (rm_imu_sample*)payload };
}

map_pv unpack_pv(uint8_t* payload, uint32_t size)
{
    return { payload, (pv_metadata*)(payload + size - sizeof(pv_metadata)) };
}

map_microphone_raw unpack_microphone_raw(uint8_t* payload)
{
    return { (int16_t*)payload };
}

map_microphone_aac unpack_microphone_aac(uint8_t* payload)
{
    return { (float*)payload };
}

map_microphone_array unpack_microphone_array(uint8_t* payload)
{
    return { (float*)payload };
}

map_si unpack_si(uint8_t* payload)
{
    return { (si_frame*)payload };
}

map_eet unpack_eet(uint8_t* payload)
{
    return { (eet_frame*)payload };
}

map_extended_audio_raw unpack_extended_audio_raw(uint8_t* payload)
{
    return { (int16_t*)payload };
}

map_extended_audio_aac unpack_extended_audio_aac(uint8_t* payload)
{
    return { (float*)payload };
}

map_extended_depth unpack_extended_depth(uint8_t* payload, uint32_t size)
{
    return { (uint16_t*)payload, (extended_depth_metadata*)(payload + size - sizeof(extended_depth_metadata)) };
}
}
