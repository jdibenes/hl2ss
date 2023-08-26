
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>
#include "hl2ss.h"
#include "types.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace hl2ss
{
//------------------------------------------------------------------------------
// Network Client
//------------------------------------------------------------------------------

client::client()
{
    m_socket = -1;
}

client::~client()
{
    close();
}

void client::open(char const* host, uint16_t port)
{
    m_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket < 0) { throw std::runtime_error("hl2ss::client::open : socket failed"); }

    sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    int status = inet_pton(AF_INET, host, &addr.sin_addr);
    if (status <= 0) { throw std::runtime_error("hl2ss::client::open : inet_pton failed"); }

    status = connect(m_socket, (sockaddr*)&addr, sizeof(addr));
    if (status < 0) { throw std::runtime_error("hl2ss::client::open : connect failed"); }
}

void client::sendall(char const* data, size_t count)
{
    ssize_t sent = 0;
    while (count > 0)
    {
    ssize_t bytes = send(m_socket, data + sent, count, 0);
    if (bytes < 0) { throw std::runtime_error("hl2ss::client::sendall : send failed"); }
    count -= bytes;
    sent  += bytes;
    }
}

size_t client::recv(char* buffer, size_t count)
{
    size_t bytes = ::recv(m_socket, buffer, count, 0);
    if ((bytes <= 0) && (count > 0)) { throw std::runtime_error("hl2ss::client::recv : recv failed"); }
    return bytes;
}

void client::download(char* buffer, size_t total, size_t chunk_size)
{
    size_t received = 0;
    while (total > 0)
    {
    if (chunk_size > total) { chunk_size = total; }
    size_t bytes = recv(buffer + received, chunk_size);
    total    -= bytes;
    received += bytes;
    }
}

void client::close()
{
    if (m_socket < 0) { return; }
    ::close(m_socket);
    m_socket = -1;
}

//------------------------------------------------------------------------------
// Packet
//------------------------------------------------------------------------------

packet::packet()
{
    timestamp = 0;
    sz_payload = 0;
    payload = NULL;
    pose = NULL;
}

packet::~packet()
{
    free_payload();
    free_pose();
}

void packet::init_payload()
{
    payload = new uint8_t[sz_payload];
}

void packet::init_pose()
{
    pose = new float[SZ_POSE / sizeof(float)];
}

void packet::free_payload()
{
    if (payload == NULL) { return; }
    delete[] payload;
    payload = NULL;
}

void packet::free_pose()
{
    if (pose == NULL) { return; }
    delete[] pose;
    pose = NULL;
}

//------------------------------------------------------------------------------
// Packet Gatherer
//------------------------------------------------------------------------------

void gatherer::open(char const* host, uint16_t port, size_t chunk_size, uint8_t mode)
{
    m_client.open(host, port);
    m_chunk_size = chunk_size;
    m_mode = mode;
}

void gatherer::sendall(char const* data, size_t count)
{
    m_client.sendall(data, count);
}

std::shared_ptr<packet> gatherer::get_next_packet()
{
    std::shared_ptr<packet> p = std::make_shared<packet>();

    m_client.download((char*)&p->timestamp, sizeof(p->timestamp), m_chunk_size);
    m_client.download((char*)&p->sz_payload, sizeof(p->sz_payload), m_chunk_size);
    p->init_payload();
    m_client.download((char*)p->payload, p->sz_payload, m_chunk_size);
    if (m_mode == stream_mode::MODE_1)
    {
    p->init_pose();
    m_client.download((char*)p->pose, packet::SZ_POSE, m_chunk_size);
    }

    return p;
}

void gatherer::close()
{
    m_client.close();
}

//------------------------------------------------------------------------------
// Stream Configuration
//------------------------------------------------------------------------------

static void create_configuration_for_mode(std::vector<uint8_t>& cfg, uint8_t mode)
{
    v8 _mode;
    
    _mode.b = mode;

    cfg.push_back(_mode.b);
}

static void create_configuration_for_video_format(std::vector<uint8_t>& cfg, uint16_t width, uint16_t height, uint8_t framerate)
{
    v16 _width;
    v16 _height;
    v8  _framerate;

    _width.w = width;
    _height.w = height;
    _framerate.b = framerate;

    cfg.push_back(_width.b.b0.b);
    cfg.push_back(_width.b.b1.b);
    cfg.push_back(_height.b.b0.b);
    cfg.push_back(_height.b.b1.b);
    cfg.push_back(_framerate.b);
}

static void create_configuration_for_video_divisor(std::vector<uint8_t>& cfg, uint8_t divisor)
{
    v8 _divisor;

    _divisor.b = divisor;

    cfg.push_back(_divisor.b);
}

static void create_configuration_for_video_encoding(std::vector<uint8_t>& cfg, uint8_t profile, uint8_t level, uint32_t bitrate)
{
    v8 _profile;
    v8 _level;
    v32 _bitrate;

    _profile.b = profile;
    _level.b = level;
    _bitrate.d = bitrate;

    cfg.push_back(_profile.b);
    cfg.push_back(_level.b);
    cfg.push_back(_bitrate.w.w0.b.b0.b);
    cfg.push_back(_bitrate.w.w0.b.b1.b);
    cfg.push_back(_bitrate.w.w1.b.b0.b);
    cfg.push_back(_bitrate.w.w1.b.b1.b);
}

static void create_configuration_for_depth_encoding(std::vector<uint8_t>& cfg, uint8_t profile)
{
    v8 _profile;

    _profile.b = profile;

    cfg.push_back(_profile.b);
}

static void create_configuration_for_audio_encoding(std::vector<uint8_t>& cfg, uint8_t profile, uint8_t level)
{
    v8 _profile;
    v8 _level;

    _profile.b = profile;
    _level.b = level;

    cfg.push_back(_profile.b);
    cfg.push_back(_level.b);
}

static void create_configuration_for_png_encoding(std::vector<uint8_t>& cfg, uint8_t png_filter)
{
    v8 _png_filter;

    _png_filter.b = png_filter;

    cfg.push_back(_png_filter.b);
}

static void create_configuration_for_h26x_encoding(std::vector<uint8_t>& cfg, std::vector<uint64_t> const& options)
{
    v8 _count;

    _count.b = (uint8_t)(options.size() / 2);

    cfg.push_back(_count.b);

    for (size_t i = 0; i < options.size(); i += 2)
    {
    v64 _index;
    v64 _value;

    _index.q = options[i];
    _value.q = options[i+1];

    cfg.push_back(_index.d.d0.w.w0.b.b0.b);
    cfg.push_back(_index.d.d0.w.w0.b.b1.b);
    cfg.push_back(_index.d.d0.w.w1.b.b0.b);
    cfg.push_back(_index.d.d0.w.w1.b.b1.b);
    cfg.push_back(_index.d.d1.w.w0.b.b0.b);
    cfg.push_back(_index.d.d1.w.w0.b.b1.b);
    cfg.push_back(_index.d.d1.w.w1.b.b0.b);
    cfg.push_back(_index.d.d1.w.w1.b.b1.b);
    cfg.push_back(_value.d.d0.w.w0.b.b0.b);
    cfg.push_back(_value.d.d0.w.w0.b.b1.b);
    cfg.push_back(_value.d.d0.w.w1.b.b0.b);
    cfg.push_back(_value.d.d0.w.w1.b.b1.b);
    cfg.push_back(_value.d.d1.w.w0.b.b0.b);
    cfg.push_back(_value.d.d1.w.w0.b.b1.b);
    cfg.push_back(_value.d.d1.w.w1.b.b0.b);
    cfg.push_back(_value.d.d1.w.w1.b.b1.b);
    }
}

static void create_configuration_for_rm_vlc(std::vector<uint8_t>& cfg, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(cfg, mode);
    create_configuration_for_video_divisor(cfg, divisor);
    create_configuration_for_video_encoding(cfg, profile, level, bitrate);
    create_configuration_for_h26x_encoding(cfg, options);
}

static void create_configuration_for_rm_depth_ahat(std::vector<uint8_t>& cfg, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(cfg, mode);
    create_configuration_for_video_divisor(cfg, divisor);
    create_configuration_for_depth_encoding(cfg, profile_z);
    create_configuration_for_video_encoding(cfg, profile_ab, level, bitrate);
    create_configuration_for_h26x_encoding(cfg, options);
}

static void create_configuration_for_rm_depth_longthrow(std::vector<uint8_t>& cfg, uint8_t mode, uint8_t divisor, uint8_t png_filter)
{
    create_configuration_for_mode(cfg, mode);
    create_configuration_for_video_divisor(cfg, divisor);
    create_configuration_for_png_encoding(cfg, png_filter);
}

static void create_configuration_for_rm_imu(std::vector<uint8_t>& cfg, uint8_t mode)
{
    create_configuration_for_mode(cfg, mode);
}

static void create_configuration_for_pv(std::vector<uint8_t>& cfg, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    create_configuration_for_mode(cfg, mode);
    create_configuration_for_video_format(cfg, width, height, framerate);
    create_configuration_for_video_divisor(cfg, divisor);
    create_configuration_for_video_encoding(cfg, profile, level, bitrate);
    create_configuration_for_h26x_encoding(cfg, options);
}

static void create_configuration_for_microphone(std::vector<uint8_t>& cfg, uint8_t profile, uint8_t level)
{
    create_configuration_for_audio_encoding(cfg, profile, level);
}

static void create_configuration_for_eet(std::vector<uint8_t>& cfg, uint8_t fps)
{
    v8 _fps;

    _fps.b = fps;

    cfg.push_back(_fps.b);
}

static void create_configuration_for_rm_mode2(std::vector<uint8_t>& cfg, uint8_t mode)
{
    create_configuration_for_mode(cfg, mode);
}

static void create_configuration_for_pv_mode2(std::vector<uint8_t>& cfg, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate)
{
    create_configuration_for_mode(cfg, mode);
    create_configuration_for_video_format(cfg, width, height, framerate);
}

//------------------------------------------------------------------------------
// Mode 0 and Mode 1 Data Acquisition
//------------------------------------------------------------------------------

static void connect_client_rm_vlc(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_rm_vlc(cfg, mode, divisor, profile, level, bitrate, options);
    client.open(host, port, chunk_size, mode);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_rm_depth_ahat(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_rm_depth_ahat(cfg, mode, divisor, profile_z, profile_ab, level, bitrate, options);
    client.open(host, port, chunk_size, mode);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_rm_depth_longthrow(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t mode, uint8_t divisor, uint8_t png_filter)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_rm_depth_longthrow(cfg, mode, divisor, png_filter);
    client.open(host, port, chunk_size, mode);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_rm_imu(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t mode)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_rm_imu(cfg, mode);
    client.open(host, port, chunk_size, mode);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_pv(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_pv(cfg, mode, width, height, framerate, divisor, profile, level, bitrate, options);
    client.open(host, port, chunk_size, mode);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_microphone(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t profile, uint8_t level)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_microphone(cfg, profile, level);
    client.open(host, port, chunk_size, stream_mode::MODE_0);
    client.sendall((char*)cfg.data(), cfg.size());
}

static void connect_client_si(gatherer& client, char const* host, uint16_t port, size_t chunk_size)
{
    client.open(host, port, chunk_size, stream_mode::MODE_0);
}

static void connect_client_eet(gatherer& client, char const* host, uint16_t port, size_t chunk_size, uint8_t fps)
{
    std::vector<uint8_t> cfg;

    create_configuration_for_eet(cfg, fps);
    client.open(host, port, chunk_size, stream_mode::MODE_1);
    client.sendall((char*)cfg.data(), cfg.size());
}

enum PVCNT : uint8_t
{
    START  = 0x04,
    STOP   = 0x08,
    MODE_3 = 0x03,
};

void start_subsystem_pv(char const* host, uint16_t port)
{
    std::vector<uint8_t> cfg;
    create_configuration_for_pv_mode2(cfg, PVCNT::START | PVCNT::MODE_3, 1920, 1080, 30);

    client c;
    c.open(host, port);
    c.sendall((char*)cfg.data(), cfg.size());
    c.close();
}

void stop_subsystem_pv(char const* host, uint16_t port)
{
    std::vector<uint8_t> cfg;
    create_configuration_for_pv_mode2(cfg, PVCNT::STOP | PVCNT::MODE_3, 1920, 1080, 30);

    client c;
    c.open(host, port);
    c.sendall((char*)cfg.data(), cfg.size());
    c.close();
}

//------------------------------------------------------------------------------
// Receiver Wrappers
//------------------------------------------------------------------------------

std::shared_ptr<packet> rx::get_next_packet()
{
    return m_client.get_next_packet();
}

void rx::close()
{
    return m_client.close();
}

rx_rm_vlc::rx_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
    this->divisor = divisor;
    this->profile = profile;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}

void rx_rm_vlc::open()
{
    connect_client_rm_vlc(m_client, host.c_str(), port, chunk, mode, divisor, profile, level, bitrate, options);
}

rx_rm_depth_ahat::rx_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
    this->divisor = divisor;
    this->profile_z = profile_z;
    this->profile_ab = profile_ab;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}

void rx_rm_depth_ahat::open()
{
    connect_client_rm_depth_ahat(m_client, host.c_str(), port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options);
}

rx_rm_depth_longthrow::rx_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
    this->divisor = divisor;
    this->png_filter = png_filter;
}

void rx_rm_depth_longthrow::open()
{
    connect_client_rm_depth_longthrow(m_client, host.c_str(), port, chunk, mode, divisor, png_filter);
}

rx_rm_imu::rx_rm_imu(char const* host, uint16_t port, size_t chunk, uint8_t mode)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
}

void rx_rm_imu::open()
{
    connect_client_rm_imu(m_client, host.c_str(), port, chunk, mode);
}

rx_pv::rx_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->mode = mode;
    this->width = width;
    this->height = height;
    this->framerate = framerate;
    this->divisor = divisor;
    this->profile = profile;
    this->level = level;
    this->bitrate = bitrate;
    this->options = options;
}
    
void rx_pv::open()
{
    connect_client_pv(m_client, host.c_str(), port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options);
}

rx_microphone::rx_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->profile = profile;
    this->level = level;
}

void rx_microphone::open()
{
    connect_client_microphone(m_client, host.c_str(), port, chunk, profile, level);
}

rx_si::rx_si(char const* host, uint16_t port, size_t chunk)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
}

void rx_si::open()
{
    connect_client_si(m_client, host.c_str(), port, chunk);
}

rx_eet::rx_eet(char const* host, uint16_t port, size_t chunk, uint8_t fps)
{
    this->host = host;
    this->port = port;
    this->chunk = chunk;
    this->fps = fps;
}

void rx_eet::open()
{
    connect_client_eet(m_client, host.c_str(), port, chunk, fps);
}

//------------------------------------------------------------------------------
// Frame
//------------------------------------------------------------------------------

frame::frame()
{
    av_frame = NULL;
}

frame::~frame()
{
    free_frame();
}

void frame::init_frame()
{
    av_frame = av_frame_alloc();
    if (av_frame == NULL) { throw std::runtime_error("hl2ss::frame::init_frame : av_frame_alloc failed"); }
}

void frame::free_frame()
{
    if (av_frame != NULL) { av_frame_free(&av_frame); }
}

//------------------------------------------------------------------------------
// Codecs
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

    AVCodec* decoder = avcodec_find_decoder(id);
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

    f->init_frame();

    status = avcodec_receive_frame(m_c, f->av_frame);
    if (status < 0) { throw std::runtime_error("hl2ss::codec::codec : avcodec_receive_frame failed"); }

    return f;
}

void codec::close()
{
    if (m_c != NULL) { avcodec_free_context(&m_c); }
    if (m_avpkt != NULL) { av_packet_free(&m_avpkt); }
}

static void trim_plane(uint8_t* dst, uint8_t const* src, uint16_t height, uint16_t width, uint16_t stride)
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

static void collect_I420(uint8_t* dst, int width, int height, uint8_t* data[8], int linesize[8])
{
    trim_plane(dst,                           data[0], height,     width,     linesize[0]);
    trim_plane(dst + (   height * width),     data[1], height / 2, width / 2, linesize[1]);
    trim_plane(dst + (5 *height * width) / 4, data[2], height / 2, width / 2, linesize[2]);
}

static void collect_NV12(uint8_t* dst, uint16_t width, uint16_t height, uint8_t const* src, uint16_t stride)
{
    trim_plane(dst,                    src,                     height, width,     stride);
    trim_plane(dst + (height * width), src + (height * stride), height, width / 2, stride / 2);
}

static uint16_t get_video_stride(uint16_t width)
{
    return width + ((64 - (width & 63)) & 63);
}

//------------------------------------------------------------------------------
// Decoded Receivers
//------------------------------------------------------------------------------

rx_decoded_rm_vlc::rx_decoded_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)
{
}

void rx_decoded_rm_vlc::open()
{
    if (profile != video_profile::VIDEO_RAW) { m_codec.open(get_video_codec_id(profile)); }
    rx_rm_vlc::open();
}

std::shared_ptr<packet> rx_decoded_rm_vlc::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_vlc::get_next_packet();
    if (profile == video_profile::VIDEO_RAW) { return p; }
    std::shared_ptr<frame> f = m_codec.decode(p->payload, p->sz_payload);
    p->free_payload();
    p->sz_payload = parameters_rm_vlc::PIXELS * sizeof(uint8_t);
    p->init_payload();
    memcpy(p->payload, f->av_frame->data[0], p->sz_payload);
    return p;
}

void rx_decoded_rm_vlc::close()
{
    rx_rm_vlc::close();
    if (profile != video_profile::VIDEO_RAW) {  m_codec.close(); }
}

rx_decoded_rm_depth_ahat::rx_decoded_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)
{
}

void rx_decoded_rm_depth_ahat::open()
{
    if (profile_ab != video_profile::VIDEO_RAW) { m_codec.open(get_video_codec_id(profile_ab)); }
    rx_rm_depth_ahat::open();
}

std::shared_ptr<packet> rx_decoded_rm_depth_ahat::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_depth_ahat::get_next_packet();
    if (profile_z == hl2ss::depth_profile::SAME)
    {
    if (profile_ab == hl2ss::video_profile::VIDEO_RAW) { return p; }
    std::shared_ptr<frame> f = m_codec.decode(p->payload, p->sz_payload);
    p->free_payload();
    p->sz_payload = parameters_rm_depth_ahat::PIXELS * 2 * sizeof(uint16_t);
    uint32_t sz = p->sz_payload / 2;
    p->init_payload();
    cv::Mat y = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_8UC1, f->av_frame->data[0]);
    cv::Mat depth = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, p->payload);
    y.convertTo(depth, CV_16UC1, 4);
    cv::Mat u = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[1]);
    cv::Mat v = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[2]);
    std::vector<cv::Mat> uv;
    uv.push_back(u);
    uv.push_back(v);
    cv::Mat ab_4q8 = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC2);
    cv::merge(uv, ab_4q8);
    cv::Mat ab_2q8 = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, ab_4q8.data);
    cv::Mat ab_2q  = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH / 2, CV_16UC1);
    ab_2q8.convertTo(ab_2q, CV_16UC1);
    cv::Mat ab_2 = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH / 2, CV_16UC1);
    cv::multiply(ab_2q, ab_2q, ab_2);
    cv::Mat ab = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, p->payload + sz);
    cv::resize(ab_2, ab, ab.size(), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
    throw std::runtime_error("hl2ss::rx_decoded_rm_depth_ahat::get_next_packet : ZDEPTH decompression not implemented");
    }

    return p;
}

void rx_decoded_rm_depth_ahat::close()
{
    rx_rm_depth_ahat::close();
    if (profile_ab != video_profile::VIDEO_RAW) { m_codec.close(); }
}

rx_decoded_rm_depth_longthrow::rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter) : rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)
{
}

void rx_decoded_rm_depth_longthrow::open()
{
    rx_rm_depth_longthrow::open();
}

std::shared_ptr<packet> rx_decoded_rm_depth_longthrow::get_next_packet()
{
    std::shared_ptr<packet> p = rx_rm_depth_longthrow::get_next_packet();
    cv::Mat image = cv::imdecode(cv::Mat(1, p->sz_payload, CV_8UC1, p->payload), cv::IMREAD_UNCHANGED);
    p->free_payload();
    p->sz_payload = parameters_rm_depth_longthrow::PIXELS * 2 * sizeof(uint16_t);
    p->init_payload();
    memcpy(p->payload, image.data, p->sz_payload);    
    return p;
}

void rx_decoded_rm_depth_longthrow::close()
{
    rx_rm_depth_longthrow::close();
}

int const rx_decoded_pv::cv_format[5][4] = 
{
    {3, CV_8UC3, cv::COLOR_YUV2BGR_I420,  cv::COLOR_YUV2BGR_NV12 },
    {3, CV_8UC3, cv::COLOR_YUV2RGB_I420,  cv::COLOR_YUV2RGB_NV12 },
    {4, CV_8UC4, cv::COLOR_YUV2BGRA_I420, cv::COLOR_YUV2BGRA_NV12},
    {4, CV_8UC4, cv::COLOR_YUV2RGBA_I420, cv::COLOR_YUV2RGBA_NV12},
    {1, CV_8UC1, cv::COLOR_YUV2GRAY_I420, cv::COLOR_YUV2GRAY_NV12},
};

rx_decoded_pv::rx_decoded_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format) : rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)
{
    this->decoded_format = decoded_format % (sizeof(cv_format) / (4 * sizeof(int)));
}

void rx_decoded_pv::open()
{
    if (profile != video_profile::VIDEO_RAW) { m_codec.open(get_video_codec_id(profile)); }
    rx_pv::open();
}

std::shared_ptr<packet> rx_decoded_pv::get_next_packet()
{
    std::shared_ptr<packet> p = rx_pv::get_next_packet();
    float k[4];
    memcpy(k, p->payload + p->sz_payload - sizeof(k), sizeof(k));
    cv::Mat input = cv::Mat((height * 3) / 2, width, CV_8UC1);
    cv::Mat output;
    int code;
    if (profile == video_profile::VIDEO_RAW)
    {
    collect_NV12(input.data, width, height, p->payload, get_video_stride(width));
    code = cv_format[decoded_format][3];
    }
    else
    {
    std::shared_ptr<frame> f = m_codec.decode(p->payload, p->sz_payload - sizeof(k));
    collect_I420(input.data, width, height, f->av_frame->data, f->av_frame->linesize);
    code = cv_format[decoded_format][2];
    }
    p->free_payload();
    p->sz_payload = (height * width * cv_format[decoded_format][0]) + sizeof(k);
    p->init_payload();
    output = cv::Mat(height, width, cv_format[decoded_format][1], p->payload);
    cv::cvtColor(input, output, code);
    memcpy(p->payload + p->sz_payload - sizeof(k), k, sizeof(k));
    
    return p;
}

void rx_decoded_pv::close()
{
    rx_pv::close();
    if (profile != video_profile::VIDEO_RAW) { m_codec.close(); }
}

rx_decoded_microphone::rx_decoded_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level) : rx_microphone(host, port, chunk, profile, level)
{
}

void rx_decoded_microphone::open()
{
    if (profile != audio_profile::AUDIO_RAW) { m_codec.open(get_audio_codec_id(profile)); }
    rx_microphone::open();
}

std::shared_ptr<packet> rx_decoded_microphone::get_next_packet()
{
    std::shared_ptr<packet> p = rx_microphone::get_next_packet();
    if (profile == audio_profile::AUDIO_RAW) { return p; }
    std::shared_ptr<frame> f = m_codec.decode(p->payload, p->sz_payload);
    int sz = f->av_frame->linesize[0] / 2;   
    p->free_payload();
    p->sz_payload = sz * 2;
    p->init_payload();        
    memcpy(p->payload,      f->av_frame->data[0], sz);
    memcpy(p->payload + sz, f->av_frame->data[1], sz);
    return p;
}

void rx_decoded_microphone::close()
{
    rx_microphone::close();
    if (profile != audio_profile::AUDIO_RAW) { m_codec.close(); }
}

//------------------------------------------------------------------------------
// Mode 2 Data Acquisition
//------------------------------------------------------------------------------
}



int main()
{
    std::vector<uint64_t> options;
    options.push_back(hl2ss::h26x_encoder_property::CODECAPI_AVEncMPVGOPSize);
    options.push_back(45);

    //hl2ss::rx_decoded_rm_vlc client("192.168.1.7", 3800, 4096, 1, 1, 3, 0xFF, 1*1024*1024, options);
    //hl2ss::rx_rm_depth_ahat client("192.168.1.7", 3804, 4096, 1, 1, 0, 3, 0xFF, 8*1024*1024, options);
    //hl2ss::rx_decoded_rm_depth_longthrow client("192.168.1.7", 3805, 4096, 1, 1, 5);
    //hl2ss::rx_rm_imu client("192.168.1.7", 3806, 4096, 1);
    //hl2ss::start_subsystem_pv("192.168.1.7", 3810);
    //hl2ss::rx_decoded_pv client("192.168.1.7", 3810, 4096, 1, 1920, 1080, 30, 1, 0, 0xFF, 5*1024*1024, options, 0);
    //hl2ss::rx_microphone client("192.168.1.7", 3811, 4096, 3, 0x29);
    //hl2ss::rx_si client("192.168.1.7", 3812, 4096);
    //hl2ss::rx_eet client("192.168.1.7", 3817, 4096, 30);
    //hl2ss::rx_decoded_microphone client("192.168.1.7", hl2ss::stream_port::MICROPHONE, 4096, hl2ss::audio_profile::AAC_24000, hl2ss::aac_level::L2);

    hl2ss::rx_decoded_rm_depth_ahat 
    client(
        "192.168.1.7",
        hl2ss::stream_port::RM_DEPTH_AHAT,
        4096,
        hl2ss::stream_mode::MODE_1,
        1, 
        hl2ss::depth_profile::SAME,
        hl2ss::video_profile::H265_MAIN,
        hl2ss::h26x_level::DEFAULT,
        8*1024*1024,
        options
    );

    try
    {
        client.open();

        for (;;)
        {
        std::shared_ptr<hl2ss::packet> data = client.get_next_packet();

        cv::Mat depth = cv::Mat(512, 512, CV_16UC1, data->payload);
        cv::Mat ab = cv::Mat(512, 512, CV_16UC1, data->payload + (512*512*2));

        cv::imshow("depth", depth * 16);
        cv::imshow("ab", ab);
        cv::waitKey(1);

        //hl2ss::frame* frame = decoder.decode(data->payload, data->sz_payload);

        //cv::Mat bgr = hl2ss::frame_to_opencv_mat(*(frame->f));
        //cv::Mat bgr = cv::Mat(480, 640, CV_8UC1, data->payload);
        //cv::Mat depth = cv::Mat(288, 320, CV_16UC1, data->payload);
        //cv::Mat ab = cv::Mat(288, 320, CV_16UC1, data->payload + hl2ss::parameters_rm_depth_longthrow::PIXELS * sizeof(uint16_t));

        //cv::Mat bgr = cv::Mat(client.height, client.width, CV_8UC3, data->payload);
        //cv::imshow("TEST BGR", bgr);
        //cv::imshow("Test DEPTH", depth * 8);
        //cv::imshow("Test AB", ab);
        //cv::waitKey(1);

        //frame->Release();
        //std::cout << "PAYLOAD SIZE: " << data->sz_payload << std::endl;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    client.close();
    //decoder.close();    
    
    return 0;
}
