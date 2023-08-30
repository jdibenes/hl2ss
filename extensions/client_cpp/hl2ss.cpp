
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "hl2ss.h"
#include "types.h"

#ifdef HL2SS_ENABLE_ZDEPTH
#include <zdepth.hpp>
#endif

namespace hl2ss
{
//------------------------------------------------------------------------------
// * Client
//------------------------------------------------------------------------------

client::client()
{
    m_socket = -1;
}

client::~client()
{
    if (m_socket >= 0) { close(); }
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

void client::sendall(void const* data, size_t count)
{
    ssize_t sent = 0;
    while (count > 0)
    {
    ssize_t bytes = send(m_socket, (char*)data + sent, count, 0);
    if (bytes <= 0) { throw std::runtime_error("hl2ss::client::sendall : send failed"); }
    count -= bytes;
    sent  += bytes;
    }
}

size_t client::recv(void* buffer, size_t count)
{
    size_t bytes = ::recv(m_socket, buffer, count, 0);
    if ((bytes <= 0) && (count > 0)) { throw std::runtime_error("hl2ss::client::recv : recv failed"); }
    return bytes;
}

void client::download(void* buffer, size_t total, size_t chunk)
{
    size_t received = 0;
    while (total > 0)
    {
    if (chunk > total) { chunk = total; }
    size_t bytes = recv((char*)buffer + received, chunk);
    total    -= bytes;
    received += bytes;
    }
}

void client::close()
{
    ::close(m_socket);
    m_socket = -1;
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

void packet::swap_payload(uint32_t size, std::unique_ptr<uint8_t[]> new_payload)
{
    payload.swap(new_payload);
    sz_payload = size;
}

void packet::init_pose()
{
    pose = std::make_unique<float[]>(SZ_POSE / sizeof(float));
}

//------------------------------------------------------------------------------
// * Packet Gatherer
//------------------------------------------------------------------------------

void gatherer::open(char const* host, uint16_t port, size_t chunk, uint8_t mode)
{
    m_client.open(host, port);
    m_chunk = chunk;
    m_mode = mode;
}

void gatherer::sendall(void const* data, size_t count)
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

void create_configuration_for_png_encoding(std::vector<uint8_t>& sc, uint8_t png_filter)
{
    push_u8(sc, png_filter);
}

void create_configuration_for_h26x_encoding(std::vector<uint8_t>& sc, std::vector<uint64_t> const& options)
{
    push_u8(sc, (uint8_t)(options.size() / 2));

    for (size_t i = 0; i < options.size(); i += 2)
    {
    push_u64(sc, options[i]);
    push_u64(sc, options[i+1]);
    }
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

//------------------------------------------------------------------------------
// * PV Control
//------------------------------------------------------------------------------

namespace pv_control
{
uint8_t const START  = 0x04;
uint8_t const STOP   = 0x08;
uint8_t const MODE_3 = 0x03;
};

void start_subsystem_pv(char const* host, uint16_t port)
{
    std::vector<uint8_t> cfg;
    client c;

    create_configuration_for_pv_mode_2(cfg, pv_control::START | pv_control::MODE_3, 1920, 1080, 30);

    c.open(host, port);
    c.sendall(cfg.data(), cfg.size());
    c.close();
}

void stop_subsystem_pv(char const* host, uint16_t port)
{
    std::vector<uint8_t> cfg;
    client c;

    create_configuration_for_pv_mode_2(cfg, pv_control::STOP | pv_control::MODE_3, 1920, 1080, 30);

    c.open(host, port);
    c.sendall(cfg.data(), cfg.size());
    c.close();
}

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition
//------------------------------------------------------------------------------

rx::rx(char const* host, uint16_t port, size_t chunk, uint8_t mode)
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

rx_rm_vlc::rx_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
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

rx_rm_depth_ahat::rx_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
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

rx_rm_depth_longthrow::rx_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter) : rx(host, port, chunk, mode)
{
    this->divisor = divisor;
    this->png_filter = png_filter;
}

void rx_rm_depth_longthrow::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_depth_longthrow(sc, mode, divisor, png_filter);
}

rx_rm_imu::rx_rm_imu(char const* host, uint16_t port, size_t chunk, uint8_t mode) : rx(host, port, chunk, mode)
{
}

void rx_rm_imu::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_rm_imu(sc, mode);
}

rx_pv::rx_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx(host, port, chunk, mode)
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

rx_microphone::rx_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level) : rx(host, port, chunk, stream_mode::MODE_0)
{
    this->profile = profile;
    this->level = level;
}

void rx_microphone::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_microphone(sc, profile, level);
}

rx_si::rx_si(char const* host, uint16_t port, size_t chunk) : rx(host, port, chunk, stream_mode::MODE_0)
{
}

void rx_si::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_si(sc);
}

rx_eet::rx_eet(char const* host, uint16_t port, size_t chunk, uint8_t framerate) : rx(host, port, chunk, stream_mode::MODE_1)
{
    this->framerate = framerate;
}

void rx_eet::create_configuration(std::vector<uint8_t>& sc)
{
    create_configuration_for_eet(sc, framerate);
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

void collect_I420(uint8_t* dst, int width, int height, uint8_t* data[8], int linesize[8])
{
    trim_plane(dst,                           data[0], height,     width,     linesize[0]);
    trim_plane(dst + (   height * width),     data[1], height / 2, width / 2, linesize[1]);
    trim_plane(dst + (5 *height * width) / 4, data[2], height / 2, width / 2, linesize[2]);
}

void collect_NV12(uint8_t* dst, uint16_t width, uint16_t height, uint8_t const* src, uint16_t stride)
{
    trim_plane(dst,                    src,                     height, width,     stride);
    trim_plane(dst + (height * width), src + (height * stride), height, width / 2, stride / 2);
}

uint16_t get_video_stride(uint16_t width)
{
    return width + ((64 - (width & 63)) & 63);
}

int const cv_format[5][4] = 
{
    {3, CV_8UC3, cv::COLOR_YUV2BGR_I420,  cv::COLOR_YUV2BGR_NV12 },
    {3, CV_8UC3, cv::COLOR_YUV2RGB_I420,  cv::COLOR_YUV2RGB_NV12 },
    {4, CV_8UC4, cv::COLOR_YUV2BGRA_I420, cv::COLOR_YUV2BGRA_NV12},
    {4, CV_8UC4, cv::COLOR_YUV2RGBA_I420, cv::COLOR_YUV2RGBA_NV12},
    {1, CV_8UC1, cv::COLOR_YUV2GRAY_I420, cv::COLOR_YUV2GRAY_NV12},
};

void decoder_rm_vlc::open(uint8_t profile)
{
    m_profile = profile;
    if (m_profile != video_profile::RAW) { m_codec.open(get_video_codec_id(m_profile)); }
}

std::unique_ptr<uint8_t[]> decoder_rm_vlc::decode(uint8_t* data, uint32_t size)
{
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(decoded_size);
    if (m_profile != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data, size);
    memcpy(out.get(), f->av_frame->data[0], parameters_rm_vlc::PIXELS);
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
    if (m_profile_ab != hl2ss::video_profile::RAW) { m_codec.open(get_video_codec_id(profile_ab)); }
}

std::unique_ptr<uint8_t[]> decoder_rm_depth_ahat::decode(uint8_t* data, uint32_t size)
{
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(decoded_size);
    if (m_profile_z == hl2ss::depth_profile::SAME)
    {
    if (m_profile_ab != hl2ss::video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data, size);

    uint32_t sz = decoded_size / 2;
    
    cv::Mat depth = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, out.get());
    cv::Mat ab    = cv::Mat(parameters_rm_depth_ahat::HEIGHT, parameters_rm_depth_ahat::WIDTH, CV_16UC1, out.get() + sz);

    cv::Mat y  = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH,     CV_8UC1, f->av_frame->data[0]);
    cv::Mat u  = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[1]);
    cv::Mat v  = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, f->av_frame->data[2]);
    cv::Mat uv = cv::Mat(parameters_rm_depth_ahat::HEIGHT / 2, parameters_rm_depth_ahat::WIDTH / 2, CV_8UC2);
    cv::Mat cc = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH / 2, CV_8UC1, uv.data);
    cv::Mat bb = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH / 2, CV_16UC1);
    cv::Mat ww = cv::Mat(parameters_rm_depth_ahat::HEIGHT,     parameters_rm_depth_ahat::WIDTH / 2, CV_16UC1);
    
    y.convertTo(depth, CV_16UC1, 4);
    cv::merge(std::vector<cv::Mat>{u, v}, uv);
    cc.convertTo(bb, CV_16UC1);
    cv::multiply(bb, bb, ww);
    cv::resize(ww, ab, ab.size(), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
    memcpy(out.get(), data, size);
    }
    }
    else
    {
    throw std::runtime_error("hl2ss::rx_decoded_rm_depth_ahat::get_next_packet : ZDEPTH decompression not implemented");
    }
    return out;
}

void decoder_rm_depth_ahat::close()
{
    if (m_profile_ab != hl2ss::video_profile::RAW) { m_codec.close(); }
}

void decoder_rm_depth_longthrow::open()
{
}

std::unique_ptr<uint8_t[]> decoder_rm_depth_longthrow::decode(uint8_t* data, uint32_t size)
{
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(decoded_size);
    cv::Mat output = cv::Mat(parameters_rm_depth_longthrow::HEIGHT, parameters_rm_depth_longthrow::WIDTH, CV_8UC4, out.get());
    cv::imdecode(cv::Mat(1, size, CV_8UC1, data), cv::IMREAD_UNCHANGED, &output);
    return out;
}

void decoder_rm_depth_longthrow::close()
{
}

void decoder_pv::open(uint16_t width, uint16_t height, uint8_t profile)
{
    m_width = width;
    m_height = height;
    m_profile = profile;

    if (m_profile != video_profile::RAW) { m_codec.open(get_video_codec_id(m_profile)); }
}

uint32_t decoder_pv::decoded_size(uint8_t decoded_format)
{
    return m_width * m_height * cv_format[decoded_format][0] + K_SIZE;
}

std::unique_ptr<uint8_t[]> decoder_pv::decode(uint8_t* data, uint32_t size, uint8_t decoded_format)
{
    uint32_t full_size = decoded_size(decoded_format);
    uint32_t h26x_size = size - K_SIZE;

    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(full_size);

    cv::Mat input = cv::Mat((m_height * 3) / 2, m_width, CV_8UC1);
    cv::Mat output;
    int code;   

    if (m_profile != video_profile::RAW)
    {
    std::shared_ptr<frame> f = m_codec.decode(data, h26x_size);
    collect_I420(input.data, m_width, m_height, f->av_frame->data, f->av_frame->linesize);
    code = cv_format[decoded_format][2];
    }
    else
    {
    collect_NV12(input.data, m_width, m_height, data, get_video_stride(m_width));
    code = cv_format[decoded_format][3];
    }
    
    output = cv::Mat(m_height, m_width, cv_format[decoded_format][1], out.get());
    cv::cvtColor(input, output, code);

    memcpy(out.get() + full_size - K_SIZE, data + h26x_size, K_SIZE);
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
    if (m_profile != audio_profile::RAW) { 
    out = std::make_unique<uint8_t[]>(decoded_size);
    std::shared_ptr<frame> f = m_codec.decode(data, size);
    uint32_t offset = f->av_frame->linesize[0] / 2;
    memcpy(out.get(),          f->av_frame->data[0], offset);
    memcpy(out.get() + offset, f->av_frame->data[1], offset);
    }
    else
    {
    out = std::make_unique<uint8_t[]>(raw_size);
    memcpy(out.get(), data, size);
    }
    return out;
}

void decoder_microphone::close()
{
    if (m_profile != audio_profile::RAW) { m_codec.close(); }
}

//------------------------------------------------------------------------------
// * Modes 0, 1 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

rx_decoded_rm_vlc::rx_decoded_rm_vlc(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_vlc(host, port, chunk, mode, divisor, profile, level, bitrate, options)
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
    if (profile != video_profile::RAW) { p->swap_payload(decoder_rm_vlc::decoded_size, m_decoder.decode(p->payload.get(), p->sz_payload)); }    
    return p;
}

void rx_decoded_rm_vlc::close()
{
    rx_rm_vlc::close();
    m_decoder.close();
}

rx_decoded_rm_depth_ahat::rx_decoded_rm_depth_ahat(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t profile_z, uint8_t profile_ab, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options) : rx_rm_depth_ahat(host, port, chunk, mode, divisor, profile_z, profile_ab, level, bitrate, options)
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
    if ((profile_z != depth_profile::SAME) || (profile_ab != video_profile::RAW)) { p->swap_payload(decoder_rm_depth_ahat::decoded_size, m_decoder.decode(p->payload.get(), p->sz_payload)); }
    return p;
}

void rx_decoded_rm_depth_ahat::close()
{
    rx_rm_depth_ahat::close();
    m_decoder.close();
}

rx_decoded_rm_depth_longthrow::rx_decoded_rm_depth_longthrow(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint8_t divisor, uint8_t png_filter) : rx_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter)
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
    p->swap_payload(decoder_rm_depth_longthrow::decoded_size, m_decoder.decode(p->payload.get(), p->sz_payload));
    return p;
}

void rx_decoded_rm_depth_longthrow::close()
{
    rx_rm_depth_longthrow::close();
    m_decoder.close();
}

rx_decoded_pv::rx_decoded_pv(char const* host, uint16_t port, size_t chunk, uint8_t mode, uint16_t width, uint16_t height, uint8_t framerate, uint8_t divisor, uint8_t profile, uint8_t level, uint32_t bitrate, std::vector<uint64_t> const& options, uint8_t decoded_format) : rx_pv(host, port, chunk, mode, width, height, framerate, divisor, profile, level, bitrate, options)
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
    p->swap_payload(m_decoder.decoded_size(decoded_format), m_decoder.decode(p->payload.get(), p->sz_payload, decoded_format));
    return p;
}

void rx_decoded_pv::close()
{
    rx_pv::close();
    m_decoder.close();
}

rx_decoded_microphone::rx_decoded_microphone(char const* host, uint16_t port, size_t chunk, uint8_t profile, uint8_t level) : rx_microphone(host, port, chunk, profile, level)
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
    if (profile != audio_profile::RAW) { p->swap_payload(decoder_microphone::decoded_size, m_decoder.decode(p->payload.get(), p->sz_payload)); }
    return p;
}

void rx_decoded_microphone::close()
{
    rx_microphone::close();
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

    c.download(data->uv2xy,         sizeof(data->uv2xy),         chunk_size::SINGLE_TRANSFER);
    c.download(data->extrinsics,    sizeof(data->extrinsics),    chunk_size::SINGLE_TRANSFER);
    c.download(data->undistort_map, sizeof(data->undistort_map), chunk_size::SINGLE_TRANSFER);
    c.download(data->intrinsics,    sizeof(data->intrinsics),    chunk_size::SINGLE_TRANSFER);
    
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

    c.download(data->uv2xy,         sizeof(data->uv2xy),         chunk_size::SINGLE_TRANSFER);
    c.download(data->extrinsics,    sizeof(data->extrinsics),    chunk_size::SINGLE_TRANSFER);
    c.download(&data->scale,        sizeof(data->scale),         chunk_size::SINGLE_TRANSFER);
    c.download(&data->alias,        sizeof(data->alias),         chunk_size::SINGLE_TRANSFER);
    c.download(data->undistort_map, sizeof(data->undistort_map), chunk_size::SINGLE_TRANSFER);
    c.download(data->intrinsics,    sizeof(data->intrinsics),    chunk_size::SINGLE_TRANSFER);
    
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

    c.download(data->uv2xy,         sizeof(data->uv2xy),         chunk_size::SINGLE_TRANSFER);
    c.download(data->extrinsics,    sizeof(data->extrinsics),    chunk_size::SINGLE_TRANSFER);
    c.download(&data->scale,        sizeof(data->scale),         chunk_size::SINGLE_TRANSFER);
    c.download(data->undistort_map, sizeof(data->undistort_map), chunk_size::SINGLE_TRANSFER);
    c.download(data->intrinsics,    sizeof(data->intrinsics),    chunk_size::SINGLE_TRANSFER);
    
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

    c.download(data->extrinsics, sizeof(data->extrinsics), chunk_size::SINGLE_TRANSFER);

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

    c.download(data->focal_length,          sizeof(data->focal_length),          chunk_size::SINGLE_TRANSFER);
    c.download(data->principal_point,       sizeof(data->principal_point),       chunk_size::SINGLE_TRANSFER);
    c.download(data->radial_distortion,     sizeof(data->radial_distortion),     chunk_size::SINGLE_TRANSFER);
    c.download(data->tangential_distortion, sizeof(data->tangential_distortion), chunk_size::SINGLE_TRANSFER);
    c.download(data->projection,            sizeof(data->projection),            chunk_size::SINGLE_TRANSFER);
    
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
};

uint16_t get_port_index(uint16_t port)
{
    return port - stream_port::RM_VLC_LEFTFRONT;
}

char const* get_port_name(uint16_t port)
{
    uint16_t index = get_port_index(port);
    return (index < (sizeof(port_name) / sizeof(char*))) ? port_name[index] : NULL;
}

//------------------------------------------------------------------------------
// IPC
//------------------------------------------------------------------------------
























ipc::ipc(char const* host, uint16_t port)
{
    this->host = host;
    this->port = port;
}

void ipc::open()
{
    m_client.open(host.c_str(), port);
}

void ipc::send(uint8_t command, std::initializer_list<uint32_t> list)
{
    m_sc.clear();
    push_u8(m_sc, command);
    for (uint32_t argument : list) { push_u32(m_sc, argument); }
    m_client.sendall(m_sc.data(), m_sc.size());
}

void ipc::recv(void* buffer, size_t size)
{
    m_client.download(buffer, size, chunk_size::SINGLE_TRANSFER);
}

void ipc::close()
{
    m_client.close();
}

//------------------------------------------------------------------------------
// Remote Configuration
//------------------------------------------------------------------------------

struct cmd_ipc_rc
{
    static uint8_t const GET_APPLICATION_VERSION = 0x00;
    static uint8_t const GET_UTC_OFFSET = 0x01;
    static uint8_t const SET_HS_MARKER_STATE = 0x02;
    static uint8_t const GET_PV_SUBSYSTEM_STATUS = 0x03;
    static uint8_t const SET_PV_FOCUS = 0x04;
    static uint8_t const SET_PV_VIDEO_TEMPORAL_DENOISING = 0x05;
    static uint8_t const SET_PV_WHITE_BALANCE_PRESET = 0x06;
    static uint8_t const SET_PV_WHITE_BALANCE_VALUE = 0x07;
    static uint8_t const SET_PV_EXPOSURE = 0x08;
    static uint8_t const SET_PV_EXPOSURE_PRIORITY_VIDEO = 0x09;
    static uint8_t const SET_PV_ISO_SPEED = 0x0A;
    static uint8_t const SET_PV_BACKLIGHT_COMPENSATION = 0x0B;
    static uint8_t const SET_PV_SCENE_MODE = 0x0C;
};

ipc_rc::ipc_rc(char const* host, uint16_t port) : ipc(host, port)
{
}

version ipc_rc::get_application_version()
{
    send(cmd_ipc_rc::GET_APPLICATION_VERSION, {});
    version data;
    recv(data.field, sizeof(data.field));
    return data;
}

uint64_t ipc_rc::get_utc_offset(uint32_t samples)
{
    send(cmd_ipc_rc::GET_UTC_OFFSET, {samples});
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

//------------------------------------------------------------------------------
// Spatial Mapping
//------------------------------------------------------------------------------

struct guid
{
    uint64_t l;
    uint64_t h;
};

class sm_bounding_volume
{
    friend class ipc_sm;

private:
    std::vector<uint8_t> m_data;
    uint32_t m_count;

public:
    sm_bounding_volume();

    void add_box(float center[3], float extents[3]);
    void add_frustum(float near[4], float far[4], float right[4], float left[4], float top[4], float bottom[4]);
    void add_oriented_box(float center[3], float extents[3], float orientation[4]);
    void add_sphere(float center[3], float radius);
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




sm_bounding_volume::sm_bounding_volume()
{
    m_count = 0;
}

void sm_bounding_volume::add_box(float center[3], float extents[3])
{
    m_count++;

    for (int i = 0; i < 3; ++i) { push_float(m_data, center[i]); }
    for (int i = 0; i < 3; ++i) { push_float(m_data, extents[i]); }
}

void sm_bounding_volume::add_frustum(float near[4], float far[4], float right[4], float left[4], float top[4], float bottom[4])
{
    m_count++;

    for (int i = 0; i < 4; ++i) { push_float(m_data, near[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, far[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, right[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, left[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, top[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, bottom[i]); }
}

void sm_bounding_volume::add_oriented_box(float center[3], float extents[3], float orientation[4])
{
    m_count++;

    for (int i = 0; i < 3; ++i) { push_float(m_data, center[i]); }
    for (int i = 0; i < 3; ++i) { push_float(m_data, extents[i]); }
    for (int i = 0; i < 4; ++i) { push_float(m_data, orientation[i]); }
}

void sm_bounding_volume::add_sphere(float center[3], float radius)
{
    m_count++;

    for (int i = 0; i < 3; ++i) { push_float(m_data, center[i]); }
    push_float(m_data, radius);
}

sm_mesh_task::sm_mesh_task()
{
    m_count = 0;
}

void sm_mesh_task::add_task(guid id, double max_triangles_per_cubic_meter, uint32_t vertex_position_format, uint32_t triangle_index_format, uint32_t vertex_normal_format, bool include_vertex_normals, bool include_bounds)
{
    push_u64(m_data, id.l);
    push_u64(m_data, id.h);
    push_double(m_data, max_triangles_per_cubic_meter);
    push_u32(m_data, vertex_position_format);
    push_u32(m_data, triangle_index_format);
    push_u32(m_data, vertex_normal_format);
    push_u32(m_data, (1*include_vertex_normals) | (2*include_bounds));
}










struct commmand_ipc_sm
{
    static uint8_t const CREATE_OBSERVER = 0x00;
    static uint8_t const SET_VOLUMES = 0x01;
    static uint8_t const GET_OBSERVED_SURFACES = 0x02;
    static uint8_t const GET_MESHES = 0x03;
};

/*
class ipc_sm
{
public:
    ipc_sm(char const* host, uint16_t port);

    void create_observer();
    void set_volumes(sm_bounding_volume volumes);
    std::shared_ptr<sm_surface_info[]> get_observed_surfaces();
    void get_meshes(sm_mesh_task tasks, uint32_t threads);
};

ipc_sm::ipc_sm(char const* host, uint16_t port)// : ipc(host, port)
{
}

void ipc_sm::create_observer()
{
    send(commmand_ipc_sm::CREATE_OBSERVER, {});
}

void ipc_sm::set_volumes(sm_bounding_volume volumes)
{
    send(commmand_ipc_sm::SET_VOLUMES, {});
}

std::shared_ptr<sm_surface_info[]> ipc_sm::get_observed_surfaces()
{
    send(commmand_ipc_sm::GET_OBSERVED_SURFACES, {});
    uint64_t count;
    recv(&count, sizeof(count));
    //std::shared_ptr<sm_surface_info[]> data = std::make_shared<sm_surface_info[]>(count);
    //recv(data.get(), count * sizeof(sm_surface_info));
    //return data;
}

void ipc_sm::get_meshes(sm_mesh_task tasks, uint32_t threads)
{
    send(commmand_ipc_sm::GET_MESHES, {tasks.m_count, threads});




}
*/
}
