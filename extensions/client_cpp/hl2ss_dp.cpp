
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <chrono>
#include <thread>
#include <opencv2/imgproc.hpp>
#include "hl2ss.h"
#include "hl2ss_dp.h"

namespace hl2ss
{
namespace dp
{
//------------------------------------------------------------------------------
// * Client
//------------------------------------------------------------------------------

char const* client::bool_to_str(bool v)
{
    return v ? "true" : "false";
}

bool client::on_write(std::string_view const& data, intptr_t userdata)
{
    uint64_t size = data.end() - data.begin();

    if (size > 0)
    {
    chunk_descriptor cd{size, std::make_unique<uint8_t[]>(size)};
    memcpy(cd.data.get(), data.data(), size);
    {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_buffer.push(cd);
    }
    }
    
    return m_run;
}

bool client::on_progress(cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata)
{
    return m_run;
}

void client::open(char const* host, char const* port, char const* user, char const* password, mrc_configuration const& configuration)
{
    cpr::Url url = cpr::Url{"https://" + std::string(host) + "/api/holographic/stream/" + std::string(port) + ".mp4"};
    cpr::WriteCallback wb = cpr::WriteCallback([=](std::string_view const& data, intptr_t userdata){ return this->on_write(data, userdata); });
    cpr::Authentication auth = cpr::Authentication{user, password, cpr::AuthMode::BASIC};
    cpr::ProgressCallback pb = cpr::ProgressCallback([=](cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata){ return this->on_progress(downloadTotal, downloadNow, uploadTotal, uploadNow, userdata); });
    cpr::SslOptions verify = cpr::Ssl(cpr::ssl::VerifyHost{false}, cpr::ssl::VerifyPeer{false}, cpr::ssl::VerifyStatus{false});
    cpr::Parameters params = cpr::Parameters{
        {"holo",             bool_to_str(configuration.holo)},
        {"pv",               bool_to_str(configuration.pv)},
        {"mic",              bool_to_str(configuration.mic)},
        {"loopback",         bool_to_str(configuration.loopback)},
        {"RenderFromCamera", bool_to_str(configuration.RenderFromCamera)},
        {"vstab",            bool_to_str(configuration.vstab)},
        {"vstabbuffer",      std::to_string(configuration.vstabbuffer)}
    };
    
    m_read = 0;
    m_run = true;
    m_response.emplace_back(cpr::GetAsync(url, params, auth, verify, pb, wb));
}

uint64_t client::recv(void* buffer, uint64_t count)
{
    if (count <= 0) { return 0; }

    std::chrono::steady_clock::time_point wd_stp = std::chrono::steady_clock::now();

    uint8_t* base      = (uint8_t*)buffer;
    uint64_t remaining = count;

    do
    {
    if (m_response[0].wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
    m_run = false;
    throw std::runtime_error("hl2ss::dp::client : request terminated with status " + std::to_string(m_response[0].get().status_code));
    }

    while (remaining > 0)
    {
    chunk_descriptor cd;

    {
    std::lock_guard<std::mutex> guard(m_mutex);
    if (m_buffer.size() <= 0) { break; }
    cd = m_buffer.front();
    }

    uint64_t available = cd.size - m_read;
    uint64_t read = available > remaining ? remaining : available;

    memcpy(base, cd.data.get() + m_read, read);

    base      += read;
    m_read    += read;
    remaining -= read;

    if ((available - read) > 0) { continue; }

    m_read = 0;

    {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_buffer.pop();
    }
    }

    uint64_t received = count - remaining;
    if (received > 0) { return received; }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while ((std::chrono::steady_clock::now() - wd_stp) < std::chrono::seconds{8});

    m_run = false;
    throw std::runtime_error("hl2ss::dp::client::recv : timeout");
}

void client::download(void* buffer, uint64_t total, uint64_t chunk)
{
    uint64_t received = 0;
    while (total > 0)
    {
    if (chunk > total) { chunk = total; }
    uint64_t bytes = recv((uint8_t*)buffer + received, chunk);
    total    -= bytes;
    received += bytes;
    }
}

void client::close()
{
    m_run = false;
    (void)m_response[0].get();
    m_response.clear();
    {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_buffer = {};
    }
}

//------------------------------------------------------------------------------
// * Packet Gatherer
//------------------------------------------------------------------------------

uint64_t gatherer::compute_timestamp(uint64_t ct, uint64_t et, uint32_t tb)
{
    return (uint64_t)(((ct + et) / (double)tb) * hl2ss::time_base::HUNDREDS_OF_NANOSECONDS);
}

void gatherer::avcc_to_annex_b(uint8_t* sample, uint32_t size)
{
    uint32_t offset = 0;
    while (offset < size)
    {
    uint8_t* base    = sample + offset;
    uint32_t branch  = offset + 4 + ntohl(*(uint32_t*)(base));
    *(uint32_t*)base = *(uint32_t*)"\x00\x00\x00\x01";
    offset           = branch;
    }
}

void gatherer::raw_aac_to_adts(uint8_t* sample, uint32_t size)
{
    sample[0] = 0xFF;
    sample[1] = 0xF1;
    sample[2] = 0x4C;
    *(uint32_t*)(sample + 3) = htonl(0x800001EC | ((size + 7) << 13));
}

box_hold gatherer::get_next_box()
{
    box_hold b;
    m_client.download(&b, 8, m_chunk);
    b.size = ntohl(b.size);
    b.data = nullptr;
    int32_t d_size = b.size - 8;
    if (d_size <= 0) { return b; }
    b.data = std::make_unique<uint8_t[]>(d_size);
    m_client.download(b.data.get(), d_size, m_chunk);
    return b;
}

std::vector<box_view> gatherer::flatten_box(box_view const& top)
{
    uint64_t pos = 0;
    uint64_t end = top.size - 8;
    std::vector<box_view> subs;

    while (pos < end)
    {
    box_view b;
    uint8_t* base = top.data + pos;
    memcpy(&b, base, 8);
    b.size = ntohl(b.size);
    b.data = base + 8;
    subs.push_back(b);
    pos += b.size;
    }

    return subs;
}

void gatherer::open(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration)
{
    m_chunk = chunk;
    m_state = 0;

    m_video_id = 0;
    m_video_ct = 0;
    m_video_tb = 30000;
    m_video_et = 0;

    m_audio_id = 0;
    m_audio_ct = 0;
    m_audio_tb = 48000;
    m_audio_et = 0;

    m_client.open(host, port, user, password, configuration);
}

std::shared_ptr<packet> gatherer::get_next_packet()
{
    while (m_packets.empty())
    {
        box_hold own = get_next_box();
        box_view top{own.size, own.type, own.data.get()};

        switch (m_state)
        {
        case 0:
            if (top.type == *(uint32_t*)"moov")
            {
                for (auto& moov_box : flatten_box(top))
                {
                    if (moov_box.type == *(uint32_t*)"trak")
                    {
                        uint32_t id;
                        for (auto& trak_box : flatten_box(moov_box))
                        {
                            if (trak_box.type == *(uint32_t*)"tkhd")
                            {
                                id = ntohl(*(uint32_t*)(trak_box.data + 12));
                            }
                            else if (trak_box.type == *(uint32_t*)"mdia")
                            {
                                uint64_t ct;
                                uint32_t tb;
                                for (auto& mdia_box : flatten_box(trak_box))
                                {
                                    if (mdia_box.type == *(uint32_t*)"mdhd")
                                    {
                                        ct = ntohl(*(uint32_t*)(mdia_box.data +  4));
                                        tb = ntohl(*(uint32_t*)(mdia_box.data + 12));
                                    }
                                    else if (mdia_box.type == *(uint32_t*)"minf")
                                    {
                                        for (auto& minf_box : flatten_box(mdia_box))
                                        {
                                            if (minf_box.type == *(uint32_t*)"stbl")
                                            {
                                                for (auto& stbl_box : flatten_box(minf_box))
                                                {
                                                    if (stbl_box.type == *(uint32_t*)"stsd")
                                                    {
                                                        uint8_t* stbl_data = stbl_box.data;
                                                        uint32_t stbl_type = *(uint32_t*)(stbl_data + 12);
                                                        if (stbl_type == *(uint32_t*)"avc1")
                                                        {
                                                            m_video_id = id;
                                                            m_video_ct = (ct + 1) * tb;
                                                            m_video_tb = tb;

                                                            uint32_t sps_size = 134 - 106;
                                                            uint32_t pps_size = 141 - 133;
                                                            uint32_t nal_size = sps_size + pps_size;
                                                            uint8_t* sps_base = stbl_data + 106;
                                                            uint8_t* pps_base = stbl_data + 133;

                                                            m_video_init_data = std::make_unique<uint8_t[]>(nal_size);
                                                            m_video_init_size = nal_size;

                                                            uint8_t* content = m_video_init_data.get();

                                                            uint8_t* content_sps = content;
                                                            uint8_t* content_pps = content_sps + sps_size;

                                                            memcpy(content_sps, sps_base, sps_size);
                                                            memcpy(content_pps, pps_base, pps_size);

                                                            *(uint16_t*)(content_sps) = 0;
                                                            *(uint16_t*)(content_pps) = 0;
                                                        }
                                                        else if (stbl_type == *(uint32_t*)"mp4a")
                                                        {
                                                            m_audio_id = id;
                                                            m_audio_ct = ct * tb;
                                                            m_audio_tb = tb;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                m_state = 1;
            }
            break;
        case 1:
            if (top.type == *(uint32_t*)"moof")
            {
                m_streams.clear();
                for (auto& moof_box : flatten_box(top))
                {
                    if (moof_box.type == *(uint32_t*)"traf")
                    {
                        uint32_t id;
                        for (auto& traf_box : flatten_box(moof_box))
                        {
                            if (traf_box.type == *(uint32_t*)"tfhd")
                            {
                                id = ntohl(*(uint32_t*)(traf_box.data + 4));
                            }
                            else if (traf_box.type == *(uint32_t*)"trun")
                            {
                                uint32_t sample_count = ntohl(*(uint32_t*)(traf_box.data + 4));
                                uint32_t offset       = ntohl(*(uint32_t*)(traf_box.data + 8));

                                std::shared_ptr<uint8_t[]> stream = std::make_unique<uint8_t[]>(sizeof(stream_header) + (sample_count * sizeof(sample_descriptor)));
                                stream_descriptor* sd = (stream_descriptor*)stream.get();
                                
                                sd->header = {id, offset, sample_count};
                                for (uint32_t i = 0; i < sample_count; ++i)
                                { 
                                    uint32_t* base = (uint32_t*)(traf_box.data + 12 + (16 * i));
                                    sd->samples[i] = {ntohl(base[0]), ntohl(base[1]), ntohl(base[2])};
                                }

                                m_streams.push_back(stream);
                            }
                        }
                    }
                }
                m_state = 2;
            }
            break;
        case 2:
            if (top.type == *(uint32_t*)"mdat")
            {
                for (auto const& stream : m_streams)
                {
                    stream_descriptor const* sd = (stream_descriptor*)stream.get();

                    uint32_t id    = sd->header.id;
                    uint8_t* base  = top.data + sd->header.offset;
                    uint32_t count = sd->header.count;
                    uint32_t offset = 0;

                    for (uint32_t j = 0; j < count; ++j)
                    {
                        sample_descriptor const& ss = sd->samples[j];

                        uint32_t span = ss.span;
                        uint32_t size = ss.size; 
                        uint8_t  keyf = (~ss.flag >> 14) & 0x04;
                        uint8_t* sample = base + offset;

                        offset += size;

                        std::shared_ptr<packet> data = std::make_shared<packet>();

                        if (id == m_video_id)
                        {
                            data->timestamp = compute_timestamp(m_video_ct, m_video_et, m_video_tb);
                            if (!m_video_init_data)
                            {
                                data->init_payload(1 + size);
                                uint8_t* payload = data->payload.get();
                                uint8_t* content = payload + 1;
                                payload[0] = stream_kind::VIDEO | keyf;
                                memcpy(content, sample, size);
                            }
                            else
                            {
                                data->init_payload(1 + m_video_init_size + size);
                                uint8_t* payload = data->payload.get();
                                uint8_t* aud     = payload + 1;
                                uint8_t* sps_pps = aud + 6;
                                uint8_t* content = sps_pps + m_video_init_size;
                                payload[0] = stream_kind::VIDEO | keyf;
                                memcpy(aud,     sample,                  6);
                                memcpy(sps_pps, m_video_init_data.get(), m_video_init_size);
                                memcpy(content, sample + 6,              size - 6);
                                m_video_init_data = nullptr;
                            }
                            avcc_to_annex_b(data->payload.get() + 1, data->sz_payload - 1);
                            m_video_et += span;
                        }
                        else if (id == m_audio_id)
                        {
                            data->timestamp = compute_timestamp(m_audio_ct, m_audio_et, m_audio_tb);
                            data->init_payload(1 + 7 + size);
                            uint8_t* payload = data->payload.get();
                            uint8_t* header  = payload + 1;
                            uint8_t* content = header + 7;
                            payload[0] = stream_kind::AUDIO | keyf;
                            raw_aac_to_adts(header, size);
                            memcpy(content, sample, size);
                            m_audio_et += span;
                        }

                        if (data->sz_payload > 0) { m_packets.push(data); }
                    }
                }
                m_state = 1;
            }
            break;
        }
    }

    std::shared_ptr<packet> data = m_packets.front();
    m_packets.pop();

    return data; 
}

void gatherer::close()
{
    m_client.close();

    m_streams = {};
    m_packets = {};
}

//------------------------------------------------------------------------------
// * Mode 0 Data Acquisition
//------------------------------------------------------------------------------

rx_mrc::rx_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration)
{
    this->host = host;
    this->port = port;
    this->user = user;
    this->password = password;
    this->chunk = chunk;
    this->configuration = configuration;
}

rx_mrc::~rx_mrc()
{
}

void rx_mrc::open()
{
    m_client.open(host.c_str(), port.c_str(), user.c_str(), password.c_str(), chunk, configuration);
}

std::shared_ptr<packet> rx_mrc::get_next_packet()
{
    return m_client.get_next_packet();
}

void rx_mrc::close()
{
    m_client.close();
}

//------------------------------------------------------------------------------
// * Decoder
//------------------------------------------------------------------------------

std::unique_ptr<uint8_t[]> decoder_mrc::decode_video(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size, uint16_t& width, uint16_t& height)
{
    uint8_t* sample_base = data + 1;
    uint32_t sample_size = size - 1;
    std::shared_ptr<frame> f = m_video_codec.decode(sample_base, sample_size);
    width = f->av_frame->width;
    height = f->av_frame->height;
    cv::Mat input = cv::Mat((height * 3) / 2, width, CV_8UC1);
    collect_i420(input.data, width, height, f->av_frame->data, f->av_frame->linesize);
    decoded_size = width * height * decoder_pv::decoded_bpp(decoded_format);
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(8 + decoded_size);
    cv::Mat output = cv::Mat(height, width, decoder_pv::decoded_cv_type(decoded_format), out.get() + 8);
    memcpy(out.get(),      data,   1);
    memcpy(out.get() + 4, &width,  2);
    memcpy(out.get() + 6, &height, 2);
    cv::cvtColor(input, output, decoder_pv::decoded_cv_i420(decoded_format));
    return out;
}

std::unique_ptr<uint8_t[]> decoder_mrc::decode_audio(uint8_t* data, uint32_t size, uint32_t& decoded_size)
{
    uint8_t* sample_base = data + 1;
    uint32_t sample_size = size - 1;
    std::shared_ptr<frame> f = m_audio_codec.decode(sample_base, sample_size);
    decoded_size = f->av_frame->linesize[0];
    uint32_t offset = decoded_size / 2;
    std::unique_ptr<uint8_t[]> out = std::make_unique<uint8_t[]>(8 + decoded_size);
    memcpy(out.get(),              data,                      1);
    memcpy(out.get() + 8,          f->av_frame->data[0], offset);
    memcpy(out.get() + 8 + offset, f->av_frame->data[1], offset);
    return out;
}

void decoder_mrc::open()
{
    m_video_codec.open(get_video_codec_id(video_profile::H264_MAIN));
    m_audio_codec.open(get_audio_codec_id(audio_profile::AAC_24000));
}

std::unique_ptr<uint8_t[]> decoder_mrc::decode(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size, uint16_t& width, uint16_t& height)
{
    switch (data[0] & 3)
    {
    case stream_kind::VIDEO: return decode_video(data, size, decoded_format, decoded_size, width, height);
    case stream_kind::AUDIO: return decode_audio(data, size, decoded_size);
    default:                 return nullptr;
    }
}

void decoder_mrc::close()
{
    m_video_codec.close();
    m_audio_codec.close();
}

//------------------------------------------------------------------------------
// * Mode 0 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

rx_decoded_mrc::rx_decoded_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration, uint8_t decoded_format) : rx_mrc(host, port, user, password, chunk, configuration)
{
    this->decoded_format = decoded_format;
    this->width = 0;
    this->height = 0;
}

void rx_decoded_mrc::open()
{
    m_decoder.open();
    rx_mrc::open();
}

std::shared_ptr<packet> rx_decoded_mrc::get_next_packet()
{
    std::shared_ptr<packet> p = rx_mrc::get_next_packet();
    uint32_t size;
    std::unique_ptr<uint8_t[]> payload = m_decoder.decode(p->payload.get(), p->sz_payload, decoded_format, size, width, height);
    p->set_payload(size, std::move(payload));
    return p;
}

void rx_decoded_mrc::close()
{
    rx_mrc::close();
    m_decoder.close();
}

//------------------------------------------------------------------------------
// * Unpacking
//------------------------------------------------------------------------------

map_mrc_video unpack_mrc_video(uint8_t* payload)
{
    return { (mrc_video_metadata*)payload, payload + 8 };
}

map_mrc_audio unpack_mrc_audio(uint8_t* payload)
{
    return { (mrc_audio_metadata*)payload, (float*)(payload + 8) };
}
}
}
