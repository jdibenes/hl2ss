
#include <queue>
#include <mutex>
#include <memory>
#include <atomic>
#include <cpr/cpr.h>

namespace hl2ss
{
namespace dp
{
//------------------------------------------------------------------------------
// * Constants
//------------------------------------------------------------------------------

namespace stream_port
{
char const* const LIVE      = "live";
char const* const LIVE_HIGH = "live_high";
char const* const LIVE_MED  = "live_med";
char const* const LIVE_LOW  = "live_low";
};

namespace stream_kind
{
uint8_t const VIDEO = 1;
uint8_t const AUDIO = 2;
}

namespace chunk_size
{
uint64_t const MRC = 4096;
}

//------------------------------------------------------------------------------
// * Client
//------------------------------------------------------------------------------

struct mrc_configuration
{
    bool holo;
    bool pv;
    bool mic;
    bool loopback;
    bool RenderFromCamera;
    bool vstab;
    int vstabbuffer;
};

struct chunk_descriptor
{
    uint64_t size;
    std::shared_ptr<uint8_t[]> data;
};

class client
{
private:
    std::vector<cpr::AsyncResponse> m_response;
    std::atomic<bool> m_run;
    std::mutex m_mutex;
    std::queue<chunk_descriptor> m_buffer;
    uint64_t m_read;

    bool on_write(std::string_view const& data, intptr_t userdata);
    bool on_progress(cpr::cpr_off_t downloadTotal, cpr::cpr_off_t downloadNow, cpr::cpr_off_t uploadTotal, cpr::cpr_off_t uploadNow, intptr_t userdata);

    static char const* bool_to_str(bool v);

public:
    void open(char const* host, char const* port, char const* user, char const* password, mrc_configuration const& configuration);
    uint64_t recv(void* buffer, uint64_t count);
    void download(void* buffer, uint64_t total, uint64_t chunk);
    void close();
};

//------------------------------------------------------------------------------
// * Packet Gatherer
//------------------------------------------------------------------------------

template<typename T>
struct mp4_box
{
    uint32_t size;
    uint32_t type;
    T data;
};

typedef mp4_box<std::shared_ptr<uint8_t[]>> box_hold;
typedef mp4_box<uint8_t*>                   box_view;

struct stream_header
{
    uint32_t id;
    uint32_t offset;
    uint32_t count;
};

struct sample_descriptor
{
    uint32_t span;
    uint32_t size;
    uint32_t flag;
};

struct stream_descriptor
{
    stream_header header;
    sample_descriptor samples[1];
};

class gatherer
{
private:
    std::vector<std::shared_ptr<uint8_t[]>> m_streams;
    std::queue<std::shared_ptr<packet>> m_packets;

    client m_client;

    uint64_t m_chunk;
    uint32_t m_state;
    
    uint32_t m_video_id;
    uint64_t m_video_ct;
    uint32_t m_video_tb;
    uint64_t m_video_et;

    std::unique_ptr<uint8_t[]> m_video_init_data;
    uint32_t                   m_video_init_size;

    uint32_t m_audio_id;
    uint64_t m_audio_ct;
    uint32_t m_audio_tb;
    uint64_t m_audio_et;

    box_hold get_next_box();

    static uint64_t compute_timestamp(uint64_t ct, uint64_t et, uint32_t tb);
    static void avcc_to_annex_b(uint8_t* sample, uint32_t size);
    static void raw_aac_to_adts(uint8_t* sample, uint32_t size);
    static std::vector<box_view> flatten_box(box_view const& top);

public:
    void open(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration);
    std::shared_ptr<packet> get_next_packet();
    void close();
};

//------------------------------------------------------------------------------
// * Mode 0 Data Acquisition
//------------------------------------------------------------------------------

class rx_mrc
{
private:
    gatherer m_client;

public:
    std::string host;
    std::string port;
    std::string user;
    std::string password;
    uint64_t chunk;
    mrc_configuration configuration;

    rx_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration);
    virtual ~rx_mrc();

    virtual void open();
    virtual std::shared_ptr<packet> get_next_packet();
    virtual void close();
};

//------------------------------------------------------------------------------
// * Decoder
//------------------------------------------------------------------------------

class decoder_mrc
{
private:
    codec m_video_codec;
    codec m_audio_codec;

    std::unique_ptr<uint8_t[]> decode_video(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size, uint16_t& width, uint16_t& height);
    std::unique_ptr<uint8_t[]> decode_audio(uint8_t* data, uint32_t size, uint32_t& decoded_size);

public:
    void open();
    std::unique_ptr<uint8_t[]> decode(uint8_t* data, uint32_t size, uint8_t decoded_format, uint32_t& decoded_size, uint16_t& width, uint16_t& height);
    void close();
};

//------------------------------------------------------------------------------
// * Mode 0 Data Acquisition (Decoded)
//------------------------------------------------------------------------------

class rx_decoded_mrc : public rx_mrc
{
protected:
    decoder_mrc m_decoder;

public:
    uint8_t decoded_format;
    uint16_t width;
    uint16_t height;

    rx_decoded_mrc(char const* host, char const* port, char const* user, char const* password, uint64_t chunk, mrc_configuration const& configuration, uint8_t decoded_format);

    void open() override;
    std::shared_ptr<packet> get_next_packet() override;
    void close() override;
};

//------------------------------------------------------------------------------
// * Unpacking
//------------------------------------------------------------------------------

struct mrc_video_metadata
{
    uint8_t header;
    uint8_t _reserved[3];
    uint16_t width;
    uint16_t height;
};

struct mrc_audio_metadata
{
    uint8_t header;
    uint8_t _reserved[7];
};

struct map_mrc_video
{
    mrc_video_metadata* header;
    uint8_t* data;
};

struct map_mrc_audio
{
    mrc_audio_metadata* header;
    float* data;
};

map_mrc_video unpack_mrc_video(uint8_t* payload);
map_mrc_audio unpack_mrc_audio(uint8_t* payload);

constexpr uint8_t mrc_get_kind(uint8_t header)
{
    return header & 3;
}

constexpr bool mrc_is_key_frame(uint8_t header)
{
    return (header & 4) != 0;
}
}
}
