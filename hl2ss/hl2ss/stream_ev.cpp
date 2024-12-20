
#include "extended_video.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_pv.h"

#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture::Frames;

class Channel_EV : public Channel
{
private:
    std::unique_ptr<Encoder_PV> m_pEncoder;
    bool m_enable_location;
    uint32_t m_counter;
    uint32_t m_divisor;
    uint16_t m_width;
    uint16_t m_height;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(MediaFrameReference const& frame);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);
    
    static void TranslateEncoderOptions(std::vector<uint64_t> const& options, uint32_t& stride_mask, MediaFrameReaderAcquisitionMode& acquisition_mode);

    static void Thunk_Sensor(MediaFrameReference const& frame, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_EV(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_EV> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_EV::TranslateEncoderOptions(std::vector<uint64_t> const& options, uint32_t& stride_mask, MediaFrameReaderAcquisitionMode& acquisition_mode)
{
    stride_mask      = 0x3F;
    acquisition_mode = MediaFrameReaderAcquisitionMode::Buffered;

    for (int i = 0; i < static_cast<int>(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case HL2SSAPI::HL2SSAPI_VideoStrideMask: stride_mask      = static_cast<uint32_t>(options[i + 1]);                                                                        break;
    case HL2SSAPI::HL2SSAPI_AcquisitionMode: acquisition_mode = (options[i + 1] & 1) ? MediaFrameReaderAcquisitionMode::Buffered : MediaFrameReaderAcquisitionMode::Realtime; break;
    }
    }
}

// OK
void Channel_EV::Thunk_Sensor(MediaFrameReference const& frame, void* self)
{
    static_cast<Channel_EV*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_EV::Thunk_Encoder(void* encoder, DWORD encoder_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_EV*>(self)->OnEncodingComplete(encoder, encoder_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_EV::OnFrameArrived(MediaFrameReference const& frame)
{
    PV_Metadata p;

    if (m_counter == 0)
    {
    memset(&p, 0, sizeof(p));

    p.timestamp  = frame.SystemRelativeTime().Value().count();
    p.resolution = (static_cast<uint32_t>(m_height) << 16) | static_cast<uint32_t>(m_width);

    m_pEncoder->WriteSample(frame, &p);
    }
    m_counter = (m_counter + 1) % m_divisor;
}

// OK
void Channel_EV::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    ULONG const embed_size = sizeof(PV_Metadata) - sizeof(PV_Metadata::timestamp) - sizeof(PV_Metadata::pose);

    PV_Metadata* p = static_cast<PV_Metadata*>(metadata);    
    ULONG full_size = encoded_size + embed_size;
    WSABUF wsaBuf[5];

    pack_buffer(wsaBuf, 0, &p->timestamp, sizeof(p->timestamp));
    pack_buffer(wsaBuf, 1, &full_size,    sizeof(full_size));
    pack_buffer(wsaBuf, 2, encoded,       encoded_size);
    pack_buffer(wsaBuf, 3, p,             embed_size);
    pack_buffer(wsaBuf, 4, &p->pose,      sizeof(p->pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EV::Execute_Mode0(bool enable_location)
{
    MediaFrameReaderAcquisitionMode acquisition_mode;
    H26xFormat format;
    std::vector<uint64_t> options;
    VideoSubtype subtype;
    uint32_t stride_mask;    
    bool ok;

    if (!ExtendedVideo_Status()) { return; }

    ok = ReceiveH26xFormat_Video(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveEncoderOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    ok = ExtendedVideo_SetFormat(format.width, format.height, format.framerate, subtype);
    if (!ok) { return; }

    TranslateEncoderOptions(options, stride_mask, acquisition_mode);

    uint32_t stride = (format.width + stride_mask) & ~stride_mask;

    m_pEncoder        = std::make_unique<Encoder_PV>(Thunk_Encoder, this, subtype, format, stride, options);    
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;
    m_width           = format.width;
    m_height          = format.height;

    ExtendedVideo_ExecuteSensorLoop(acquisition_mode, Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
void Channel_EV::Execute_Mode2()
{
    winrt::hstring query = ExtendedVideo_QueryDevices();
    WSABUF wsaBuf[2];    

    uint32_t bytes = query.size() * sizeof(wchar_t);

    pack_buffer(wsaBuf, 0, &bytes,        sizeof(bytes));
    pack_buffer(wsaBuf, 1, query.c_str(), bytes);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_EV::Channel_EV(char const* name, char const* port, uint32_t id) : 
Channel(name, port, id)
{
}

// OK
bool Channel_EV::Startup()
{
    SetNoDelay(true);
    return true;
}

// OK
void Channel_EV::Run()
{
    MRCVideoOptions options;
    uint8_t mode;    
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, m_event_client, mode);
    if (!ok) { return; }

    if (mode & 4)
    {
    ok = ReceiveMRCVideoOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    if (ExtendedVideo_Status()) { ExtendedVideo_Close(); }

    ExtendedVideo_Open(options);
    }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    case 2: Execute_Mode2();      break;
    }

    if (mode & 8)
    {
    if (ExtendedVideo_Status()) { ExtendedVideo_Close(); }
    }
}

// OK
void Channel_EV::Cleanup()
{
}

// OK
void EV_Startup()
{
    g_channel = std::make_unique<Channel_EV>("EV", PORT_NAME_EV, PORT_ID_EV);
}

// OK
void EV_Cleanup()
{
    g_channel.reset();
}
