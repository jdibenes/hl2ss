
#include "extended_depth.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_ez.h"
#include "log.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Media::Capture::Frames;

class Channel_EZ : Channel
{
private:
    std::unique_ptr<Encoder_EZ> m_pEncoder;
    bool m_enable_location;
    uint32_t m_counter;
    uint32_t m_divisor;
    uint16_t m_width;
    uint16_t m_height;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);

    void OnFrameArrived(MediaFrameReference const& frame);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void TranslateEncoderOptions(std::vector<uint64_t> const& options, uint32_t& media_index, uint32_t& stride_mask);

    static void Thunk_Sensor(MediaFrameReference const& frame, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_EZ(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_EZ> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_EZ::TranslateEncoderOptions(std::vector<uint64_t> const& options, uint32_t& media_index, uint32_t& stride_mask)
{
    media_index = ~0UL;
    stride_mask = 0x3F;

    for (int i = 0; i < static_cast<int>(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case HL2SSAPI::HL2SSAPI_VideoMediaIndex: media_index = static_cast<uint32_t>(options[i + 1]); break;
    case HL2SSAPI::HL2SSAPI_VideoStrideMask: stride_mask = static_cast<uint32_t>(options[i + 1]); break;
    }
    }
}

// OK
void Channel_EZ::Thunk_Sensor(MediaFrameReference const& frame, void* self)
{
    static_cast<Channel_EZ*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_EZ::Thunk_Encoder(void* encoder, DWORD encoder_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_EZ*>(self)->OnEncodingComplete(encoder, encoder_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_EZ::OnFrameArrived(MediaFrameReference const& frame)
{
    EZ_Metadata p;

    if (m_counter == 0)
    {
    p.resolution = (static_cast<uint32_t>(m_height) << 16) | static_cast<uint32_t>(m_width);
    if (!m_pEncoder->WriteSample(frame, &p)) { SetEvent(m_event_client); }
    }
    m_counter = (m_counter + 1) % m_divisor;
}

// OK
void Channel_EZ::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;

    EZ_Metadata* p = static_cast<EZ_Metadata*>(metadata);
    ULONG full_size = encoded_size + metadata_size;
    float4x4 pose;
    WSABUF wsaBuf[5];

    pack_buffer(wsaBuf, 0, &sample_time, sizeof(sample_time));
    pack_buffer(wsaBuf, 1, &full_size,   sizeof(full_size));
    pack_buffer(wsaBuf, 2, encoded,      encoded_size);
    pack_buffer(wsaBuf, 3, p,            metadata_size);
    pack_buffer(wsaBuf, 4, &pose,        sizeof(pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EZ::Execute_Mode0(bool enable_location)
{
    H26xFormat format;
    ZABFormat zabformat;
    std::vector<uint64_t> options;
    winrt::hstring subtype;
    uint32_t media_index;
    uint32_t stride_mask;
    bool ok;

    if (!ExtendedDepth_Status()) { return; }

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveZABFormat_Profile(m_socket_client, m_event_client, zabformat);
    if (!ok) { return; }

    ok = ReceiveEncoderOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    TranslateEncoderOptions(options, media_index, stride_mask);

    ok = ExtendedDepth_SetFormat(media_index, format.width, format.height, format.framerate, subtype);
    if (!ok) { return; }

    format.width = static_cast<uint16_t>((format.width + stride_mask) & ~stride_mask);

    m_pEncoder        = std::make_unique<Encoder_EZ>(Thunk_Encoder, this, format, zabformat);
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;
    m_width           = format.width;
    m_height          = format.height;

    ExtendedDepth_ExecuteSensorLoop(MediaFrameReaderAcquisitionMode::Realtime, Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
Channel_EZ::Channel_EZ(char const* name, char const* port, uint32_t id) : 
Channel(name, port, id)
{
}

// OK
bool Channel_EZ::Startup()
{
    SetNoDelay(true);
    return true;
}

// OK
void Channel_EZ::Run()
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

    if (ExtendedDepth_Status()) { ExtendedDepth_Close(); }

    ExtendedDepth_Open(options);
    }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    }

    if (mode & 8)
    {
    if (ExtendedDepth_Status()) { ExtendedDepth_Close(); }
    }
}

// OK
void Channel_EZ::Cleanup()
{
}

// OK
void EZ_Startup()
{
    g_channel = std::make_unique<Channel_EZ>("EZ", PORT_NAME_EZ, PORT_ID_EZ);
}

// OK
void EZ_Cleanup()
{
    g_channel.reset();
}
