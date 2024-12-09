
#include "extended_audio.h"
#include "channel.h"
#include "ipc_sc.h"
#include "ports.h"
#include "encoder_ea.h"
#include "timestamps.h"

#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture::Frames;

class Channel_EA : public Channel
{
private:
    std::unique_ptr<Encoder_EA> m_pEncoder;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0();
    void Execute_Mode2();

    void OnFrameArrived(MediaFrameReference const& frame);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(MediaFrameReference const& frame, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, LONGLONG sample_time, void* metadata, UINT32 metadat_size, void* self);

public:
    Channel_EA(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_EA> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_EA::Thunk_Sensor(MediaFrameReference const& frame, void* self)
{
    static_cast<Channel_EA*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_EA::Thunk_Encoder(void* encoded, DWORD encoded_size, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_EA*>(self)->OnEncodingComplete(encoded, encoded_size, sample_time, metadata, metadata_size);
}

// OK
void Channel_EA::OnFrameArrived(MediaFrameReference const& frame)
{
    m_pEncoder->WriteSample(frame);
}

// OK
void Channel_EA::OnEncodingComplete(void* encoded, DWORD encoded_size, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)metadata;
    (void)metadata_size;

    WSABUF wsaBuf[3];

    pack_buffer(wsaBuf, 0, &sample_time,  sizeof(sample_time));
    pack_buffer(wsaBuf, 1, &encoded_size, sizeof(encoded_size));
    pack_buffer(wsaBuf, 2, encoded,       encoded_size);

    bool ok = send_multiple(m_socket_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(m_event_client); }
}

// OK
void Channel_EA::Execute_Mode2()
{
    winrt::hstring query;
    WSABUF wsaBuf[2];

    ExtendedAudio_QueryDevices(query);

    uint32_t bytes = query.size() * sizeof(wchar_t);

    pack_buffer(wsaBuf, 0, &bytes,        sizeof(bytes));
    pack_buffer(wsaBuf, 1, query.c_str(), bytes);

    send_multiple(m_socket_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EA::Execute_Mode0()
{
    AACFormat format;
    AudioSubtype subtype;
    uint32_t channels;
    bool ok;

    Encoder_EA::SetAACFormat(format);

    ok = ReceiveAACFormat_Profile(m_socket_client, format);
    if (!ok) { return; }

    ExtendedAudio_GetCurrentFormat(subtype, channels);

    m_pEncoder = std::make_unique<Encoder_EA>(Thunk_Encoder, this, subtype, format, channels);

    ExtendedAudio_ExecuteSensorLoop(Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
Channel_EA::Channel_EA(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_EA::Startup()
{
    return true;
}

// OK
void Channel_EA::Run()
{
    MRCAudioOptions options;    
    bool ok;

    ok = ReceiveMRCAudioOptions(m_socket_client, options);
    if (!ok) { return; }

    if (options.mixer_mode & 0x80000000)
    {
    Execute_Mode2();
    return;
    }

    ok = ExtendedAudio_Open(options);
    if (!ok) { return; }

    Execute_Mode0();

    ExtendedAudio_Close();
}

// OK
void Channel_EA::Cleanup()
{
}

// OK
void EA_Initialize()
{
    g_channel = std::make_unique<Channel_EA>("EA", PORT_NAME_EA, PORT_NUMBER_EA - PORT_NUMBER_BASE);
}

// OK
void EA_Cleanup()
{
    g_channel.reset();
}
