
#include "extended_audio.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_ea.h"
#include "timestamp.h"
#include "types.h"

#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture::Frames;

class Channel_EA : public Channel
{
private:
    std::unique_ptr<Encoder_EA> m_pEncoder;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(ExtendedAudio_Control const& control, AACFormat& format);
    void Execute_Mode2(MRCAudioOptions const& options, ExtendedAudio_Control const& control);

    void OnFrameArrived(MediaFrameReference const& frame);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(MediaFrameReference const& frame, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

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
void Channel_EA::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_EA*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_EA::OnFrameArrived(MediaFrameReference const& frame)
{
    m_pEncoder->WriteSample(frame);
}

// OK
void Channel_EA::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)metadata;
    (void)metadata_size;

    WSABUF wsaBuf[3];

    pack_buffer(wsaBuf, 0, &sample_time,  sizeof(sample_time));
    pack_buffer(wsaBuf, 1, &encoded_size, sizeof(encoded_size));
    pack_buffer(wsaBuf, 2, encoded,       encoded_size);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EA::Execute_Mode2(MRCAudioOptions const& options, ExtendedAudio_Control const& control)
{
    winrt::hstring query = ExtendedAudio_QueryDevices(options, control);
    WSABUF wsaBuf[2];    

    uint32_t bytes = query.size() * sizeof(wchar_t);

    pack_buffer(wsaBuf, 0, &bytes,        sizeof(bytes));
    pack_buffer(wsaBuf, 1, query.c_str(), bytes);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EA::Execute_Mode0(ExtendedAudio_Control const& control, AACFormat& format)
{
    AudioSubtype subtype;
    uint32_t channels;
    uint32_t samplerate;

    ExtendedAudio_GetCurrentFormat(subtype, channels, samplerate);

    if (control.enable_passthrough)
    {
    format.samplerate = static_cast<uint16_t>(samplerate);
    format.channels   = static_cast<uint8_t>(channels);
    subtype           = AudioSubtype::AudioSubtype_S16;
    }
    else
    {
    format.samplerate = 48000;
    format.channels   = 2;
    }

    m_pEncoder = std::make_unique<Encoder_EA>(Thunk_Encoder, this, subtype, format, channels, control.enable_passthrough);

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
    SetNoDelay(true);
    return true;
}

// OK
void Channel_EA::Run()
{
    MRCAudioOptions options;
    AACFormat format;
    ExtendedAudio_Control control;
    bool ok;

    ok = ReceiveMRCAudioOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    ok = ReceiveAACFormat_Profile(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    if (format.profile == AACProfile::AACProfile_None)
    {
    control.enable_passthrough = bit_test( format.level, 7);
    }
    else
    {
    control.enable_passthrough = false;
    }

    control.query              = bit_field(options.mixer_mode,  0, 3) == 3;
    control.device_index       = bit_field(options.mixer_mode,  2, 0x3FF);

    if (control.enable_passthrough)
    {
    control.source_index       = bit_field(options.mixer_mode, 12, 0x3FF);
    control.format_index       = bit_field(options.mixer_mode, 22, 0x3FF);
    control.media_category     = bit_field(format.level, 0, 0x07);
    control.shared             = bit_test( format.level, 3);
    control.audio_raw          = bit_test( format.level, 5);
    control.disable_effect     = bit_test( format.level, 6);
    }
    else
    {
    control.source_index       = ~0UL; // X
    control.format_index       = ~0UL; // X
    control.media_category     = 2;
    control.shared             = true;
    control.audio_raw          = false;
    control.disable_effect     = false;
    }

    if (control.query)
    {
    Execute_Mode2(options, control);
    return;
    }

    ExtendedAudio_Open(options, control);

    if (!ExtendedAudio_Status()) { return; }

    Execute_Mode0(control, format);

    ExtendedAudio_Close();
}

// OK
void Channel_EA::Cleanup()
{
}

// OK
void EA_Startup()
{
    g_channel = std::make_unique<Channel_EA>("EA", PORT_NAME_EA, PORT_ID_EA);
}

// OK
void EA_Cleanup()
{
    g_channel.reset();
}
