
#include "extended_eye_tracking.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_eet.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Microsoft::MixedReality::EyeTracking;

class Channel_EET : public Channel
{
private:
    std::unique_ptr<Encoder_EET> m_pEncoder;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode1();

    void OnFrameArrived(EyeGazeTrackerReading const& frame, UINT64 host_ticks);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(EyeGazeTrackerReading const& frame, UINT64 host_ticks, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_EET(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_EET> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_EET::Thunk_Sensor(EyeGazeTrackerReading const& frame, UINT64 host_ticks, void* self)
{
    static_cast<Channel_EET*>(self)->OnFrameArrived(frame, host_ticks);
}

// OK
void Channel_EET::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_EET*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_EET::OnFrameArrived(EyeGazeTrackerReading const& frame, UINT64 host_ticks)
{
    float4x4 pose = ExtendedEyeTracking_GetNodeWorldPose(host_ticks);
    m_pEncoder->WriteSample(frame, pose, host_ticks);
}

// OK
void Channel_EET::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata;
    (void)metadata_size;

    WSABUF wsaBuf[1];
    pack_buffer(wsaBuf, 0, encoded, encoded_size);
    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EET::Execute_Mode1()
{
    uint8_t fps;
    bool ok;

    ok = ReceiveEETFramerate(m_socket_client, m_event_client, fps);
    if (!ok) { return; }

    ok = ExtendedEyeTracking_SetTargetFrameRate(fps);
    if (!ok) { return; }

    m_pEncoder = std::make_unique<Encoder_EET>(Thunk_Encoder, this);

    ExtendedEyeTracking_ExecuteSensorLoop(Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
Channel_EET::Channel_EET(char const* name, char const* port, uint32_t id) : 
Channel(name, port, id)
{
}

// OK
bool Channel_EET::Startup()
{
    SetNoDelay(true);
    return ExtendedEyeTracking_WaitForConsent();
}

// OK
void Channel_EET::Run()
{
    ExtendedEyeTracking_Open(true);

    if (!ExtendedEyeTracking_Status()) { return; }

    Execute_Mode1();

    ExtendedEyeTracking_Close();
}

// OK
void Channel_EET::Cleanup()
{
}

// OK
void EET_Startup()
{
    g_channel = std::make_unique<Channel_EET>("EET", PORT_NAME_EET, PORT_ID_EET);
}

// OK
void EET_Cleanup()
{
    g_channel.reset();
}
