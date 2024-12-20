
#include "extended_eye_tracking.h"
#include "server_channel.h"
#include "server_settings.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Microsoft::MixedReality::EyeTracking;

struct EET_Frame
{
    float3   c_origin;
    float3   c_direction;
    float3   l_origin;
    float3   l_direction;
    float3   r_origin;
    float3   r_direction;
    float    l_openness;
    float    r_openness;
    float    vergence_distance;
    uint32_t valid;
};

struct EET_Packet
{
    uint64_t  timestamp;
    uint32_t  size;
    uint32_t  _reserved;
    EET_Frame frame;
    float4x4  pose;
};

class Channel_EET : public Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode1();

    void OnFrameArrived(EyeGazeTrackerReading const& frame, UINT64 host_ticks);
    void OnEmptyArrived(UINT64 host_ticks);

    static void Thunk_Sensor(EyeGazeTrackerReading const& frame, UINT64 host_ticks, void* self);

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
    if (frame)
    {
    static_cast<Channel_EET*>(self)->OnFrameArrived(frame, host_ticks);
    }
    else
    {
    static_cast<Channel_EET*>(self)->OnEmptyArrived(host_ticks);
    }
}

// OK
void Channel_EET::OnFrameArrived(EyeGazeTrackerReading const& frame, UINT64 host_ticks)
{
    EET_Packet eet_packet;
    WSABUF wsaBuf[1];

    memset(&eet_packet, 0, sizeof(eet_packet));

    bool cg_valid = frame.TryGetCombinedEyeGazeInTrackerSpace(eet_packet.frame.c_origin, eet_packet.frame.c_direction);
    bool lg_valid = frame.TryGetLeftEyeGazeInTrackerSpace(eet_packet.frame.l_origin, eet_packet.frame.l_direction);
    bool rg_valid = frame.TryGetRightEyeGazeInTrackerSpace(eet_packet.frame.r_origin, eet_packet.frame.r_direction);
    bool lo_valid = frame.TryGetLeftEyeOpenness(eet_packet.frame.l_openness);
    bool ro_valid = frame.TryGetRightEyeOpenness(eet_packet.frame.r_openness);
    bool vd_valid = frame.TryGetVergenceDistance(eet_packet.frame.vergence_distance);
    bool ec_valid = frame.IsCalibrationValid();

    eet_packet.timestamp   = host_ticks;
    eet_packet.size        = sizeof(EET_Packet::_reserved) + sizeof(EET_Packet::frame);
    eet_packet._reserved   = 0;
    eet_packet.frame.valid = (vd_valid << 6) | (ro_valid << 5) | (lo_valid << 4) | (rg_valid << 3) | (lg_valid << 2) | (cg_valid << 1) | (ec_valid << 0);
    eet_packet.pose        = ExtendedEyeTracking_GetNodeWorldPose(host_ticks);

    pack_buffer(wsaBuf, 0, &eet_packet, sizeof(eet_packet));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_EET::OnEmptyArrived(UINT64 host_ticks)
{
    EET_Packet eet_packet;
    WSABUF wsaBuf[1];

    memset(&eet_packet, 0, sizeof(eet_packet));

    eet_packet.timestamp = host_ticks;
    eet_packet.size      = sizeof(EET_Packet::_reserved) + sizeof(EET_Packet::frame);

    pack_buffer(wsaBuf, 0, &eet_packet, sizeof(eet_packet));

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

    ExtendedEyeTracking_ExecuteSensorLoop(Thunk_Sensor, this, m_event_client);
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
