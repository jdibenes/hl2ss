
#include "server.h"
#include "locator.h"
#include "extended_eye_tracking.h"
#include "extended_execution.h"
#include "ports.h"
#include "timestamps.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
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

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL;
static HANDLE g_thread = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void EET_Stream(SOCKET clientsocket, SpatialLocator const &locator, uint64_t utc_offset)
{
    PerceptionTimestamp ts = nullptr;
    EyeGazeTrackerReading egtr = nullptr;
    DateTime td;
    EET_Packet eet_packet;
    uint8_t fps;
    uint64_t delay;
    int64_t max_delta;
    int fps_index;
    bool cg_valid;
    bool lg_valid;
    bool rg_valid;
    bool lo_valid;
    bool ro_valid;
    bool vd_valid;
    bool ec_valid;    
    WSABUF wsaBuf[1];
    bool ok;

    ok = recv_u8(clientsocket, fps);
    if (!ok) { return; }

    switch (fps)
    {
    case 30: fps_index = 0; break;
    case 60: fps_index = 1; break;
    case 90: fps_index = 2; break;
    default: return;
    }

    max_delta = HNS_BASE / fps;
    delay = max_delta * 1;

    ExtendedEyeTracking_SetTargetFrameRate(fps_index);

    pack_buffer(wsaBuf, 0, &eet_packet, sizeof(eet_packet));

    eet_packet.size = sizeof(uint32_t) + sizeof(EET_Frame);
    eet_packet._reserved = 0;

    do
    {
    eet_packet.timestamp = GetCurrentUTCTimestamp() - delay;

    Sleep(1000 / fps);
    
    egtr = ExtendedEyeTracking_GetReading(DateTime(QPCTimestampToTimeSpan(eet_packet.timestamp)), max_delta);

    if (egtr)
    {
    eet_packet.timestamp = egtr.Timestamp().time_since_epoch().count() - utc_offset;

    cg_valid = egtr.TryGetCombinedEyeGazeInTrackerSpace(eet_packet.frame.c_origin, eet_packet.frame.c_direction);
    lg_valid = egtr.TryGetLeftEyeGazeInTrackerSpace(eet_packet.frame.l_origin, eet_packet.frame.l_direction);
    rg_valid = egtr.TryGetRightEyeGazeInTrackerSpace(eet_packet.frame.r_origin, eet_packet.frame.r_direction);
    lo_valid = egtr.TryGetLeftEyeOpenness(eet_packet.frame.l_openness);
    ro_valid = egtr.TryGetRightEyeOpenness(eet_packet.frame.r_openness);
    vd_valid = egtr.TryGetVergenceDistance(eet_packet.frame.vergence_distance);
    ec_valid = egtr.IsCalibrationValid();

    eet_packet.frame.valid = (vd_valid << 6) | (ro_valid << 5) | (lo_valid << 4) | (rg_valid << 3) | (lg_valid << 2) | (cg_valid << 1) | (ec_valid << 0);

    ts = QPCTimestampToPerceptionTimestamp(eet_packet.timestamp);
    eet_packet.pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    }
    else
    {
    eet_packet.timestamp -= utc_offset;
    eet_packet.frame.valid = 0;
    }

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    }
    while (ok);
}

// OK
static DWORD WINAPI EET_EntryPoint(void* param)
{
    (void)param;

    SpatialLocator locator = nullptr;
    uint64_t utc_offset;
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket
    int base_priority;

    ShowMessage("EET: Waiting for consent");

    ExtendedEyeTracking_Initialize();
    locator = ExtendedEyeTracking_CreateLocator();
    utc_offset = GetQPCToUTCOffset(32);

    listensocket = CreateSocket(PORT_NAME_EET);

    ShowMessage("EET: Listening at port %s", PORT_NAME_EET);

    base_priority = GetThreadPriority(GetCurrentThread());

    do
    {
    ShowMessage("EET: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("EET: Client connected");

    SetThreadPriority(GetCurrentThread(), ExtendedExecution_GetInterfacePriority(PORT_NUMBER_EET - PORT_NUMBER_BASE));

    EET_Stream(clientsocket, locator, utc_offset);

    SetThreadPriority(GetCurrentThread(), base_priority);

    closesocket(clientsocket);

    ShowMessage("EET: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("EET: Closed");

    return 0;
}

// OK
void EET_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, EET_EntryPoint, NULL, 0, NULL);
}

// OK
void EET_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void EET_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_quit);
}
