
#include <queue>
#include "server.h"
#include "locator.h"
#include "spatial_input.h"
#include "ports.h"
#include "timestamps.h"
#include "log.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::People;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SI_Stream(SOCKET clientsocket)
{
    int const hand_size = HAND_JOINTS * sizeof(JointPose);
    int32_t const packet_size = sizeof(uint32_t) + sizeof(SpatialInput_Frame) + sizeof(SpatialInput_Ray) + (2 * hand_size);

    PerceptionTimestamp ts = nullptr;
    SpatialCoordinateSystem world = nullptr;
    UINT64 qpc;
    int status1;
    int status2;
    uint32_t valid;
    std::vector<JointPose> left_poses;
    std::vector<JointPose> right_poses;
    SpatialInput_Frame head_pose;
    SpatialInput_Ray eye_ray;
    WSABUF wsaBuf[7];
    bool ok;
    
    left_poses.resize(HAND_JOINTS);
    right_poses.resize(HAND_JOINTS);

    pack_buffer(wsaBuf, 0, &qpc, sizeof(qpc));
    pack_buffer(wsaBuf, 1, &packet_size, sizeof(packet_size));
    pack_buffer(wsaBuf, 2, &valid, sizeof(valid));
    pack_buffer(wsaBuf, 3, &head_pose, sizeof(head_pose));
    pack_buffer(wsaBuf, 4, &eye_ray, sizeof(eye_ray));
    pack_buffer(wsaBuf, 5, left_poses.data(), hand_size);
    pack_buffer(wsaBuf, 6, right_poses.data(), hand_size);

    do
    {
    Sleep(1000 / 30);

    qpc = GetCurrentQPCTimestamp();
    ts = QPCTimestampToPerceptionTimestamp(qpc);

    world   = Locator_GetWorldCoordinateSystem(ts);
    status1 = SpatialInput_GetHeadPoseAndEyeRay(world, ts, head_pose, eye_ray);
    status2 = SpatialInput_GetHandPose(world, ts, left_poses, right_poses);
    valid   = (status1 | (status2 << 2)) & 0x0F;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    }
    while (ok);
}

// OK
static DWORD WINAPI SI_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    ShowMessage("SI: Waiting for consent");

    SpatialInput_WaitForEyeConsent();

    listensocket = CreateSocket(PORT_NAME_SI);

    ShowMessage("SI: Listening at port %s", PORT_NAME_SI);

    do
    {
    ShowMessage("SI: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("SI: Client connected");

    SI_Stream(clientsocket);

    closesocket(clientsocket);

    ShowMessage("SI: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SI: Closed");

    return 0;
}

// OK
void SI_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SI_EntryPoint, NULL, 0, NULL);
}

// OK
void SI_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void SI_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_quit);
}
