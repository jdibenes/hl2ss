
#include "server.h"
#include "holographic_space.h"
#include "locator.h"
#include "spatial_input.h"
#include "utilities.h"
#include "ports.h"

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
static HANDLE g_quitevent = NULL; // CloseHandle
static HANDLE g_dataevent = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SI_Stream(SOCKET clientsocket)
{
    int const hand_size = HAND_JOINTS * sizeof(JointPose);

    PerceptionTimestamp ts = nullptr;
    SpatialCoordinateSystem world = nullptr;
    UINT64 qpc;
    int status1;
    int status2;
    uint8_t valid;
    std::vector<JointPose> left_poses;
    std::vector<JointPose> right_poses;
    Frame head_pose;
    Ray eye_ray;
    WSABUF wsaBuf[6];
    bool ok;
    
    left_poses.resize(HAND_JOINTS);
    right_poses.resize(HAND_JOINTS);

    WaitForSingleObject(g_dataevent, INFINITE);

    do
    {
    WaitForSingleObject(g_dataevent, INFINITE);

    ts = HolographicSpace_GetTimestamp();
    world = Locator_GetWorldCoordinateSystem(ts);

    status1 = SpatialInput_GetHeadPoseAndEyeRay(world, ts, head_pose, eye_ray);
    status2 = SpatialInput_GetHandPose(world, ts, left_poses, right_poses);

    qpc = ts.SystemRelativeTargetTime().count();
    valid = (status1 | (status2 << 2)) & 0x0F;

    wsaBuf[0].buf = (char*)&qpc;
    wsaBuf[0].len = (ULONG)sizeof(qpc);

    wsaBuf[1].buf = (char*)&valid;
    wsaBuf[1].len = (ULONG)sizeof(valid);

    wsaBuf[2].buf = (char*)&head_pose;
    wsaBuf[2].len = (ULONG)sizeof(head_pose);

    wsaBuf[3].buf = (char*)&eye_ray;
    wsaBuf[3].len = (ULONG)sizeof(eye_ray);

    wsaBuf[4].buf = (char*)left_poses.data();
    wsaBuf[4].len = hand_size;

    wsaBuf[5].buf = (char*)right_poses.data();
    wsaBuf[5].len = hand_size;

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

    listensocket = CreateSocket(PORT_SI);

    ShowMessage("SI: Listening at port %s", PORT_SI);

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
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SI: Closed");

    return 0;
}

// OK
void SI_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_dataevent = CreateEvent(NULL, FALSE, FALSE, NULL);

    g_thread = CreateThread(NULL, 0, SI_EntryPoint, NULL, 0, NULL);
}

// OK
void SI_NotifyNextFrame()
{
    SetEvent(g_dataevent);
}

// OK
void SI_Quit()
{
    SetEvent(g_quitevent);
}

// OK
void SI_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);
    CloseHandle(g_dataevent);

    g_thread = NULL;
    g_quitevent = NULL;
    g_dataevent = NULL;
}
