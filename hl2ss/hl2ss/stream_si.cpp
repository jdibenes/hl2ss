
#include <queue>
#include "server.h"
#include "locator.h"
#include "spatial_input.h"
#include "ports.h"
#include "log.h"
#include "lock.h"

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

static bool g_enable = false;
static std::queue<PerceptionTimestamp> g_queue;
static HANDLE g_semaphore = NULL; // CloseHandle
static CRITICAL_SECTION g_lock; // DeleteCriticalSection
static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_quitevent = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SI_Stream(SOCKET clientsocket)
{
    int const hand_size = HAND_JOINTS * sizeof(JointPose);
    int32_t const packet_size = sizeof(uint8_t) + sizeof(Frame) + sizeof(Ray) + (2 * hand_size);

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
    WSABUF wsaBuf[7];
    bool ok;
    
    left_poses.resize(HAND_JOINTS);
    right_poses.resize(HAND_JOINTS);

    wsaBuf[0].buf = (char*)&qpc;
    wsaBuf[0].len = (ULONG)sizeof(qpc);

    wsaBuf[1].buf = (char*)&packet_size;
    wsaBuf[1].len = sizeof(packet_size);

    wsaBuf[2].buf = (char*)&valid;
    wsaBuf[2].len = (ULONG)sizeof(valid);

    wsaBuf[3].buf = (char*)&head_pose;
    wsaBuf[3].len = (ULONG)sizeof(head_pose);

    wsaBuf[4].buf = (char*)&eye_ray;
    wsaBuf[4].len = (ULONG)sizeof(eye_ray);

    wsaBuf[5].buf = (char*)left_poses.data();
    wsaBuf[5].len = hand_size;

    wsaBuf[6].buf = (char*)right_poses.data();
    wsaBuf[6].len = hand_size;

    {
    CriticalSection cs(&g_lock);
    g_enable = true;
    g_semaphore = CreateSemaphore(NULL, 0, LONG_MAX, NULL);    
    }

    do
    {
    WaitForSingleObject(g_semaphore, INFINITE);

    {
    CriticalSection cs(&g_lock);
    ts = g_queue.front();
    g_queue.pop();
    }

    world = Locator_GetWorldCoordinateSystem(ts);
    status1 = SpatialInput_GetHeadPoseAndEyeRay(world, ts, head_pose, eye_ray);
    status2 = SpatialInput_GetHandPose(world, ts, left_poses, right_poses);
    qpc = ts.SystemRelativeTargetTime().count();
    valid = (status1 | (status2 << 2)) & 0x0F;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    }
    while (ok);

    CriticalSection cs(&g_lock);
    g_enable = false;
    g_queue = {};
    CloseHandle(g_semaphore);
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
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SI: Closed");

    return 0;
}

// OK
void SI_NotifyNextFrame(PerceptionTimestamp const& ts)
{
    CriticalSection cs(&g_lock);
    if (!g_enable) { return; }
    g_queue.push(ts);
    ReleaseSemaphore(g_semaphore, 1, NULL);
}

// OK
void SI_Initialize()
{
    InitializeCriticalSection(&g_lock);
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SI_EntryPoint, NULL, 0, NULL);
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

    DeleteCriticalSection(&g_lock);
}
