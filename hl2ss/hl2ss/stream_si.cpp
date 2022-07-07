
#include "server.h"
#include "spatial_input.h"
#include "locator.h"
#include "utilities.h"
#include "ports.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::People;

HANDLE g_thread = NULL; // CloseHandle
HANDLE g_quitevent = NULL; // CloseHandle

static void SI_Stream(SOCKET clientsocket)
{
    PerceptionTimestamp ts = nullptr;
    UINT64 qpc;
    Frame head_pose;
    Ray eye_ray;
    WSABUF wsaBuf[3];
    bool ok;

    do
    {
    Sleep(33);

    qpc = GetCurrentQPCTimeHns();
    ts = QPCTimestampToPerceptionTimestamp(qpc);
    memset(&head_pose, 0, sizeof(head_pose));
    memset(&eye_ray, 0, sizeof(eye_ray));
    SpatialInput_GetHeadPoseAndEyeRay(ts, head_pose, eye_ray);

    wsaBuf[0].buf = (char*)&qpc;
    wsaBuf[0].len = (ULONG)sizeof(qpc);

    wsaBuf[1].buf = (char*)&head_pose;
    wsaBuf[1].len = (ULONG)sizeof(head_pose);

    wsaBuf[2].buf = (char*)&eye_ray;
    wsaBuf[2].len = (ULONG)sizeof(eye_ray);

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    }
    while (ok);
}

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

void SI_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SI_EntryPoint, NULL, 0, NULL);
}

void SI_Quit()
{
    SetEvent(g_quitevent);
}

void SI_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);

    g_thread = NULL;
    g_quitevent = NULL;
}
