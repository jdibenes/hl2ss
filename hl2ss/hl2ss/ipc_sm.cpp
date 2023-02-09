
#include <Windows.h>
#include "server.h"
#include "ports.h"
#include "spatial_mapping.h"
#include "log.h"

#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL;
static HANDLE g_quitevent = NULL;
static HANDLE g_clientevent = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SM_TransferError()
{
    SetEvent(g_clientevent);
}

// OK
static void SM_MSG_CreateObserver(SOCKET clientsocket)
{
    (void)clientsocket;
    SpatialMapping_CreateObserver();
}

// OK
static void SM_MSG_SetVolumes(SOCKET clientsocket)
{
    std::vector<VolumeDescription> vd;
    uint32_t type;
    uint8_t count;
    int size;
    bool ok;

    ok = recv_u8(clientsocket, count);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    vd.resize(count);
    VolumeDescription* data = vd.data();

    for (int i = 0; i < count; ++i)
    {
    ok = recv_u32(clientsocket, type);
    if (!ok) 
    {
        SM_TransferError();
        return;
    }

    switch (type)
    {
    case VolumeType::VolumeType_Box:         size = sizeof(SpatialBoundingBox);         break;
    case VolumeType::VolumeType_Frustum:     size = sizeof(SpatialBoundingFrustum);     break;
    case VolumeType::VolumeType_OrientedBox: size = sizeof(SpatialBoundingOrientedBox); break;
    case VolumeType::VolumeType_Sphere:      size = sizeof(SpatialBoundingSphere);      break;
    default:
        SM_TransferError();
        return;
    }

    vd[i].type = (VolumeType)type;
    ok = recv(clientsocket, (char*)&(data[i].data), size);
    if (!ok)
    {
        SM_TransferError();
        return;
    }
    }

    SpatialMapping_SetVolumes(data, count);
}

// OK
static void SM_MSG_GetObservedSurfaces(SOCKET clientsocket)
{
    WSABUF wsaBuf[2];
    winrt::guid const* ids;
    size_t count;
    bool ok;

    SpatialMapping_GetObservedSurfaces(ids, count);
    
    wsaBuf[0].buf = (char*)&count;
    wsaBuf[0].len = sizeof(count);

    wsaBuf[1].buf = (char*)ids;
    wsaBuf[1].len = (ULONG)(count * sizeof(winrt::guid));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SM_TransferError(); }
}

// OK
static void SM_MSG_GetMeshes(SOCKET clientsocket)
{
    std::vector<MeshTask> md;
    uint32_t count;
    uint32_t maxtasks;
    MeshTask* task;
    MeshInfo* info;
    WSABUF wsaBuf[4];
    bool ok;

    ok = recv_u32(clientsocket, count);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    ok = recv_u32(clientsocket, maxtasks);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    md.resize(count);
    task = md.data();

    for (uint32_t i = 0; i < count; ++i)
    {
    ok = recv(clientsocket, (char*)(task + i), sizeof(MeshDescription));
    if (!ok)
    {
        SM_TransferError();
        return;
    }
    }

    SpatialMapping_BeginComputeMeshes(task, count, maxtasks);

    for (; count > 0; --count)
    {
    info = SpatialMapping_GetNextMesh();

    wsaBuf[0].buf = (char*)info;
    wsaBuf[0].len = MESH_INFO_HEADER_SIZE;

    wsaBuf[1].buf = (char*)info->vpd;
    wsaBuf[1].len = info->vpl;

    wsaBuf[2].buf = (char*)info->tid;
    wsaBuf[2].len = info->til;

    wsaBuf[3].buf = (char*)info->vnd;
    wsaBuf[3].len = info->vnl;

    ok = send_multiple(clientsocket, wsaBuf, (info->status == 0) ? (sizeof(wsaBuf) / sizeof(WSABUF)) : 1);
    if (!ok)
    {
        SM_TransferError();
        break;
    }
    }

    SpatialMapping_EndComputeMeshes();
}

// OK
static void SM_Dispatch(SOCKET clientsocket)
{
    uint8_t state;
    bool ok;

    ok = recv_u8(clientsocket, state);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    switch (state)
    {
    case 0x00: SM_MSG_CreateObserver(clientsocket);      break;
    case 0x01: SM_MSG_SetVolumes(clientsocket);          break;
    case 0x02: SM_MSG_GetObservedSurfaces(clientsocket); break;
    case 0x03: SM_MSG_GetMeshes(clientsocket);           break;
    default:
        SM_TransferError();
        return;
    }
}

// OK
static void SM_Translate(SOCKET clientsocket)
{
    g_clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    do { SM_Dispatch(clientsocket); } while (WaitForSingleObject(g_clientevent, 0) == WAIT_TIMEOUT);
    CloseHandle(g_clientevent);
}

// OK
static DWORD WINAPI SM_EntryPoint(void* param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    ShowMessage("SM: Waiting for consent");

    SpatialMapping_WaitForConsent();

    listensocket = CreateSocket(PORT_NAME_SM);

    ShowMessage("SM: Listening at port %s", PORT_NAME_SM);

    do
    {
    ShowMessage("SM: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("SM: Client connected");

    SM_Translate(clientsocket);

    closesocket(clientsocket);

    ShowMessage("SM: Client disconnected");
    }
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SM: Closed");

    return 0;
}

// OK
void SM_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SM_EntryPoint, NULL, 0, NULL);
}

// OK
void SM_Quit()
{
    SetEvent(g_quitevent);
}

// OK
void SM_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);

    g_thread = NULL;
    g_quitevent = NULL;
}
