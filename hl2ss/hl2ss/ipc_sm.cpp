
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
static HANDLE g_event_quit = NULL;
static HANDLE g_event_client = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SM_TransferError()
{
    SetEvent(g_event_client);
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
    std::vector<SpatialMapping_VolumeDescription> vd;
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
    SpatialMapping_VolumeDescription* data = vd.data();

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
    case SpatialMapping_VolumeType::VolumeType_Box:         size = sizeof(SpatialBoundingBox);         break;
    case SpatialMapping_VolumeType::VolumeType_Frustum:     size = sizeof(SpatialBoundingFrustum);     break;
    case SpatialMapping_VolumeType::VolumeType_OrientedBox: size = sizeof(SpatialBoundingOrientedBox); break;
    case SpatialMapping_VolumeType::VolumeType_Sphere:      size = sizeof(SpatialBoundingSphere);      break;
    default:
        SM_TransferError();
        return;
    }

    vd[i].type = (SpatialMapping_VolumeType)type;
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
    SpatialMapping_SurfaceInfo const* ids;
    size_t count;
    bool ok;

    SpatialMapping_GetObservedSurfaces(ids, count);

    pack_buffer(wsaBuf, 0, &count, sizeof(count));
    pack_buffer(wsaBuf, 1, ids, (ULONG)(count * sizeof(SpatialMapping_SurfaceInfo)));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        SM_TransferError();
        return;
    }
}

// OK
static void SM_MSG_GetMeshes(SOCKET clientsocket)
{
    std::vector<SpatialMapping_MeshTask> md;
    uint32_t count;
    uint32_t maxtasks;
    SpatialMapping_MeshTask* task;
    SpatialMapping_MeshInfo* info;
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
    ok = recv(clientsocket, (char*)(task + i), sizeof(SpatialMapping_MeshDescription));
    if (!ok)
    {
        SM_TransferError();
        return;
    }
    }

    SpatialMapping_BeginComputeMeshes(task, count, maxtasks);

    while (count > 0)
    {
    --count;

    info = SpatialMapping_GetNextMesh();

    pack_buffer(wsaBuf, 0, info, SM_MESH_INFO_HEADER_SIZE + info->bsz);
    pack_buffer(wsaBuf, 1, info->vpd, info->vpl);
    pack_buffer(wsaBuf, 2, info->tid, info->til);
    pack_buffer(wsaBuf, 3, info->vnd, info->vnl);

    ok = send_multiple(clientsocket, wsaBuf, (info->status == 0) ? (sizeof(wsaBuf) / sizeof(WSABUF)) : 1);
    
    SpatialMapping_DestroyMesh(info);
    
    if (!ok)
    {
        SM_TransferError();
        break;
    }
    }

    while (count > 0)
    {
    --count;

    info = SpatialMapping_GetNextMesh();
    SpatialMapping_DestroyMesh(info);
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
    ResetEvent(g_event_client);
    do { SM_Dispatch(clientsocket); } while (WaitForSingleObject(g_event_client, 0) == WAIT_TIMEOUT);
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
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SM: Closed");

    return 0;
}

// OK
void SM_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SM_EntryPoint, NULL, 0, NULL);
}

// OK
void SM_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void SM_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_client = NULL;
    g_event_quit = NULL;
}
