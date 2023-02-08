
#include <Windows.h>
#include "server.h"
#include "ports.h"
#include "spatial_mapping.h"
#include "log.h"

#include <winrt/Windows.Storage.Streams.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Surfaces;
using namespace winrt::Windows::Storage::Streams;

struct MeshStatus
{
    uint32_t index;
    uint32_t status;
};

static HANDLE g_thread = NULL;
static HANDLE g_quitevent = NULL;
static HANDLE g_clientevent = NULL;

static void SM_TransferError()
{
    SetEvent(g_clientevent);
}

static void SM_MSG_CreateObserver(SOCKET clientsocket)
{
    (void)clientsocket;
    SpatialMapping_CreateObserver();
    ShowMessage("SM: CreateObserver");
}

static void SM_MSG_SetVolumes(SOCKET clientsocket)
{
    std::vector<VolumeDescription> desc;
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

    desc.resize(count);
    VolumeDescription* data = desc.data();

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

    desc[i].type = (VolumeType)type;
    ok = recv(clientsocket, (char*)&(data[i].data), size);
    if (!ok)
    {
        SM_TransferError();
        return;
    }
    }

    SpatialMapping_SetVolumes(desc);
    ShowMessage("SM SetVolume of Type %d", type);
}

static void SM_MSG_GetObservedSurfaces(SOCKET clientsocket)
{
    WSABUF wsaBuf[2];
    winrt::guid const* ids;
    uint32_t count;
    bool ok;

    SpatialMapping_GetObservedSurfaces();
    SpatialMapping_ReportIDs(ids, count);
    
    wsaBuf[0].buf = (char*)&count;
    wsaBuf[0].len = sizeof(count);

    wsaBuf[1].buf = (char*)ids;
    wsaBuf[1].len = count * sizeof(winrt::guid);

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SM_TransferError(); }

    ShowMessage("SM: Got %d surfaces", count);
}

static void SM_MSG_GetMeshes(SOCKET clientsocket)
{
    SpatialSurfaceMesh mesh = nullptr;
    std::vector<MeshTask> md;
    uint32_t count;
    uint32_t maxtasks;
    MeshTask* data;
    MeshStatus ms;
    float3 scale;
    IBuffer vertex_positions;
    uint8_t* vertex_positions_data;
    uint32_t vertex_positions_length;
    IBuffer triangle_indices;
    uint8_t* triangle_indices_data;
    uint32_t triangle_indices_length;
    IBuffer vertex_normals;
    uint8_t* vertex_normals_data;
    uint32_t vertex_normals_length;
    WSABUF wsaBufStatus[1];
    WSABUF wsaBufData[7];
    bool ok;

    ok = recv_u32(clientsocket, count);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    ShowMessage("SM: GetMesh count %d", count);

    ok = recv_u32(clientsocket, maxtasks);
    if (!ok)
    {
        SM_TransferError();
        return;
    }

    ShowMessage("SM: GetMesh maxtasks %d", maxtasks);

    md.resize(count);
    data = md.data();

    for (uint32_t i = 0; i < count; ++i)
    {
    ok = recv(clientsocket, (char*)&data[i], sizeof(MeshDescription));
    if (!ok)
    {
        SM_TransferError();
        return;
    }
    }

    ShowMessage("SM: GetMesh Desc %f %d %d %d %d", md[0].md.maxtpcm, md[0].md.vertex_format, md[0].md.triangle_format, md[0].md.normal_format, md[0].md.normals);

    ShowMessage("SM: BeginComputeMeshes");

    SpatialMapping_BeginComputeMeshes(md, maxtasks);

    for (; count > 0; --count)
    {
    ms.index = SpatialMapping_WaitComputeMeshes();
    ms.status = SpatialMapping_GetStatusComputeMeshes(ms.index);

    wsaBufStatus[0].buf = (char*)&ms;
    wsaBufStatus[0].len = sizeof(ms);

    ShowMessage("SM: index %d status %d", ms.index, ms.status);

    ok = send_multiple(clientsocket, wsaBufStatus, sizeof(wsaBufStatus) / sizeof(WSABUF));
    if (!ok)
    {
        SM_TransferError();
        break;
    }

    if (ms.status != 0) { continue; }

    SpatialMapping_GetMeshComputeMeshes(ms.index, mesh);

    scale            = mesh.VertexPositionScale();
    vertex_positions = mesh.VertexPositions().Data();
    triangle_indices = mesh.TriangleIndices().Data();
    vertex_normals   = mesh.VertexNormals().Data();

    vertex_positions_data   = vertex_positions.data();
    vertex_positions_length = vertex_positions.Length();
    triangle_indices_data   = triangle_indices.data();
    triangle_indices_length = triangle_indices.Length();
    vertex_normals_data     = vertex_normals.data();
    vertex_normals_length   = vertex_normals.Length();

    wsaBufData[0].buf = (char*)&scale;
    wsaBufData[0].len = sizeof(scale);

    wsaBufData[1].buf = (char*)&vertex_positions_length;
    wsaBufData[1].len = sizeof(vertex_positions_length);

    ShowMessage("vpl %d", vertex_positions_length);

    wsaBufData[2].buf = (char*)vertex_positions_data;
    wsaBufData[2].len = vertex_positions_length;

    wsaBufData[3].buf = (char*)&triangle_indices_length;
    wsaBufData[3].len = sizeof(triangle_indices_length);

    ShowMessage("til %d", triangle_indices_length);

    wsaBufData[4].buf = (char*)triangle_indices_data;
    wsaBufData[4].len = triangle_indices_length;

    wsaBufData[5].buf = (char*)&vertex_normals_length;
    wsaBufData[5].len = sizeof(vertex_normals_length);

    ShowMessage("vnl %d", vertex_normals_length);

    wsaBufData[6].buf = (char*)vertex_normals_data;
    wsaBufData[6].len = vertex_normals_length;

    ok = send_multiple(clientsocket, wsaBufData, sizeof(wsaBufData) / sizeof(WSABUF));
    if (!ok) 
    {
        SM_TransferError();
        break;
    }
    }

    ShowMessage("SM: EndComputeMeshes");
    SpatialMapping_EndComputeMeshes();
    ShowMessage("SM: GetMeshes complete!");
}

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

static void SM_Translate(SOCKET clientsocket)
{
    g_clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    do { SM_Dispatch(clientsocket); } while (WaitForSingleObject(g_clientevent, 0) == WAIT_TIMEOUT);
    CloseHandle(g_clientevent);
}

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

void SM_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SM_EntryPoint, NULL, 0, NULL);
}

void SM_Quit()
{
    SetEvent(g_quitevent);
}

void SM_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);

    g_thread = NULL;
    g_quitevent = NULL;
}




// sizeof(VolumeDescription) = 100 = 4 + 96
// Box -> 6f -> 24 (same as sizeof)
// Frustum -> 6x4f -> 24f -> 96 (same as sizeof)
// OrientedBox -> (6+4)f -> 40 (same as sizeof)
// Sphere -> 4f -> 16 (same as sizeof)

// index  u32
// status u32

// vertex scale 3xf32
// positions size u32
// positions
// indices size u32
// indices
// normals size
// normals

//mesh.VertexNormals(); // optional!
