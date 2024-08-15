
#include <Windows.h>
#include "server.h"
#include "ports.h"
#include "scene_understanding.h"
#include "log.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace Microsoft::MixedReality::SceneUnderstanding;

struct Task
{
    SceneQuerySettings sqs;
    float query_radius;
    bool use_previous;
    uint8_t kind_flags;
    bool get_orientation;
    bool get_position;
    bool get_location_matrix;
    bool get_quad;
    bool get_meshes;
    bool get_collider_meshes;
    uint32_t guid_count;
};

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
static void SU_TransferError()
{
    SetEvent(g_event_client);
}

// OK
static void SU_TransferMeshes(SOCKET clientsocket, std::vector<std::shared_ptr<SceneMesh>> const& meshes)
{
    uint32_t const vertex_fields = 3;
    uint32_t count = (uint32_t)meshes.size();
    uint32_t count_vertices;
    uint32_t count_vertices_fields;
    uint32_t count_triangle_indices;
    uint32_t* data_vertices;
    uint32_t* data_triangle_indices;
    std::vector<uint32_t> buffer_vertices;
    std::vector<uint32_t> buffer_triangles;
    WSABUF wsaBufHeader[1];
    WSABUF wsaBufMesh[4];
    bool ok;

    pack_buffer(wsaBufHeader, 0, &count, sizeof(count));

    ok = send_multiple(clientsocket, wsaBufHeader, sizeof(wsaBufHeader) / sizeof(WSABUF));
    if (!ok)
    {
        SU_TransferError();
        return;
    }

    for (auto const& mesh : meshes)
    {
    count_vertices = mesh->GetVertexCount();
    count_triangle_indices = mesh->GetTriangleIndexCount();

    count_vertices_fields = count_vertices * vertex_fields;

    buffer_vertices.resize(count_vertices_fields);
    buffer_triangles.resize(count_triangle_indices);

    data_vertices = buffer_vertices.data();
    data_triangle_indices = buffer_triangles.data();

    mesh->GetVertexPositions(data_vertices, count_vertices);
    mesh->GetTriangleIndices({ data_triangle_indices, count_triangle_indices });

    pack_buffer(wsaBufMesh, 0, &count_vertices_fields, sizeof(count_vertices_fields));
    pack_buffer(wsaBufMesh, 1, &count_triangle_indices, sizeof(count_triangle_indices));
    pack_buffer(wsaBufMesh, 2, data_vertices, count_vertices_fields * sizeof(uint32_t));
    pack_buffer(wsaBufMesh, 3, data_triangle_indices, count_triangle_indices * sizeof(uint32_t));

    ok = send_multiple(clientsocket, wsaBufMesh, sizeof(wsaBufMesh) / sizeof(WSABUF));
    if (!ok)
    {
        SU_TransferError();
        return;
    }
    }
}

// OK
static void SU_Dispatch(SOCKET clientsocket)
{
    Task task;
    std::vector<GUID> guids;
    SceneUnderstanding_Result const* result;
    std::shared_ptr<SceneObject> item;
    std::shared_ptr<SceneQuad> quad;
    GUID item_id;
    SceneObjectKind item_kind;
    Quaternion item_orientation;
    Matrix4x4 item_location;
    Vector3 item_position;
    SceneQuadAlignment item_alignment;
    Vector2 item_extents;
    WSABUF wsaBuf_header[1];
    WSABUF wsaBuf_items[7];
    bool ok;

    ok = recv(clientsocket, (char*)&task, sizeof(Task));
    if (!ok)
    {
        SU_TransferError();
        return;
    }

    guids.resize(task.guid_count);
    ok = recv(clientsocket, (char*)guids.data(), (int)(guids.size() * sizeof(GUID)));
    if (!ok)
    {
        SU_TransferError();
        return;
    }

    SceneUnderstanding_Query(task.sqs, task.query_radius, task.use_previous, guids.data(), guids.size(), task.kind_flags);
    result = SceneUnderstanding_WaitForResult();

    pack_buffer(wsaBuf_header, 0, result, 136);

    ok = send_multiple(clientsocket, wsaBuf_header, sizeof(wsaBuf_header) / sizeof(WSABUF));
    if (!ok)
    {
        SU_TransferError();
        return;
    }

    pack_buffer(wsaBuf_items, 0, &item_id, sizeof(item_id));
    pack_buffer(wsaBuf_items, 1, &item_kind, sizeof(item_kind));
    pack_buffer(wsaBuf_items, 2, &item_orientation, sizeof(item_orientation) * task.get_orientation);
    pack_buffer(wsaBuf_items, 3, &item_position, sizeof(item_position) * task.get_position);
    pack_buffer(wsaBuf_items, 4, &item_location, sizeof(item_location) * task.get_location_matrix);
    pack_buffer(wsaBuf_items, 5, &item_alignment, sizeof(item_alignment) * task.get_quad);
    pack_buffer(wsaBuf_items, 6, &item_extents, sizeof(item_extents) * task.get_quad);

    for (int i = 0; i < result->selected.size(); ++i)
    {
    item = result->selected[i];

    item_id = item->GetId();
    item_kind = item->GetKind();

    if (task.get_orientation)     { item_orientation = item->GetOrientation(); }
    if (task.get_position)        { item_position = item->GetPosition(); }
    if (task.get_location_matrix) { item_location = item->GetLocationAsMatrix(); }
    if (task.get_quad)            { quad = item->GetQuad();
                                    item_alignment = quad ? quad->GetAlignment() : SceneQuadAlignment::NonOrthogonal;
                                    item_extents = quad ? quad->GetExtents() : Vector2{0, 0}; }
    
    ok = send_multiple(clientsocket, wsaBuf_items, sizeof(wsaBuf_items) / sizeof(WSABUF));
    if (!ok)
    {
        SU_TransferError();
        return;
    }
    
    if (task.get_meshes)          { SU_TransferMeshes(clientsocket, item->GetMeshes()); }
    if (task.get_collider_meshes) { SU_TransferMeshes(clientsocket, item->GetColliderMeshes()); }
    }
}

// OK
static void SU_Translate(SOCKET clientsocket)
{
    ResetEvent(g_event_client);
    do { SU_Dispatch(clientsocket); } while (WaitForSingleObject(g_event_client, 0) == WAIT_TIMEOUT);
}

// OK
static DWORD WINAPI SU_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    ShowMessage("SU: Waiting for consent");

    SceneUnderstanding_WaitForConsent();

    listensocket = CreateSocket(PORT_NAME_SU);

    ShowMessage("SU: Listening at port %s", PORT_NAME_SU);

    do
    {
    ShowMessage("SU: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("SU: Client connected");

    SU_Translate(clientsocket);

    closesocket(clientsocket);

    ShowMessage("SU: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("SU: Closed");

    return 0;
}

// OK
void SU_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, SU_EntryPoint, NULL, 0, NULL);
}

// OK
void SU_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void SU_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);
}
