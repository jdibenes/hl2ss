
#include "scene_understanding.h"
#include "server_channel.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace Microsoft::MixedReality::SceneUnderstanding;

struct SU_Task
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

class Channel_SU : public Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    bool Dispatch();

    bool TX_Item_Identity(GUID const& item_id, SceneObjectKind item_kind);
    bool TX_Item_Orientation(Quaternion const& item_orientation);
    bool TX_Item_Position(Vector3 const& item_position);
    bool TX_Item_Location(Matrix4x4 const& item_location);
    bool TX_Item_Quad(std::shared_ptr<SceneQuad> quad);
    bool TX_Item_Meshes(std::vector<std::shared_ptr<SceneMesh>> const& meshes);
    bool TX_Item_Mesh(std::shared_ptr<SceneMesh> mesh);

public:
    Channel_SU(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_SU> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool Channel_SU::TX_Item_Identity(GUID const& item_id, SceneObjectKind item_kind)
{
    WSABUF wsaBuf[2];
    pack_buffer(wsaBuf, 0, &item_id,   sizeof(item_id));
    pack_buffer(wsaBuf, 1, &item_kind, sizeof(item_kind));
    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::TX_Item_Orientation(Quaternion const& item_orientation)
{
    WSABUF wsaBuf[1];
    pack_buffer(wsaBuf, 0, &item_orientation, sizeof(item_orientation));
    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::TX_Item_Position(Vector3 const& item_position)
{
    WSABUF wsaBuf[1];
    pack_buffer(wsaBuf, 0, &item_position, sizeof(item_position));
    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::TX_Item_Location(Matrix4x4 const& item_location)
{
    WSABUF wsaBuf[1];
    pack_buffer(wsaBuf, 0, &item_location, sizeof(item_location));
    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::TX_Item_Quad(std::shared_ptr<SceneQuad> quad)
{
    SceneQuadAlignment item_alignment = quad ? quad->GetAlignment() : SceneQuadAlignment::NonOrthogonal;
    Vector2            item_extents   = quad ? quad->GetExtents()   : Vector2{ 0, 0 };
    WSABUF wsaBuf[2];
    pack_buffer(wsaBuf, 0, &item_alignment, sizeof(item_alignment));
    pack_buffer(wsaBuf, 1, &item_extents,   sizeof(item_extents));
    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::TX_Item_Meshes(std::vector<std::shared_ptr<SceneMesh>> const& meshes)
{
    uint32_t count = static_cast<uint32_t>(meshes.size());
    WSABUF wsaBuf[1];
    bool ok;

    pack_buffer(wsaBuf, 0, &count, sizeof(count));

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    for (auto const& mesh : meshes) 
    {
    ok = TX_Item_Mesh(mesh);
    if (!ok) { return false; }
    }

    return true;
}

// OK
bool Channel_SU::TX_Item_Mesh(std::shared_ptr<SceneMesh> mesh)
{
    uint32_t const vertex_fields = 3;

    uint32_t count_vertices;
    uint32_t count_vertices_fields;
    uint32_t count_triangle_indices;
    uint32_t* data_vertices;
    uint32_t* data_triangle_indices;
    std::vector<uint32_t> buffer_vertices;
    std::vector<uint32_t> buffer_triangles;    
    WSABUF wsaBuf[4];

    count_vertices         = mesh->GetVertexCount();
    count_triangle_indices = mesh->GetTriangleIndexCount();
    count_vertices_fields  = count_vertices * vertex_fields;

    buffer_vertices.resize(count_vertices_fields);
    buffer_triangles.resize(count_triangle_indices);

    data_vertices         = buffer_vertices.data();
    data_triangle_indices = buffer_triangles.data();

    mesh->GetVertexPositions(  data_vertices,         count_vertices);
    mesh->GetTriangleIndices({ data_triangle_indices, count_triangle_indices });

    pack_buffer(wsaBuf, 0, &count_vertices_fields,  sizeof(count_vertices_fields));
    pack_buffer(wsaBuf, 1, &count_triangle_indices, sizeof(count_triangle_indices));
    pack_buffer(wsaBuf, 2, data_vertices,           sizeof(uint32_t) * count_vertices_fields);
    pack_buffer(wsaBuf, 3, data_triangle_indices,   sizeof(uint32_t) * count_triangle_indices);

    return send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SU::Dispatch()
{
    int const header_size = sizeof(SceneUnderstanding_Result::status) + sizeof(SceneUnderstanding_Result::extrinsics) + sizeof(SceneUnderstanding_Result::pose) + sizeof(SceneUnderstanding_Result::items);
    
    SU_Task task;
    std::vector<GUID> guids;
    SceneUnderstanding_Result const* result;
    WSABUF wsaBuf[1];
    bool ok;

    ok = recv(m_socket_client, m_event_client, &task, sizeof(SU_Task));
    if (!ok) { return false; }

    guids.resize(task.guid_count);

    ok = recv(m_socket_client, m_event_client, guids.data(), static_cast<int>(guids.size() * sizeof(GUID)));
    if (!ok) { return false; }

    SceneUnderstanding_Query(task.sqs, task.query_radius, task.use_previous, guids.data(), guids.size(), task.kind_flags);
    result = SceneUnderstanding_WaitForResult();

    pack_buffer(wsaBuf, 0, result, header_size);

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    for (auto const& item : result->selected)
    {
    if (                            !TX_Item_Identity(item->GetId(), item->GetKind())) { return false; }
    if (task.get_orientation     && !TX_Item_Orientation(item->GetOrientation()))      { return false; }
    if (task.get_position        && !TX_Item_Position(item->GetPosition()))            { return false; }
    if (task.get_location_matrix && !TX_Item_Location(item->GetLocationAsMatrix()))    { return false; }
    if (task.get_quad            && !TX_Item_Quad(item->GetQuad()))                    { return false; }
    if (task.get_meshes          && !TX_Item_Meshes(item->GetMeshes()))                { return false; }
    if (task.get_collider_meshes && !TX_Item_Meshes(item->GetColliderMeshes()))        { return false; }
    }

    return true;
}

// OK
Channel_SU::Channel_SU(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_SU::Startup()
{
    SetNoDelay(true);
    return SceneUnderstanding_WaitForConsent();
}

// OK
void Channel_SU::Run()
{
    while (Dispatch());
}

// OK
void Channel_SU::Cleanup()
{
}

// OK
void SU_Startup()
{
    g_channel = std::make_unique<Channel_SU>("SU", PORT_NAME_SU, PORT_ID_SU);
}

// OK
void SU_Cleanup()
{
}
