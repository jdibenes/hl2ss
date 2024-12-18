
#include "spatial_mapping.h"
#include "server_channel.h"

#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Perception::Spatial;

class Channel_SM : public Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    bool Dispatch();

    bool MSG_SetVolumes();
    bool MSG_GetObservedSurfaces();
    bool MSG_GetMeshes();

    void OnFrameArrived(SpatialMapping_MeshInfo const& mesh);

    static void Thunk_Sensor(SpatialMapping_MeshInfo const& mesh, void* self);

public:
    Channel_SM(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_SM> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_SM::Thunk_Sensor(SpatialMapping_MeshInfo const& mesh, void* self)
{
    static_cast<Channel_SM*>(self)->OnFrameArrived(mesh);
}

// OK
void Channel_SM::OnFrameArrived(SpatialMapping_MeshInfo const& mesh)
{
    int const header_size = sizeof(SpatialMapping_MeshInfo) - (3 * sizeof(uint8_t*));

    WSABUF wsaBuf[4];

    pack_buffer(wsaBuf, 0, &mesh,     header_size);
    pack_buffer(wsaBuf, 1,  mesh.vpd, mesh.vpl);
    pack_buffer(wsaBuf, 2,  mesh.tid, mesh.til);
    pack_buffer(wsaBuf, 3,  mesh.vnd, mesh.vnl);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
bool Channel_SM::MSG_SetVolumes()
{
    std::vector<SpatialMapping_VolumeDescription> vd;
    uint32_t type;
    uint8_t count;
    int size;
    bool ok;

    ok = recv_u8(m_socket_client, m_event_client, count);
    if (!ok) { return false; }

    vd.resize(count);

    for (int i = 0; i < count; ++i)
    {
    ok = recv_u32(m_socket_client, m_event_client, type);
    if (!ok) { return false; }

    vd[i].type = static_cast<SpatialMapping_VolumeType>(type);

    switch (type)
    {
    case SpatialMapping_VolumeType::VolumeType_Box:         size = sizeof(SpatialBoundingBox);         break;
    case SpatialMapping_VolumeType::VolumeType_Frustum:     size = sizeof(SpatialBoundingFrustum);     break;
    case SpatialMapping_VolumeType::VolumeType_OrientedBox: size = sizeof(SpatialBoundingOrientedBox); break;
    case SpatialMapping_VolumeType::VolumeType_Sphere:      size = sizeof(SpatialBoundingSphere);      break;
    default:                                                                                           return false;
    }

    ok = recv(m_socket_client, m_event_client, &(vd[i].data), size);
    if (!ok) { return false; }
    }

    SpatialMapping_SetVolumes(vd);

    return true;
}

// OK
bool Channel_SM::MSG_GetObservedSurfaces()
{
    std::vector<SpatialMapping_SurfaceInfo> smsi;
    uint32_t count;
    WSABUF wsaBuf[2];
    bool ok;

    SpatialMapping_GetObservedSurfaces(smsi);

    count = static_cast<uint32_t>(smsi.size());

    pack_buffer(wsaBuf, 0, &count,      sizeof(count));
    pack_buffer(wsaBuf, 1, smsi.data(), sizeof(SpatialMapping_SurfaceInfo) * count);

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    return true;
}

// OK
bool Channel_SM::MSG_GetMeshes()
{
    std::vector<SpatialMapping_MeshDescription> md;
    uint32_t count;    
    bool ok;

    ok = recv_u32(m_socket_client, m_event_client, count);
    if (!ok) { return false; }

    md.resize(count);
    
    for (uint32_t i = 0; i < count; ++i)
    {
    ok = recv(m_socket_client, m_event_client, &(md[i]), sizeof(SpatialMapping_MeshDescription));
    if (!ok) { return false; }
    }

    return SpatialMapping_ExecuteSensorLoop(md, Thunk_Sensor, this, m_event_client);
}

// OK
bool Channel_SM::Dispatch()
{
    uint8_t state;
    bool ok;

    ok = recv_u8(m_socket_client, m_event_client, state);
    if (!ok) { return false; }

    switch (state)
    {
    case 0x00: return MSG_SetVolumes();
    case 0x01: return MSG_GetObservedSurfaces();
    case 0x02: return MSG_GetMeshes();
    default:   return false;
    }
}

// OK
Channel_SM::Channel_SM(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_SM::Startup()
{
    SetNoDelay(true);
    return SpatialMapping_WaitForConsent();
}

// OK
void Channel_SM::Run()
{
    SpatialMapping_Open();
    while (Dispatch());
    SpatialMapping_Close();
}

// OK
void Channel_SM::Cleanup()
{
}

// OK
void SM_Startup()
{
    g_channel = std::make_unique<Channel_SM>("SM", PORT_NAME_SM, PORT_ID_SM);
}

// OK
void SM_Cleanup()
{
}
