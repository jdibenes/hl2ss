
#include "locator.h"
#include "timestamp.h"
#include "spatial_mapping.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Surfaces.h>
#include <winrt/Windows.Graphics.DirectX.h>
#include <winrt/Windows.Storage.Streams.h>

using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Surfaces;
using namespace winrt::Windows::Graphics::DirectX;
using namespace winrt::Windows::Storage::Streams;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static SpatialSurfaceObserver g_sso = nullptr;
static SpatialCoordinateSystem g_world = nullptr;
static IMapView<winrt::guid, SpatialSurfaceInfo> g_observed = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static SpatialBoundingOrientedBox SpatialMapping_GetBounds(SpatialSurfaceInfo const& ssi, SpatialSurfaceMesh const& ssm)
{
    auto sbob = ssi.TryGetBounds(ssm.CoordinateSystem());
    return sbob ? sbob.Value() : SpatialBoundingOrientedBox{ {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0} };
}

// OK
bool SpatialMapping_WaitForConsent()
{
    return SpatialSurfaceObserver::RequestAccessAsync().get() == SpatialPerceptionAccessStatus::Allowed;
}

// OK
void SpatialMapping_Open()
{
    g_sso = SpatialSurfaceObserver();
}

// OK
void SpatialMapping_Close()
{
    g_observed = nullptr;
    g_world = nullptr;
    g_sso = nullptr;
}

// OK
void SpatialMapping_SetVolumes(std::vector<SpatialMapping_VolumeDescription> const& vd)
{
    std::vector<SpatialBoundingVolume> volumes;

    g_world = Locator_GetWorldCoordinateSystem();
    
    for (int i = 0; i < static_cast<int>(vd.size()); ++i)
    {
    switch (vd[i].type)
    {
    case VolumeType_Box:         volumes.push_back(SpatialBoundingVolume::FromBox(        g_world, vd[i].data.box));          break;
    case VolumeType_Frustum:     volumes.push_back(SpatialBoundingVolume::FromFrustum(    g_world, vd[i].data.frustum));      break;
    case VolumeType_OrientedBox: volumes.push_back(SpatialBoundingVolume::FromOrientedBox(g_world, vd[i].data.oriented_box)); break;
    case VolumeType_Sphere:      volumes.push_back(SpatialBoundingVolume::FromSphere(     g_world, vd[i].data.sphere));       break;
    }
    }

    g_sso.SetBoundingVolumes(volumes);
}

// OK
void SpatialMapping_GetObservedSurfaces(std::vector<SpatialMapping_SurfaceInfo>& smsi)
{
    g_observed = g_sso.GetObservedSurfaces();
    int i = 0;
    smsi.resize(g_observed.Size());
    for (auto const& pair : g_observed) { smsi[i++] = { pair.Key(), pair.Value().UpdateTime().time_since_epoch().count() }; }
}

// OK
static void SpatialMapping_ComputeMesh(SpatialMapping_MeshDescription const& task, int index, HOOK_SM_PROC hook, void* param)
{
    SpatialSurfaceMeshOptions options = SpatialSurfaceMeshOptions();
    SpatialSurfaceInfo ssi = nullptr;
    SpatialSurfaceMesh ssm = nullptr;
    SpatialMapping_MeshInfo observed_mesh_info;
    IBuffer vertex_positions;
    IBuffer triangle_indices;
    IBuffer vertex_normals;

    memset(&observed_mesh_info, 0, sizeof(observed_mesh_info));

    observed_mesh_info.status = 1;
    observed_mesh_info.index  = index;

    ssi = g_observed.TryLookup(task.id);
    if (!ssi) { goto _push_smmi; }

    options.VertexPositionFormat(static_cast<DirectXPixelFormat>(task.vertex_format));
    options.TriangleIndexFormat(static_cast<DirectXPixelFormat>(task.triangle_format));
    options.VertexNormalFormat(static_cast<DirectXPixelFormat>(task.normal_format));
    options.IncludeVertexNormals(true);

    observed_mesh_info.status = 2;
    
    ssm = ssi.TryComputeLatestMeshAsync(task.maxtpcm, options).get();
    if (!ssm) { goto _push_smmi; }

    vertex_positions = ssm.VertexPositions().Data();
    triangle_indices = ssm.TriangleIndices().Data();
    vertex_normals   = ssm.VertexNormals().Data();

    observed_mesh_info.status = 0;
    observed_mesh_info.vpl    = vertex_positions.Length();
    observed_mesh_info.til    = triangle_indices.Length();
    observed_mesh_info.vnl    = vertex_normals.Length();
    observed_mesh_info.vpd    = vertex_positions.data();
    observed_mesh_info.tid    = triangle_indices.data();
    observed_mesh_info.vnd    = vertex_normals.data();
    observed_mesh_info.pose   = Locator_GetTransformTo(ssm.CoordinateSystem(), g_world);
    observed_mesh_info.scale  = ssm.VertexPositionScale();
    observed_mesh_info.bounds = SpatialMapping_GetBounds(ssi, ssm);

_push_smmi:
    hook(observed_mesh_info, param);
}

// OK
bool SpatialMapping_ExecuteSensorLoop(std::vector<SpatialMapping_MeshDescription> const& md, HOOK_SM_PROC hook, void* param, HANDLE event_stop)
{
    uint32_t index = 0;

    do
    {
    if (index >= md.size()) { return true; }
    SpatialMapping_ComputeMesh(md[index], index, hook, param);
    index++;
    }
    while (WaitForSingleObject(event_stop, 0) == WAIT_TIMEOUT);

    return false;
}
