
#include <Windows.h>
#include "timestamps.h"
#include "locator.h"
#include "spatial_mapping.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Surfaces.h>
#include <winrt/Windows.Graphics.DirectX.h>
#include <winrt/Windows.Storage.Streams.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Surfaces;
using namespace winrt::Windows::Graphics::DirectX;
using namespace winrt::Windows::Storage::Streams;

struct SSM
{
    SpatialSurfaceMesh mesh = nullptr;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_consent = NULL; // CloseHandle
static HANDLE g_thread_consent = NULL; // CloseHandle
static SpatialPerceptionAccessStatus g_status_consent = SpatialPerceptionAccessStatus::Unspecified;
static SpatialSurfaceObserver g_sso = nullptr;
static SpatialCoordinateSystem g_world = nullptr;
static IMapView<winrt::guid, SpatialSurfaceInfo> g_observed = nullptr;
static std::vector<winrt::guid> g_observed_ids;
static std::vector<SSM> g_observed_meshes;
static std::vector<MeshInfo> g_observed_meshes_info;
static std::vector<HANDLE> g_observed_meshes_event; // CloseHandle
static std::vector<HANDLE> g_observed_meshes_thread; // CloseHandle
static HANDLE g_observed_meshes_semaphore; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static DWORD WINAPI SpatialMapping_RequestAccess(void *param)
{
    (void)param;

    g_status_consent = SpatialSurfaceObserver::RequestAccessAsync().get();
    SetEvent(g_event_consent);

    return 0;
}

// OK
void SpatialMapping_Initialize()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread_consent = CreateThread(NULL, 0, SpatialMapping_RequestAccess, NULL, 0, NULL);
}

// OK
bool SpatialMapping_WaitForConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return g_status_consent == SpatialPerceptionAccessStatus::Allowed;
}

// OK
void SpatialMapping_CreateObserver()
{
    g_sso = SpatialSurfaceObserver();
}

// OK
void SpatialMapping_SetVolumes(VolumeDescription const* vd, size_t size)
{
    std::vector<SpatialBoundingVolume> volumes;

    g_world = Locator_GetWorldCoordinateSystem(QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimestamp()));
    
    for (int i = 0; i < size; ++i)
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
void SpatialMapping_GetObservedSurfaces(winrt::guid const*& data, size_t& size)
{
    g_observed = g_sso.GetObservedSurfaces();
    int i = 0;
    g_observed_ids.resize(g_observed.Size());
    for (auto pair : g_observed) { g_observed_ids[i++] = pair.Key(); }
    data = g_observed_ids.data();
    size = g_observed_ids.size();
}

// OK
static SpatialBoundingOrientedBox SpatialMapping_GetBounds(SpatialSurfaceInfo ssi, SpatialSurfaceMesh ssm)
{
    auto sbob = ssi.TryGetBounds(ssm.CoordinateSystem());
    return sbob ? sbob.Value() : SpatialBoundingOrientedBox{ {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0} };
}

// OK
static void SpatialMapping_ComputeMesh(MeshTask* task)
{
    SpatialSurfaceInfo ssi = nullptr;
    SpatialSurfaceMeshOptions options = nullptr;
    SpatialSurfaceMesh ssm = nullptr;
    IBuffer vertex_positions;
    IBuffer triangle_indices;
    IBuffer vertex_normals;

    g_observed_meshes_info[task->index].index = task->index;

    ssi = g_observed.TryLookup(task->md.id);
    if (!ssi)
    {
        g_observed_meshes_info[task->index].status = 1;
        return;
    }

    options = SpatialSurfaceMeshOptions();
    options.VertexPositionFormat((DirectXPixelFormat)task->md.vertex_format);
    options.TriangleIndexFormat((DirectXPixelFormat)task->md.triangle_format);
    options.VertexNormalFormat((DirectXPixelFormat)task->md.normal_format);
    options.IncludeVertexNormals(task->md.normals);

    ssm = ssi.TryComputeLatestMeshAsync(task->md.maxtpcm, options).get();
    if (!ssm)
    {
        g_observed_meshes_info[task->index].status = 2;
        return;
    }

    g_observed_meshes[task->index].mesh = ssm;

    vertex_positions = ssm.VertexPositions().Data();
    triangle_indices = ssm.TriangleIndices().Data();
    vertex_normals   = task->md.normals ? ssm.VertexNormals().Data() : nullptr;

    g_observed_meshes_info[task->index].status      = 0;
    g_observed_meshes_info[task->index].vpl         = vertex_positions.Length();
    g_observed_meshes_info[task->index].til         = triangle_indices.Length();
    g_observed_meshes_info[task->index].vnl         = task->md.normals ? vertex_normals.Length() : 0;
    g_observed_meshes_info[task->index].vpd         = vertex_positions.data();
    g_observed_meshes_info[task->index].tid         = triangle_indices.data();
    g_observed_meshes_info[task->index].vnd         = task->md.normals ? vertex_normals.data() : NULL;
    g_observed_meshes_info[task->index].update_time = ssi.UpdateTime().time_since_epoch().count();
    g_observed_meshes_info[task->index].pose        = Locator_GetTransformTo(ssm.CoordinateSystem(), g_world);
    g_observed_meshes_info[task->index].scale       = ssm.VertexPositionScale();
    g_observed_meshes_info[task->index].bounds      = SpatialMapping_GetBounds(ssi, ssm);
}

// OK
static DWORD WINAPI SpatialMapping_ComputeMesh(void* param)
{
    MeshTask* task = (MeshTask*)param;
    WaitForSingleObject(g_observed_meshes_semaphore, INFINITE);
    SpatialMapping_ComputeMesh(task);
    SetEvent(g_observed_meshes_event[task->index]);
    ReleaseSemaphore(g_observed_meshes_semaphore, 1, NULL);
    return 0;
}

// OK
void SpatialMapping_BeginComputeMeshes(MeshTask* task, size_t size, int maxtasks)
{
    g_observed_meshes.resize(size);
    g_observed_meshes_info.resize(size);
    g_observed_meshes_event.resize(size);
    g_observed_meshes_thread.resize(size);

    g_observed_meshes_semaphore = CreateSemaphore(NULL, maxtasks, maxtasks, NULL);

    for (int i = 0; i < size; ++i) { task[i].index = i; }
    for (int i = 0; i < size; ++i) { g_observed_meshes_event[i] = CreateEvent(NULL, FALSE, FALSE, NULL); }
    for (int i = 0; i < size; ++i) { g_observed_meshes_thread[i] = CreateThread(NULL, 0, SpatialMapping_ComputeMesh, (void*)(task + i), 0, NULL); }
}

// OK
MeshInfo* SpatialMapping_GetNextMesh()
{
    size_t size = g_observed_meshes_event.size();
    DWORD status = WaitForMultipleObjects((DWORD)size, g_observed_meshes_event.data(), FALSE, INFINITE);
    return ((status >= WAIT_OBJECT_0) && (status < (WAIT_OBJECT_0 + size))) ? (g_observed_meshes_info.data() + (status - WAIT_OBJECT_0)) : NULL;
}

// OK
void SpatialMapping_EndComputeMeshes()
{
    size_t size = g_observed_meshes.size();

    WaitForMultipleObjects((DWORD)size, g_observed_meshes_thread.data(), TRUE, INFINITE);

    for (int i = 0; i < size; ++i) { CloseHandle(g_observed_meshes_thread[i]); }
    for (int i = 0; i < size; ++i) { CloseHandle(g_observed_meshes_event[i]); }
    CloseHandle(g_observed_meshes_semaphore);
}
