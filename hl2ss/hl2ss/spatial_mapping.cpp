
#include <Windows.h>
#include <queue>
#include "timestamps.h"
#include "locator.h"
#include "spatial_mapping.h"
#include "lock.h"

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

struct SpatialMapping_MeshTaskGroup
{
    SpatialMapping_MeshTask* task;
    size_t size;
    int maxtasks;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread_consent = NULL; // CloseHandle
static SpatialPerceptionAccessStatus g_status_consent = SpatialPerceptionAccessStatus::Unspecified;
static CRITICAL_SECTION g_lock; // DeleteCriticalSection

static SpatialSurfaceObserver g_sso = nullptr;
static SpatialCoordinateSystem g_world = nullptr;
static IMapView<winrt::guid, SpatialSurfaceInfo> g_observed = nullptr;
static std::vector<SpatialMapping_SurfaceInfo> g_observed_ids;

static HANDLE g_compute_thread = NULL; // CloseHandle
static HANDLE g_semaphore_observed_meshes = NULL; // CloseHandle
static std::queue<SpatialMapping_MeshInfo*> g_observed_meshes_info;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static DWORD WINAPI SpatialMapping_RequestAccess(void *param)
{
    (void)param;
    g_status_consent = SpatialSurfaceObserver::RequestAccessAsync().get();
    return 0;
}

// OK
void SpatialMapping_Initialize()
{
    InitializeCriticalSection(&g_lock);
    g_thread_consent = CreateThread(NULL, 0, SpatialMapping_RequestAccess, NULL, 0, NULL);
}

// OK
bool SpatialMapping_WaitForConsent()
{
    WaitForSingleObject(g_thread_consent, INFINITE);
    CloseHandle(g_thread_consent);
    return g_status_consent == SpatialPerceptionAccessStatus::Allowed;
}

// OK
void SpatialMapping_CreateObserver()
{
    g_sso = SpatialSurfaceObserver();
}

// OK
void SpatialMapping_SetVolumes(SpatialMapping_VolumeDescription const* vd, size_t size)
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
void SpatialMapping_GetObservedSurfaces(SpatialMapping_SurfaceInfo const*& data, size_t& size)
{
    g_observed = g_sso.GetObservedSurfaces();
    int i = 0;
    g_observed_ids.resize(g_observed.Size());
    for (auto const& pair : g_observed) { g_observed_ids[i++] = { pair.Key(), pair.Value().UpdateTime().time_since_epoch().count() }; }
    data = g_observed_ids.data();
    size = g_observed_ids.size();
}

// OK
static SpatialBoundingOrientedBox SpatialMapping_GetBounds(SpatialSurfaceInfo ssi, SpatialSurfaceMesh ssm)
{
    auto const& sbob = ssi.TryGetBounds(ssm.CoordinateSystem());
    return sbob ? sbob.Value() : SpatialBoundingOrientedBox{ {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0} };
}

// OK
static DWORD WINAPI SpatialMapping_ComputeMesh(void* param)
{
    SpatialMapping_MeshTask* task = (SpatialMapping_MeshTask*)param;
    SpatialMapping_MeshInfo* observed_meshes_info = new SpatialMapping_MeshInfo; // delete
    SpatialSurfaceInfo ssi = nullptr;
    SpatialSurfaceMeshOptions options = nullptr;
    SpatialSurfaceMesh ssm = nullptr;
    bool compute_normals = (task->md.flags & SPATIALMAPPING_COMPUTE_NORMALS) != 0;
    bool compute_bounds  = (task->md.flags & SPATIALMAPPING_COMPUTE_BOUNDS)  != 0;
    IBuffer vertex_positions;
    IBuffer triangle_indices;
    IBuffer vertex_normals;    
    
    memset(observed_meshes_info, 0, sizeof(observed_meshes_info));

    observed_meshes_info->index = task->index;

    ssi = g_observed.TryLookup(task->md.id);
    if (!ssi)
    {
        observed_meshes_info->status = 1;
        goto _push_mesh_info;
    }

    options = SpatialSurfaceMeshOptions();
    options.VertexPositionFormat((DirectXPixelFormat)task->md.vertex_format);
    options.TriangleIndexFormat((DirectXPixelFormat)task->md.triangle_format);
    options.VertexNormalFormat((DirectXPixelFormat)task->md.normal_format);
    options.IncludeVertexNormals(compute_normals);

    ssm = ssi.TryComputeLatestMeshAsync(task->md.maxtpcm, options).get();
    if (!ssm)
    {
        observed_meshes_info->status = 2;
        goto _push_mesh_info;
    }

    vertex_positions = ssm.VertexPositions().Data();
    triangle_indices = ssm.TriangleIndices().Data();
    vertex_normals   = compute_normals ? ssm.VertexNormals().Data() : nullptr;

    observed_meshes_info->status = 0;
    observed_meshes_info->vpl    = vertex_positions.Length();
    observed_meshes_info->til    = triangle_indices.Length();
    observed_meshes_info->vnl    = compute_normals ? vertex_normals.Length() : 0;
    observed_meshes_info->vpd    = vertex_positions.data();
    observed_meshes_info->tid    = triangle_indices.data();
    observed_meshes_info->vnd    = compute_normals ? vertex_normals.data() : NULL;
    observed_meshes_info->pose   = Locator_GetTransformTo(ssm.CoordinateSystem(), g_world);
    observed_meshes_info->scale  = ssm.VertexPositionScale();
    observed_meshes_info->bsz    = compute_bounds ? sizeof(SpatialBoundingOrientedBox) : 0;
    if (compute_bounds)
    {
    observed_meshes_info->bounds = SpatialMapping_GetBounds(ssi, ssm);
    }
    observed_meshes_info->ssm    = winrt::detach_abi(ssm); // attach_abi

_push_mesh_info:
    {
    CriticalSection cs(&g_lock);
    g_observed_meshes_info.push(observed_meshes_info);
    }

    ReleaseSemaphore(g_semaphore_observed_meshes, 1, NULL);

    return 0;
}

// OK
static DWORD WINAPI SpatialMapping_ComputeMeshes(void* param)
{
    SpatialMapping_MeshTaskGroup* mtg = (SpatialMapping_MeshTaskGroup*)param; // delete
    SpatialMapping_MeshTask* task = mtg->task;
    size_t size = mtg->size;
    int maxtasks = (size < mtg->maxtasks) ? (int)size : mtg->maxtasks;
    std::vector<HANDLE> threads; // CloseHandle
    int i;
    DWORD status;
    int s;

    threads.resize(maxtasks);

    for (i = 0; i < size;     ++i)
    {
    task[i].index = i;
    }
    for (i = 0; i < maxtasks; ++i) 
    { 
    threads[i] = CreateThread(NULL, 0, SpatialMapping_ComputeMesh, (void*)(task + i), 0, NULL);
    }
    for (;      i < size;     ++i)
    {
    status = WaitForMultipleObjects(maxtasks, threads.data(), FALSE, INFINITE);
    s = status - WAIT_OBJECT_0;
    CloseHandle(threads[s]);
    threads[s] = CreateThread(NULL, 0, SpatialMapping_ComputeMesh, (void*)(task + i), 0, NULL);
    }
    status = WaitForMultipleObjects(maxtasks, threads.data(), TRUE,  INFINITE);
    for (i = 0; i < maxtasks; ++i) 
    {
    CloseHandle(threads[i]);
    }

    delete mtg;

    return 0;
}

// OK
void SpatialMapping_BeginComputeMeshes(SpatialMapping_MeshTask* task, size_t size, int maxtasks)
{
    void* param = (void*)(new SpatialMapping_MeshTaskGroup{ task, size, maxtasks }); // delete

    g_semaphore_observed_meshes = CreateSemaphore(NULL, 0, 0x7FFFFFFF, NULL);
    g_compute_thread = CreateThread(NULL, 0, SpatialMapping_ComputeMeshes, param, 0, NULL);
}

// OK
SpatialMapping_MeshInfo* SpatialMapping_GetNextMesh()
{
    SpatialMapping_MeshInfo* mi;

    WaitForSingleObject(g_semaphore_observed_meshes, INFINITE);

    {
    CriticalSection cs(&g_lock);
    mi = g_observed_meshes_info.front();
    g_observed_meshes_info.pop();
    }

    return mi;
}

// OK
void SpatialMapping_DestroyMesh(SpatialMapping_MeshInfo* mi)
{
    if (mi->ssm) { SpatialSurfaceMesh ssm{ mi->ssm, winrt::take_ownership_from_abi }; }
    delete mi;
}

// OK
void SpatialMapping_EndComputeMeshes()
{
    WaitForSingleObject(g_compute_thread, INFINITE);
    CloseHandle(g_compute_thread);
    CloseHandle(g_semaphore_observed_meshes);
}
