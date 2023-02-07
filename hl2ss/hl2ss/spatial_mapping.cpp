
#include <Windows.h>
#include "timestamps.h"
#include "locator.h"
#include "spatial_mapping.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Surfaces.h>
#include <winrt/Windows.Graphics.DirectX.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Surfaces;
using namespace winrt::Windows::Graphics::DirectX;

struct SSI_W // eww
{
    SpatialSurfaceMesh mesh = nullptr;
};


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_consent = NULL;
static HANDLE g_thread_consent = NULL;
static SpatialPerceptionAccessStatus g_status_consent = SpatialPerceptionAccessStatus::Unspecified;
static SpatialSurfaceObserver g_sso = nullptr;
static IMapView<winrt::guid, SpatialSurfaceInfo> g_observed = nullptr;
static std::vector<winrt::guid> g_observed_ids;
static std::vector<SSI_W> g_observed_meshes;
static std::vector<int32_t> g_observed_meshes_status;
static std::vector<HANDLE> g_observed_meshes_event; // CloseHandle
static std::vector<HANDLE> g_observed_meshes_thread; // CloseHandle
static HANDLE g_observed_meshes_semaphore; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

static DWORD WINAPI SpatialMapping_RequestAccess(void *param)
{
    (void)param;

    g_status_consent = SpatialSurfaceObserver::RequestAccessAsync().get();
    SetEvent(g_event_consent);

    return 0;
}

bool SpatialMapping_WaitForConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return g_status_consent == SpatialPerceptionAccessStatus::Allowed;
}

void SpatialMapping_Initialize()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread_consent = CreateThread(NULL, 0, SpatialMapping_RequestAccess, NULL, 0, NULL);
}

void SpatialMapping_CreateObserver()
{
    g_sso = SpatialSurfaceObserver();
}

void SpatialMapping_SetVolumes(std::vector<VolumeDescription> const& vd)
{
    SpatialCoordinateSystem scs = Locator_GetWorldCoordinateSystem(QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimestamp()));
    std::vector<SpatialBoundingVolume> volumes;
    
    for (int i = 0; i < vd.size(); ++i)
    {
    switch (vd[i].type)
    {
    case VolumeType_Box:         volumes.push_back(SpatialBoundingVolume::FromBox(        scs, vd[i].box));          break;
    case VolumeType_Frustum:     volumes.push_back(SpatialBoundingVolume::FromFrustum(    scs, vd[i].frustum));      break;
    case VolumeType_OrientedBox: volumes.push_back(SpatialBoundingVolume::FromOrientedBox(scs, vd[i].oriented_box)); break;
    case VolumeType_Sphere:      volumes.push_back(SpatialBoundingVolume::FromSphere(     scs, vd[i].sphere));       break;
    }
    }

    g_sso.SetBoundingVolumes(volumes);
}

void SpatialMapping_GetObservedSurfaces()
{
    g_observed = g_sso.GetObservedSurfaces();
    int i = 0;
    g_observed_ids.resize(g_observed.Size());
    for (auto pair : g_observed) { g_observed_ids[i++] = pair.Key(); }
}

void SpatialMapping_ReportIDs(winrt::guid const*& data, size_t& count)
{
    data = g_observed_ids.data();
    count = g_observed_ids.size();
}

static DWORD WINAPI SpatialMapping_ComputeMesh(void* param)
{
    SpatialSurfaceMeshOptions options = nullptr;
    MeshDescription* desc;

    WaitForSingleObject(g_observed_meshes_semaphore, INFINITE);
    
    desc = (MeshDescription*)param;

    if (g_observed.HasKey(desc->id))
    {
    options = SpatialSurfaceMeshOptions();
    options.IncludeVertexNormals(desc->normals);
    options.TriangleIndexFormat((DirectXPixelFormat)desc->triangle_format);
    options.VertexNormalFormat((DirectXPixelFormat)desc->normal_format);
    options.VertexPositionFormat((DirectXPixelFormat)desc->vertex_format);

    g_observed_meshes[desc->index].mesh = g_observed.Lookup(desc->id).TryComputeLatestMeshAsync(desc->maxtpcm, options).get();
    g_observed_meshes_status[desc->index] = 0;
    }
    else
    {
    g_observed_meshes_status[desc->index] = -1;
    }

    SetEvent(g_observed_meshes_event[desc->index]);
    ReleaseSemaphore(g_observed_meshes_semaphore, 1, NULL);
}

void SpatialMapping_BeginComputeMeshes(std::vector<MeshDescription>& desc, int maxtasks)
{
    size_t size = desc.size();

    g_observed_meshes.resize(size);
    g_observed_meshes_status.resize(size);
    g_observed_meshes_event.resize(size);

    g_observed_meshes_semaphore = CreateSemaphore(NULL, maxtasks, maxtasks, NULL);
    for (int i = 0; i < size; ++i) { g_observed_meshes_event[i] = CreateEvent(NULL, FALSE, FALSE, NULL); }
    for (int i = 0; i < size; ++i) { desc[i].index = i; g_observed_meshes_thread[i] = CreateThread(NULL, 0, SpatialMapping_ComputeMesh, (void*)&desc[i], 0, NULL); }
}

int SpatialMapping_WaitComputeMeshes()
{
    size_t size = g_observed_meshes.size();
    DWORD status = WaitForMultipleObjects((DWORD)size, g_observed_meshes_event.data(), FALSE, INFINITE);
    if ((status >= WAIT_OBJECT_0) && (status <= (WAIT_OBJECT_0 + size - 1))) { return status - WAIT_OBJECT_0; }
    return -1;
}

int SpatialMapping_GetStatusComputeMeshes(int index)
{
    return g_observed_meshes_status[index];
}

void SpatialMapping_GetMeshComputeMeshes(int index, SpatialSurfaceMesh &mesh)
{
    mesh = g_observed_meshes[index].mesh;
}

void SpatialMapping_EndComputeMeshes()
{
    size_t size = g_observed_meshes.size();

    WaitForMultipleObjects((DWORD)size, g_observed_meshes_thread.data(), TRUE, INFINITE);
    for (int i = 0; i < size; ++i) { CloseHandle(g_observed_meshes_thread[i]); }
    for (int i = 0; i < size; ++i) { CloseHandle(g_observed_meshes_event[i]); }
    CloseHandle(g_observed_meshes_semaphore);
}




// Box:         Center 3f, Extents 3f
// Frustum:     Near plane, Far plane, Right plane, Left plane, Top plane, Bottom plane | plane: normal 3f, d 1f
// OrientedBox: Center 3f, Extents 3f, Orientation quaternion | quaternion: x, y, z, w float
// Sphere:      Center 3f, Radius 1f


//mesh.TriangleIndices().Data().data(); // Data, ElementCount, Stride
    //mesh.VertexNormals();
    //mesh.VertexPositions();
    //mesh.VertexPositionScale();











// guid: u32 Data1, u16 Data2, u16 Data3, u8 Data4[8] -> 16 bytes



