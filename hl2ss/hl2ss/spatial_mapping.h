
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

enum SpatialMapping_VolumeType
{
    VolumeType_Box,
    VolumeType_Frustum,
    VolumeType_OrientedBox,
    VolumeType_Sphere
};

struct SpatialMapping_VolumeDescription
{
    SpatialMapping_VolumeType type;
    union
    {
    winrt::Windows::Perception::Spatial::SpatialBoundingBox box;
    winrt::Windows::Perception::Spatial::SpatialBoundingFrustum frustum;
    winrt::Windows::Perception::Spatial::SpatialBoundingOrientedBox oriented_box;
    winrt::Windows::Perception::Spatial::SpatialBoundingSphere sphere;
    }
    data;
};

struct SpatialMapping_SurfaceInfo
{
    winrt::guid id;
    int64_t update_time;
};

#define SPATIALMAPPING_COMPUTE_NORMALS 0x01
#define SPATIALMAPPING_COMPUTE_BOUNDS  0x02

struct SpatialMapping_MeshDescription
{
    winrt::guid id;
    double maxtpcm;
    uint32_t vertex_format;
    uint32_t triangle_format;
    uint32_t normal_format;
    uint32_t flags;
};

struct SpatialMapping_MeshTask
{
    SpatialMapping_MeshDescription md;
    uint32_t index;
    uint32_t reserved;
};

struct SpatialMapping_MeshInfo
{
    uint32_t index;
    uint32_t status;
    uint32_t vpl;
    uint32_t til;
    uint32_t vnl;
    winrt::Windows::Foundation::Numerics::float3 scale;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
    uint32_t bsz;
    winrt::Windows::Perception::Spatial::SpatialBoundingOrientedBox bounds;
    uint8_t* vpd;
    uint8_t* tid;
    uint8_t* vnd;
    void* ssm;
};

int const SM_MESH_INFO_HEADER_SIZE = 100;

void SpatialMapping_Initialize();
bool SpatialMapping_WaitForConsent();
void SpatialMapping_CreateObserver();
void SpatialMapping_SetVolumes(SpatialMapping_VolumeDescription const* vd, size_t size);
void SpatialMapping_GetObservedSurfaces(SpatialMapping_SurfaceInfo const*& data, size_t& size);
void SpatialMapping_BeginComputeMeshes(SpatialMapping_MeshTask* task, size_t size, int maxtasks);
SpatialMapping_MeshInfo* SpatialMapping_GetNextMesh();
void SpatialMapping_DestroyMesh(SpatialMapping_MeshInfo* mi);
void SpatialMapping_EndComputeMeshes();
