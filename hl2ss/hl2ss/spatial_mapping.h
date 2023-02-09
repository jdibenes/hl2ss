
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

enum VolumeType
{
    VolumeType_Box,
    VolumeType_Frustum,
    VolumeType_OrientedBox,
    VolumeType_Sphere
};

struct VolumeDescription
{
    VolumeType type;
    union
    {
    winrt::Windows::Perception::Spatial::SpatialBoundingBox box;
    winrt::Windows::Perception::Spatial::SpatialBoundingFrustum frustum;
    winrt::Windows::Perception::Spatial::SpatialBoundingOrientedBox oriented_box;
    winrt::Windows::Perception::Spatial::SpatialBoundingSphere sphere;
    }
    data;
};

struct MeshDescription
{
    winrt::guid id;
    double maxtpcm;
    uint32_t vertex_format;
    uint32_t triangle_format;
    uint32_t normal_format;
    bool normals;
};

struct MeshTask
{
    MeshDescription md;
    uint32_t index;
    uint32_t reserved;
};

struct MeshInfo
{
    uint32_t index;
    uint32_t status;
    uint32_t vpl;
    uint32_t til;
    uint32_t vnl;
    winrt::Windows::Foundation::Numerics::float3 scale;
    int64_t update_time;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
    winrt::Windows::Perception::Spatial::SpatialBoundingOrientedBox bounds;
    uint8_t* vpd;
    uint8_t* tid;
    uint8_t* vnd;
};

int const MESH_INFO_HEADER_SIZE = 144;

void SpatialMapping_Initialize();
bool SpatialMapping_WaitForConsent();
void SpatialMapping_CreateObserver();
void SpatialMapping_SetVolumes(VolumeDescription const* vd, size_t size);
void SpatialMapping_GetObservedSurfaces(winrt::guid const*& data, size_t& size);
void SpatialMapping_BeginComputeMeshes(MeshTask* task, size_t size, int maxtasks);
MeshInfo* SpatialMapping_GetNextMesh();
void SpatialMapping_EndComputeMeshes();
