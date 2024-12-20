
#pragma once

#include <Windows.h>

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

struct SpatialMapping_MeshDescription
{
    winrt::guid id;
    double maxtpcm;
    uint32_t vertex_format;
    uint32_t triangle_format;
    uint32_t normal_format;
    uint32_t flags;
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
    winrt::Windows::Perception::Spatial::SpatialBoundingOrientedBox bounds;
    uint8_t* vpd;
    uint8_t* tid;
    uint8_t* vnd;
};

typedef void (*HOOK_SM_PROC)(SpatialMapping_MeshInfo const&, void*);

bool SpatialMapping_WaitForConsent();
void SpatialMapping_Open();
void SpatialMapping_Close();
void SpatialMapping_SetVolumes(std::vector<SpatialMapping_VolumeDescription> const& vd);
void SpatialMapping_GetObservedSurfaces(std::vector<SpatialMapping_SurfaceInfo>& smsi);
bool SpatialMapping_ExecuteSensorLoop(std::vector<SpatialMapping_MeshDescription> const& md, HOOK_SM_PROC hook, void* param, HANDLE event_stop);
