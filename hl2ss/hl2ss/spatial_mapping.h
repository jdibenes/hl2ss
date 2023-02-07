
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Surfaces.h>

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
    };
};

struct MeshDescription
{
    winrt::guid id;
    double      maxtpcm;
    uint32_t    index;
    uint32_t    triangle_format;
    uint32_t    normal_format;
    uint32_t    vertex_format;
    bool        normals;
};

bool SpatialMapping_WaitForConsent();
void SpatialMapping_Initialize();
void SpatialMapping_CreateObserver();
void SpatialMapping_SetVolumes(std::vector<VolumeDescription> const& vd);
void SpatialMapping_GetObservedSurfaces();
void SpatialMapping_ReportIDs(winrt::guid const*& data, size_t& count);
void SpatialMapping_BeginComputeMeshes(std::vector<MeshDescription> const& desc, int maxtasks);
int SpatialMapping_WaitComputeMeshes();
int SpatialMapping_GetStatusComputeMeshes(int index);
void SpatialMapping_GetMeshComputeMeshes(int index, winrt::Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh& mesh);
void SpatialMapping_EndComputeMeshes();
