
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
    }
    data;
};

struct MeshDescription
{
    winrt::guid id;  // 16
    double      maxtpcm; // 8 -> 24
    uint32_t    triangle_format; // 4 -> 28
    uint32_t    normal_format; // 4 -> 32
    uint32_t    vertex_format;  // 4 -> 36
    bool        normals; // 4 -> 40
};

struct MeshTask
{
    MeshDescription md;
    uint32_t        index; // 4 -> 44
    uint32_t        reserved; // 4 -> 48
};

void SpatialMapping_Initialize();
bool SpatialMapping_WaitForConsent();
void SpatialMapping_CreateObserver();
void SpatialMapping_SetVolumes(std::vector<VolumeDescription> const& vd);
void SpatialMapping_GetObservedSurfaces();
void SpatialMapping_ReportIDs(winrt::guid const*& data, uint32_t& count);
void SpatialMapping_BeginComputeMeshes(std::vector<MeshTask>& desc, int maxtasks);
int SpatialMapping_WaitComputeMeshes();
int SpatialMapping_GetStatusComputeMeshes(int index);
void SpatialMapping_GetMeshComputeMeshes(int index, winrt::Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh& mesh);
void SpatialMapping_EndComputeMeshes();
