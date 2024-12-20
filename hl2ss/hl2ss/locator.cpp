
#include "lock.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static SRWLOCK g_lock;
static SpatialCoordinateSystem g_world_override = nullptr;
static SpatialLocator g_locator = nullptr;
static SpatialStationaryFrameOfReference g_referenceFrame = nullptr;
static SpatialCoordinateSystem g_world = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Locator_Initialize()
{
    InitializeSRWLock(&g_lock);

    g_locator = SpatialLocator::GetDefault();
    g_referenceFrame = g_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
    g_world = g_referenceFrame.CoordinateSystem();
}

// OK
float4x4 Locator_Locate(PerceptionTimestamp const& timestamp, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    auto location = locator.TryLocateAtTimestamp(timestamp, world);
    return location ? (make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position())) : float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// OK
float4x4 Locator_GetTransformTo(SpatialCoordinateSystem const& src, SpatialCoordinateSystem const& dst)
{
    auto location = src.TryGetTransformTo(dst);
    return location ? location.Value() : float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// OK
SpatialCoordinateSystem Locator_GetWorldCoordinateSystem()
{
    SRWLock srw(&g_lock, false);
    return g_world_override ? g_world_override : g_world;
}

// OK
void Locator_OverrideWorldCoordinateSystem(SpatialCoordinateSystem const &scs)
{
    SRWLock srw(&g_lock, true);
    g_world_override = scs;
}

// OK
SpatialCoordinateSystem Locator_SanitizeSpatialCoordinateSystem(SpatialCoordinateSystem const& scs)
{
    // Workaround for SM GetObservedSurfaces crash (sanitizes scs somehow)
    // GetObservedSurfaces crashes when using OpenXR coordinate system directly (Windows.Mirage.dll)
    SpatialGraphInteropFrameOfReferencePreview sgiforp = SpatialGraphInteropPreview::TryCreateFrameOfReference(scs);
    return sgiforp ? sgiforp.CoordinateSystem() : nullptr;
}
