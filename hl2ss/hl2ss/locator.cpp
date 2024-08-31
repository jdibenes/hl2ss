
#include "lock.h"

#include <winrt/Windows.Foundation.h>
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
static SpatialLocatability g_locatability = SpatialLocatability::Unavailable;
static SpatialStationaryFrameOfReference g_referenceFrame = nullptr;
static SpatialLocatorAttachedFrameOfReference g_attachedReferenceFrame = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void Locator_OnLocatabilityChanged(winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Foundation::IInspectable const&)
{
    g_locatability = locator.Locatability();
}

// OK
void Locator_Initialize()
{
    InitializeSRWLock(&g_lock);

    g_locator = SpatialLocator::GetDefault();
    g_locator.LocatabilityChanged(Locator_OnLocatabilityChanged);
    g_locatability = g_locator.Locatability();
    g_referenceFrame = g_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
    g_attachedReferenceFrame = g_locator.CreateAttachedFrameOfReferenceAtCurrentHeading();
}

// OK
float4x4 Locator_Locate(PerceptionTimestamp const& timestamp, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    auto const& location = locator.TryLocateAtTimestamp(timestamp, world);
    return location ? (make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position())) : float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// OK
float4x4 Locator_GetTransformTo(SpatialCoordinateSystem const& src, SpatialCoordinateSystem const& dst)
{
    auto const& location = src.TryGetTransformTo(dst);
    return location ? location.Value() : float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// OK
static SpatialCoordinateSystem Locator_GetWorldCoordinateSystemInternal(PerceptionTimestamp const& ts)
{
    return (g_locatability == SpatialLocatability::PositionalTrackingActive) ? g_referenceFrame.CoordinateSystem() : g_attachedReferenceFrame.GetStationaryCoordinateSystemAtTimestamp(ts);
}

// OK
SpatialCoordinateSystem Locator_GetWorldCoordinateSystem(PerceptionTimestamp const& ts)
{
    {
    SRWLock srw(&g_lock, false);
    if (g_world_override) { return g_world_override; }
    }
    return Locator_GetWorldCoordinateSystemInternal(ts);
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
