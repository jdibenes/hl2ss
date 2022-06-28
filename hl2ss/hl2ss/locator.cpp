
#include "utilities.h"
#include "types.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

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
    g_locator = SpatialLocator::GetDefault();
    g_locator.LocatabilityChanged(Locator_OnLocatabilityChanged);
    g_locatability = g_locator.Locatability();
    g_referenceFrame = g_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
    g_attachedReferenceFrame = g_locator.CreateAttachedFrameOfReferenceAtCurrentHeading();
}

// OK
PerceptionTimestamp Locator_QPCTimestampToPerceptionTimestamp(LONGLONG qpctime)
{
    return PerceptionTimestampHelper::FromSystemRelativeTargetTime(std::chrono::duration<int64_t, std::ratio<1, HNS_BASE>>(qpctime));
}

// OK
float4x4 Locator_Locate(PerceptionTimestamp const& timestamp, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    auto location = locator.TryLocateAtTimestamp(timestamp, world);
    return location ? (make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position())) : float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// OK
float4x4 Locator_Locate(UINT64 qpctime, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    return Locator_Locate(Locator_QPCTimestampToPerceptionTimestamp(qpctime), locator, world);
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
    return (g_locatability == SpatialLocatability::PositionalTrackingActive) ? g_referenceFrame.CoordinateSystem() : g_attachedReferenceFrame.GetStationaryCoordinateSystemAtTimestamp(Locator_QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimeHns()));
}
