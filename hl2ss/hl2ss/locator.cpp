
#include "utilities.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

static SpatialLocator g_locator = nullptr;
static SpatialLocatability g_locatability = SpatialLocatability::Unavailable;
static SpatialStationaryFrameOfReference g_referenceFrame = nullptr;
static SpatialLocatorAttachedFrameOfReference g_attachedReferenceFrame = nullptr;

// OK
static PerceptionTimestamp GetCurrentPerceptionTimestamp()
{
    return PerceptionTimestampHelper::FromSystemRelativeTargetTime(std::chrono::duration<int64_t, std::ratio<1, 10'000'000>>(GetCurrentQPCTimeHns()));
}

// OK
static void OnLocatabilityChanged(winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Foundation::IInspectable const&)
{
    g_locatability = locator.Locatability();
}

// OK
SpatialCoordinateSystem GetWorldCoordinateSystem()
{
    return (g_locatability == SpatialLocatability::PositionalTrackingActive) ? g_referenceFrame.CoordinateSystem() : g_attachedReferenceFrame.GetStationaryCoordinateSystemAtTimestamp(GetCurrentPerceptionTimestamp());
}

// OK
void InitializeLocator()
{
    g_locator = SpatialLocator::GetDefault();
    g_locator.LocatabilityChanged(OnLocatabilityChanged);
    g_locatability = g_locator.Locatability();
    g_referenceFrame = g_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
    g_attachedReferenceFrame = g_locator.CreateAttachedFrameOfReferenceAtCurrentHeading();
}
