
#include <Windows.h>
#include "timestamps.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Perception.People.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.UI.Input.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Perception::People;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;
using namespace winrt::Windows::UI::Input;
using namespace winrt::Microsoft::MixedReality::EyeTracking;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_tracker = NULL; // CloseHandle
static EyeGazeTrackerWatcher g_watcher = nullptr;
static EyeGazeTracker g_tracker = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void OnEyeGazeTrackerAdded(const EyeGazeTrackerWatcher& sender, const EyeGazeTracker& tracker)
{
    (void)sender;
    g_tracker = tracker;
    SetEvent(g_event_tracker);
}

// OK
static void OnEyeGazeTrackerRemoved(const EyeGazeTrackerWatcher& sender, const EyeGazeTracker& tracker)
{
    (void)sender;
    (void)tracker;
}

// OK
void ExtendedEyeTracking_Initialize()
{
    EyesPose::RequestAccessAsync().get();
    g_event_tracker = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_watcher = EyeGazeTrackerWatcher();
    g_watcher.EyeGazeTrackerAdded(OnEyeGazeTrackerAdded);
    g_watcher.EyeGazeTrackerRemoved(OnEyeGazeTrackerRemoved);
    g_watcher.StartAsync().get();
    WaitForSingleObject(g_event_tracker, INFINITE);
    g_watcher.Stop();
    CloseHandle(g_event_tracker);
    g_tracker.OpenAsync(true).get();
}

// OK
void ExtendedEyeTracking_QueryCapabilities()
{
    ShowMessage("AreLeftAndRightGazesSupported: %d", g_tracker.AreLeftAndRightGazesSupported());
    ShowMessage("IsEyeOpennessSupported: %d", g_tracker.IsEyeOpennessSupported());
    ShowMessage("IsRestrictedModeSupported: %d", g_tracker.IsRestrictedModeSupported());
    ShowMessage("IsVergenceDistanceSupported: %d", g_tracker.IsVergenceDistanceSupported());
    ShowMessage("SupportedTargetFrameRates");
    for (auto const& stfr : g_tracker.SupportedTargetFrameRates()) { ShowMessage("%d FPS", stfr.FramesPerSecond()); }
}

// OK
SpatialLocator ExtendedEyeTracking_CreateLocator()
{
    return SpatialGraphInteropPreview::CreateLocatorForNode(g_tracker.TrackerSpaceLocatorNodeId());
}

// OK
void ExtendedEyeTracking_SetTargetFrameRate(int index)
{
    auto const& stfrs = g_tracker.SupportedTargetFrameRates();
    g_tracker.SetTargetFrameRate(stfrs.GetAt(index));
}

// OK
EyeGazeTrackerReading ExtendedEyeTracking_GetReading(DateTime const &dt, int64_t max_delta)
{
    EyeGazeTrackerReading r = g_tracker.TryGetReadingAtTimestamp(dt);
    return (r && (std::abs(dt.time_since_epoch().count() - r.Timestamp().time_since_epoch().count()) < max_delta)) ? r : nullptr;
}
