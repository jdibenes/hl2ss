
#include "extended_execution.h"
#include "locator.h"
#include "timestamp.h"
#include "extended_eye_tracking.h"
#include "lock.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Perception.People.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.UI.Input.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
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
static SpatialLocator g_locator = nullptr;
static uint8_t g_fps = 30;
static UINT64 g_utc_offset = 0;
static bool g_ready = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedEyeTracking_OnEyeGazeTrackerAdded(EyeGazeTrackerWatcher const& sender, EyeGazeTracker const& tracker)
{
    (void)sender;
    g_tracker = tracker;
    SetEvent(g_event_tracker);
}

// OK
static void ExtendedEyeTracking_OnEyeGazeTrackerRemoved(EyeGazeTrackerWatcher const& sender, EyeGazeTracker const& tracker)
{
    (void)sender;
    (void)tracker;
}

// OK
bool ExtendedEyeTracking_WaitForConsent()
{
    Cleaner log_error_eyetracker([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedEyeTracker); });
    if (EyesPose::RequestAccessAsync().get() != GazeInputAccessStatus::Allowed) { return false; }
    log_error_eyetracker.Set(false);
    return true;
}

// OK
void ExtendedEyeTracking_Open(bool restricted_mode)
{
    if (g_ready) { return; }
    g_event_tracker = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_watcher = EyeGazeTrackerWatcher();
    g_watcher.EyeGazeTrackerAdded(ExtendedEyeTracking_OnEyeGazeTrackerAdded);
    g_watcher.EyeGazeTrackerRemoved(ExtendedEyeTracking_OnEyeGazeTrackerRemoved);
    g_watcher.StartAsync().get();
    WaitForSingleObject(g_event_tracker, INFINITE);
    g_watcher.Stop();
    g_watcher = nullptr;
    CloseHandle(g_event_tracker);
    g_event_tracker = NULL;
    g_tracker.OpenAsync(restricted_mode).get();
    g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(g_tracker.TrackerSpaceLocatorNodeId());
    g_utc_offset = Timestamp_GetQPCToUTCOffset();
    g_ready = true;
}

// OK
void ExtendedEyeTracking_Close()
{
    //g_tracker.Close() // This crashes always
}

// OK
bool ExtendedEyeTracking_Status()
{
    return g_ready;
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
float4x4 ExtendedEyeTracking_GetNodeWorldPose(UINT64 host_ticks)
{
    return Locator_Locate(Timestamp_QPCToPerception(host_ticks), g_locator, Locator_GetWorldCoordinateSystem());
}

// OK
bool ExtendedEyeTracking_SetTargetFrameRate(uint8_t fps)
{
    for (auto const& stfr : g_tracker.SupportedTargetFrameRates())
    {
    if (stfr.FramesPerSecond() != fps) { continue; }
    g_tracker.SetTargetFrameRate(stfr);
    g_fps = fps;
    return true;
    }

    return false;
}

// OK
void ExtendedEyeTracking_ExecuteSensorLoop(HOOK_EET_PROC hook, void* param, HANDLE event_stop)
{
    int64_t const dt_wait = (9LL * HNS_BASE) / g_fps;
    int32_t const dt_idle = 1000 / g_fps;
    int64_t const dt_send = (HNS_BASE / 2LL) / 90LL; // filter 0,1,2,3 dt noise? 180 hz

    int64_t  last_ts = Timestamp_GetCurrentUTC();
    DateTime utc_ts  = Timestamp_UTCToDateTime(last_ts);

    do
    {
    EyeGazeTrackerReading frame = g_tracker.TryGetReadingAfterTimestamp(utc_ts);

    if (frame)
    {
    utc_ts = frame.Timestamp();
    int64_t ts = utc_ts.time_since_epoch().count();
    if ((ts - last_ts) <= dt_send) { continue; }
    last_ts = ts;
    }
    else
    {
    Sleep(dt_idle);
    int64_t ts = Timestamp_GetCurrentUTC();
    if ((ts - last_ts) <= dt_wait) { continue; }
    last_ts = ts;
    utc_ts = Timestamp_UTCToDateTime(last_ts);
    }

    hook(frame, last_ts - g_utc_offset, param);
    }
    while (WaitForSingleObject(event_stop, 0) == WAIT_TIMEOUT);
}
