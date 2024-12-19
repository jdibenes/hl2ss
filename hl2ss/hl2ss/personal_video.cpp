
#include "extended_execution.h"
#include "locator.h"
#include "personal_video.h"
#include "lock.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static NamedMutex g_mutex;
static CRITICAL_SECTION g_lock; // DeleteCriticalSection
static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_videoSource = nullptr;
static bool g_shared = false;
static bool g_ready = false;
static HANDLE g_event = NULL;
static HOOK_PV_PROC g_hook = nullptr;
static void* g_param = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void PersonalVideo_OnFailed(MediaCapture const& c, MediaCaptureFailedEventArgs const& b)
{
    (void)c;
    ShowMessage(L"PersonalVideo_OnFailed - 0x%X : '%s'", b.Code(), b.Message().c_str());
    if (g_event != NULL) { SetEvent(g_event); }
}

// OK
static void PersonalVideo_OnVideoFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    auto frame = sender.TryAcquireLatestFrame();
    if (frame) { g_hook(frame, g_param); }
}

// OK
static bool PersonalVideo_FindMediaSourceGroup(uint32_t width, uint32_t height, double framerate, MediaFrameSourceGroup& sourceGroup, MediaCaptureVideoProfile& profile, MediaCaptureVideoProfileMediaDescription& description)
{
    auto const& mediaFrameSourceGroup = MediaFrameSourceGroup::FromIdAsync(GetBuiltInVideoCaptureId()).get();

    for (auto const& knownVideoProfile : MediaCapture::FindKnownVideoProfiles(mediaFrameSourceGroup.Id(), KnownVideoProfile::VideoConferencing))
    {
    for (auto const& supportedRecordMediaDescription : knownVideoProfile.SupportedRecordMediaDescription())
    {
    if (supportedRecordMediaDescription.Width()     != width)     { continue; }
    if (supportedRecordMediaDescription.Height()    != height)    { continue; }
    if (supportedRecordMediaDescription.FrameRate() != framerate) { continue; }

    sourceGroup = mediaFrameSourceGroup;
    profile     = knownVideoProfile;
    description = supportedRecordMediaDescription;

    return true;
    }
    }

    return false;
}

// OK
static bool PersonalVideo_FindVideoSource()
{
    for (auto const& frameSource : g_mediaCapture.FrameSources())
    {
    auto const& frameSourceInfo = frameSource.Value().Info();
    if ((frameSourceInfo.MediaStreamType() != MediaStreamType::VideoRecord) || (frameSourceInfo.SourceKind() != MediaFrameSourceKind::Color)) { continue; }
    g_videoSource = frameSource.Value();
    return true;
    }

    return false;
}

// OK
void PersonalVideo_CreateNamedMutex(wchar_t const* name)
{
    g_mutex.Create(name);
}

// OK
void PersonalVideo_Startup()
{
    InitializeCriticalSection(&g_lock);
}

// OK
void PersonalVideo_Cleanup()
{
    DeleteCriticalSection(&g_lock);
}

// OK
void PersonalVideo_Open(MRCVideoOptions const& options)
{
    if (!g_mutex.Acquire(0)) { return; }

    uint32_t const width     = 1920;
    uint32_t const height    = 1080;
    double   const framerate = 30;

    MediaFrameSourceGroup sourceGroup = nullptr;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaCaptureInitializationSettings settings;
    bool ok;

    CriticalSection cs(&g_lock);

    ok = PersonalVideo_FindMediaSourceGroup(width, height, framerate, sourceGroup, profile, description);
    if (!ok) { goto _fail_fmsg; }

    if (!options.shared)
    {
    settings.VideoProfile(profile);
    settings.RecordMediaDescription(description);
    settings.SharingMode(MediaCaptureSharingMode::ExclusiveControl);
    }
    else
    {
    settings.SharingMode(MediaCaptureSharingMode::SharedReadOnly);
    }

    settings.VideoDeviceId(sourceGroup.Id());
    settings.StreamingCaptureMode(StreamingCaptureMode::Video);
    settings.MemoryPreference(MediaCaptureMemoryPreference::Cpu);
    settings.SourceGroup(sourceGroup);
    settings.MediaCategory(MediaCategory::Media);

    g_mediaCapture = MediaCapture();

    try 
    {
    Cleaner log_error_camera([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedCamera); });
    g_mediaCapture.InitializeAsync(settings).get();
    log_error_camera.Set(false);
    }
    catch (...) 
    {
    goto _fail_open; 
    }
    
    g_mediaCapture.Failed({ PersonalVideo_OnFailed });
    if (options.enable) { g_mediaCapture.AddVideoEffectAsync(MRCVideoEffect(options), MediaStreamType::VideoRecord).get(); }

    ok = PersonalVideo_FindVideoSource();
    if (!ok) { goto _fail_find; }

    g_shared = options.shared;
    g_ready  = true;

    return;

_fail_find:
    g_mediaCapture.Close();

_fail_open:
    g_mediaCapture = nullptr;

_fail_fmsg:
    g_mutex.Release();
}

// OK
void PersonalVideo_Close()
{
    CriticalSection cs(&g_lock);
    g_ready = false;
    g_videoSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
    g_mutex.Release();
}

// OK
bool PersonalVideo_Status()
{
    return g_ready;
}

// OK
uint32_t PersonalVideo_GetStride(uint32_t width)
{
    uint32_t const mask = 64 - 1;
    return (width + mask) & ~mask;
}

// OK
float4x4 PersonalVideo_GetFrameWorldPose(MediaFrameReference const& frame)
{
    return Locator_GetTransformTo(frame.CoordinateSystem(), Locator_GetWorldCoordinateSystem());
}

// OK
bool PersonalVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate)
{
    if (g_shared)
    {
    width     = static_cast<uint16_t>(g_videoSource.CurrentFormat().VideoFormat().Width());
    height    = static_cast<uint16_t>(g_videoSource.CurrentFormat().VideoFormat().Height());
    framerate = static_cast<uint8_t>( g_videoSource.CurrentFormat().FrameRate().Numerator());
    return true;
    }

    for (auto const& format : g_videoSource.SupportedFormats())
    {
    if (format.VideoFormat().Width()   != width)     { continue; }
    if (format.VideoFormat().Height()  != height)    { continue; }    
    if (format.FrameRate().Numerator() != framerate) { continue; } // assuming all denominators are 1

    g_videoSource.SetFormatAsync(format).get();

    return true;
    }

    return false;
}

// OK
void PersonalVideo_ExecuteSensorLoop(MediaFrameReaderAcquisitionMode mode, HOOK_PV_PROC hook, void* param, HANDLE event_stop)
{
    g_hook  = hook;
    g_param = param;
    g_event = event_stop;

    auto reader = g_mediaCapture.CreateFrameReaderAsync(g_videoSource).get();
    
    reader.AcquisitionMode(mode);
    reader.FrameArrived(PersonalVideo_OnVideoFrameArrived);

    reader.StartAsync().get();
    WaitForSingleObject(g_event, INFINITE);
    reader.StopAsync().get();

    reader.Close();

    g_event = NULL;
}

// OK
void PersonalVideo_SetFocus(uint32_t focusmode, uint32_t autofocusrange, uint32_t distance, uint32_t value, uint32_t disabledriverfallback)
{
    AutoFocusRange afr;
    ManualFocusDistance mfd;
    FocusMode fm;
    FocusSettings fs;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    switch (focusmode)
    {
    case 0:  fm = FocusMode::Auto;       break;
    case 1:  fm = FocusMode::Single;     break;
    case 2:  fm = FocusMode::Continuous; break;
    case 3:  fm = FocusMode::Manual;     break;
    default: return;
    }

    switch (autofocusrange)
    {
    case 0:  afr = AutoFocusRange::FullRange; break;
    case 1:  afr = AutoFocusRange::Macro;     break;
    case 2:  afr = AutoFocusRange::Normal;    break;
    default: return;
    }

    switch (distance)
    {
    case 0:  mfd = ManualFocusDistance::Infinity; break;
    case 2:  mfd = ManualFocusDistance::Nearest;  break;
    default: return;
    }

    if ((value < 170) || (value > 10000)) { return; }

    fs.Mode(fm);
    fs.AutoFocusRange(afr);
    fs.Distance(mfd);
    fs.Value(value);
    fs.DisableDriverFallback(disabledriverfallback != 0);

    g_mediaCapture.VideoDeviceController().FocusControl().Configure(fs);
    g_mediaCapture.VideoDeviceController().FocusControl().FocusAsync().get();
}

// OK
void PersonalVideo_SetVideoTemporalDenoising(uint32_t mode)
{
    VideoTemporalDenoisingMode vtdm;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    switch (mode)
    {
    case 0:  vtdm = VideoTemporalDenoisingMode::Off; break;
    case 1:  vtdm = VideoTemporalDenoisingMode::On;  break;
    default: return;
    }

    g_mediaCapture.VideoDeviceController().VideoTemporalDenoisingControl().Mode(vtdm);
}

// OK
void PersonalVideo_SetWhiteBalance_Preset(uint32_t preset)
{
    ColorTemperaturePreset ctp;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    switch (preset)
    {
    case 0:  ctp = ColorTemperaturePreset::Auto;        break;
    case 1:  ctp = ColorTemperaturePreset::Manual;      break;
    case 2:  ctp = ColorTemperaturePreset::Cloudy;      break;
    case 3:  ctp = ColorTemperaturePreset::Daylight;    break;
    case 4:  ctp = ColorTemperaturePreset::Flash;       break;
    case 5:  ctp = ColorTemperaturePreset::Fluorescent; break;
    case 6:  ctp = ColorTemperaturePreset::Tungsten;    break;
    case 7:  ctp = ColorTemperaturePreset::Candlelight; break;
    default: return;
    }

    g_mediaCapture.VideoDeviceController().WhiteBalanceControl().SetPresetAsync(ctp).get();
}

// OK
void PersonalVideo_SetWhiteBalance_Value(uint32_t value)
{
    uint32_t temperature;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    temperature = value * 25;
    if ((temperature >= 2300) && (temperature <= 7500)) { g_mediaCapture.VideoDeviceController().WhiteBalanceControl().SetValueAsync(temperature).get(); }    
}

// OK
void PersonalVideo_SetExposure(uint32_t setauto, uint32_t value)
{
    uint32_t exposure;
    bool mode;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    
    mode = setauto != 0;
    g_mediaCapture.VideoDeviceController().ExposureControl().SetAutoAsync(mode).get();
    if (mode) { return; }
    exposure = value * 10;
    if ((exposure >= 1000) && (exposure <= 660000)) { g_mediaCapture.VideoDeviceController().ExposureControl().SetValueAsync(TimeSpan(exposure)).get(); }    
}

// OK
void PersonalVideo_SetExposurePriorityVideo(uint32_t enabled)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    g_mediaCapture.VideoDeviceController().ExposurePriorityVideoControl().Enabled(enabled != 0);
}

// OK
void PersonalVideo_SetSceneMode(uint32_t mode)
{
    CaptureSceneMode value;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    switch (mode)
    {
    case 0:  value = CaptureSceneMode::Auto;          break;
    case 2:  value = CaptureSceneMode::Macro;         break;
    case 3:  value = CaptureSceneMode::Portrait;      break;
    case 4:  value = CaptureSceneMode::Sport;         break;
    case 5:  value = CaptureSceneMode::Snow;          break;
    case 6:  value = CaptureSceneMode::Night;         break;
    case 7:  value = CaptureSceneMode::Beach;         break;
    case 8:  value = CaptureSceneMode::Sunset;        break;
    case 9:  value = CaptureSceneMode::Candlelight;   break;
    case 10: value = CaptureSceneMode::Landscape;     break;
    case 11: value = CaptureSceneMode::NightPortrait; break;
    case 12: value = CaptureSceneMode::Backlit;       break;
    default: return;
    }

    g_mediaCapture.VideoDeviceController().SceneModeControl().SetValueAsync(value).get();
}

// OK
void PersonalVideo_SetIsoSpeed(uint32_t setauto, uint32_t value)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    if (setauto != 0) { g_mediaCapture.VideoDeviceController().IsoSpeedControl().SetAutoAsync().get(); } else if ((value >= 100) && (value <= 3200)) { g_mediaCapture.VideoDeviceController().IsoSpeedControl().SetValueAsync(value).get(); }
}

// OK
void PersonalVideo_SetBacklightCompensation(bool enable)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    g_mediaCapture.VideoDeviceController().BacklightCompensation().TrySetValue(enable ? 1.0 : 0.0);
}

// OK
void PersonalVideo_SetDesiredOptimization(uint32_t mode)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    if (mode > 6) { return; }
    g_mediaCapture.VideoDeviceController().DesiredOptimization(static_cast<MediaCaptureOptimization>(mode));
}

// OK
void PersonalVideo_SetPrimaryUse(uint32_t mode)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    if (mode > 2) { return; }
    g_mediaCapture.VideoDeviceController().PrimaryUse(static_cast<CaptureUse>(mode));
}

// OK
void PersonalVideo_SetOpticalImageStabilization(uint32_t mode)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    if (mode > 1) { return; }
    g_mediaCapture.VideoDeviceController().OpticalImageStabilizationControl().Mode(static_cast<OpticalImageStabilizationMode>(mode));
}

// OK
void PersonalVideo_SetHdrVideo(uint32_t mode)
{
    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }
    if (mode > 2) { return; }
    g_mediaCapture.VideoDeviceController().HdrVideoControl().Mode(static_cast<HdrVideoMode>(mode));
}

// OK
void PersonalVideo_SetRegionsOfInterest(bool clear, bool set, bool auto_exposure, bool auto_focus, bool bounds_normalized, float x, float y, float w, float h, uint32_t type, uint32_t weight)
{
    RegionOfInterest roi;

    CriticalSection cs(&g_lock);
    if (!g_ready || g_shared) { return; }

    auto const& mc = g_mediaCapture.VideoDeviceController().RegionsOfInterestControl();

    if (clear)
    {
    mc.ClearRegionsAsync().get();
    }

    if (set)
    {
    roi.AutoExposureEnabled(auto_exposure);
    roi.AutoFocusEnabled(auto_focus);
    roi.AutoWhiteBalanceEnabled(false);
    roi.Bounds({ x, y, w, h });
    roi.BoundsNormalized(bounds_normalized);
    roi.Type(static_cast<RegionOfInterestType>(type & 1));
    roi.Weight(weight % 101);

    mc.SetRegionsAsync({ roi }).get();
    }
}
