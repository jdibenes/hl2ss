
#include "extended_execution.h"
#include "extended_depth.h"
#include "lock.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_videoSource = nullptr;
static bool g_shared = false;
static bool g_ready = false;
static HANDLE g_event = NULL;
static HOOK_EZ_PROC g_hook = nullptr;
static void* g_param = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedDepth_OnFailed(MediaCapture const& c, MediaCaptureFailedEventArgs const& b)
{
    (void)c;
    ShowMessage(L"ExtendedDepth_OnFailed - 0x%X : '%s'", b.Code(), b.Message().c_str());
    if (g_event != NULL) { SetEvent(g_event); }
}

// OK
static void ExtendedDepth_OnFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    auto frame = sender.TryAcquireLatestFrame();
    if (frame) { g_hook(frame, g_param); }
}

// OK
static bool ExtendedDepth_FindMediaSourceGroup(uint32_t indexGroup, uint32_t indexSource, uint32_t indexProfile, MediaFrameSourceGroup& sourceGroup, winrt::hstring& sourceId, MediaCaptureVideoProfile& profile, MediaCaptureVideoProfileMediaDescription& description)
{
    std::vector<winrt::hstring> ids;

    GetVideoCaptureIds(ids);
    if (indexGroup >= ids.size()) { return false; }
    sourceGroup = MediaFrameSourceGroup::FromIdAsync(ids[indexGroup]).get();

    auto sources = sourceGroup.SourceInfos();
    if (indexSource >= sources.Size()) { return false; }
    sourceId = sources.GetAt(indexSource).Id();

    auto profiles = MediaCapture::FindAllVideoProfiles(sourceGroup.Id());
    if (profiles.Size() <= 0)
    {
    profile     = nullptr;
    description = nullptr;
    }
    else
    {
    if (indexProfile >= profiles.Size()) { return false; }
    profile = profiles.GetAt(indexProfile);
    auto descriptions = profile.SupportedRecordMediaDescription();
    if (descriptions.Size() <= 0) { return false; }
    description = descriptions.GetAt(0);
    }

    return true;
}

// OK
static bool ExtendedDepth_FindVideoSource(winrt::hstring const& sourceId)
{
    auto sources = g_mediaCapture.FrameSources();
    if (!sources.HasKey(sourceId)) { return false; }
    g_videoSource = sources.Lookup(sourceId);
    return true;
}

// OK
void ExtendedDepth_Open(MRCVideoOptions const& options)
{
    MediaFrameSourceGroup sourceGroup = nullptr;
    winrt::hstring sourceId;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaCaptureInitializationSettings settings;
    bool ok;

    ok = ExtendedDepth_FindMediaSourceGroup(static_cast<uint32_t>(options.global_opacity), static_cast<uint32_t>(options.output_width), static_cast<uint32_t>(options.output_height), sourceGroup, sourceId, profile, description);
    if (!ok) { return; }

    settings.VideoProfile(profile);
    settings.RecordMediaDescription(description);
    settings.SharingMode(MediaCaptureSharingMode::ExclusiveControl);
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

    g_mediaCapture.Failed({ ExtendedDepth_OnFailed });

    ok = ExtendedDepth_FindVideoSource(sourceId);
    if (!ok) { goto _fail_find; }

    g_shared = options.shared;
    g_ready  = true;

    return;

_fail_find:
    g_mediaCapture.Close();

_fail_open:
    g_mediaCapture = nullptr;
}

// OK
void ExtendedDepth_Close()
{
    g_ready = false;
    g_videoSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
}

// OK
bool ExtendedDepth_Status()
{
    return g_ready;
}

// OK
bool ExtendedDepth_SetFormat(uint32_t index, uint16_t& width, uint16_t& height, uint8_t& framerate, winrt::hstring& subtype)
{
    auto supported_formats = g_videoSource.SupportedFormats();

    if (index >= supported_formats.Size()) { return false; }

    auto format = supported_formats.GetAt(index);

    width     = static_cast<uint16_t>(format.VideoFormat().Width());
    height    = static_cast<uint16_t>(format.VideoFormat().Height());
    framerate = static_cast<uint8_t>(static_cast<double>(format.FrameRate().Numerator()) / static_cast<double>(format.FrameRate().Numerator()));
    subtype   = format.Subtype();

    g_videoSource.SetFormatAsync(format).get();

    return true;
}

// OK
void ExtendedDepth_ExecuteSensorLoop(MediaFrameReaderAcquisitionMode mode, HOOK_EZ_PROC hook, void* param, HANDLE event_stop)
{
    g_hook  = hook;
    g_param = param;
    g_event = event_stop;

    auto reader = g_mediaCapture.CreateFrameReaderAsync(g_videoSource).get();

    reader.AcquisitionMode(mode);
    reader.FrameArrived(ExtendedDepth_OnFrameArrived);

    reader.StartAsync().get();
    WaitForSingleObject(event_stop, INFINITE);
    reader.StopAsync().get();

    reader.Close();

    g_event = NULL;
}
