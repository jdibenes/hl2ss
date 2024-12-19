
#include "extended_execution.h"
#include "extended_video.h"
#include "lock.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Data.Json.h>

using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Data::Json;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static NamedMutex g_mutex;
static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_videoSource = nullptr;
static bool g_shared = false;
static bool g_ready = false;
static HANDLE g_event = NULL;
static HOOK_EV_PROC g_hook = nullptr;
static void* g_param = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedVideo_OnFailed(MediaCapture const& c, MediaCaptureFailedEventArgs const& b)
{
    (void)c;
    ShowMessage(L"ExtendedVideo_OnFailed - 0x%X : '%s'", b.Code(), b.Message().c_str());
    if (g_event != NULL) { SetEvent(g_event); }
}

// OK
static void ExtendedVideo_OnFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    auto frame = sender.TryAcquireLatestFrame();
    if (frame) { g_hook(frame, g_param); }
}

// OK
static bool ExtendedVideo_FindMediaSourceGroup(uint32_t indexGroup, uint32_t indexSource, uint32_t indexProfile, MediaFrameSourceGroup& sourceGroup, winrt::hstring& sourceId, MediaCaptureVideoProfile& profile, MediaCaptureVideoProfileMediaDescription& description)
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
static bool ExtendedVideo_FindVideoSource(winrt::hstring const& sourceId)
{
    auto sources = g_mediaCapture.FrameSources();
    if (!sources.HasKey(sourceId)) { return false; }
    g_videoSource = sources.Lookup(sourceId);
    return true;
}

// OK
static bool ExtendedVideo_ParseSubtype(winrt::hstring const& s, VideoSubtype& v)
{
    if (s == L"NV12") { v = VideoSubtype::VideoSubtype_NV12; return true; }
    if (s == L"YUY2") { v = VideoSubtype::VideoSubtype_YUY2; return true; }
    if (s == L"IYUV") { v = VideoSubtype::VideoSubtype_IYUV; return true; }
    if (s == L"I420") { v = VideoSubtype::VideoSubtype_IYUV; return true; }
    if (s == L"YV12") { v = VideoSubtype::VideoSubtype_YV12; return true; }
    return false;
}

// OK
static double ExtendedVideo_RatioToDouble(MediaRatio const& r)
{
    return (double)r.Numerator() / (double)r.Denominator();
}

// OK
winrt::hstring ExtendedVideo_QueryDevices()
{
    std::vector<winrt::hstring> ids;

    GetVideoCaptureIds(ids);

    JsonObject root = JsonObject();
    for (uint32_t i = 0; i < ids.size(); ++i)
    {
    auto sourceGroup = MediaFrameSourceGroup::FromIdAsync(ids[i]).get();
    
    JsonObject jsourceinfos = JsonObject();
    uint32_t sourceInfo = 0;
    for (auto const& source : sourceGroup.SourceInfos())
    {   
    JsonObject jvpmd = JsonObject();
    uint32_t description = 0;

    for (auto const& md : source.VideoProfileMediaDescription())
    {
    JsonObject jmd = JsonObject();
    jmd.Insert(L"Width", JsonValue::CreateNumberValue(md.Width()));
    jmd.Insert(L"Height", JsonValue::CreateNumberValue(md.Height()));
    jmd.Insert(L"FrameRate", JsonValue::CreateNumberValue(md.FrameRate()));
    jmd.Insert(L"Subtype", JsonValue::CreateStringValue(md.Subtype()));

    jvpmd.Insert(winrt::to_hstring(description++), jmd);    
    }

    JsonObject jsource = JsonObject();
    jsource.Insert(L"MediaStreamType", JsonValue::CreateNumberValue((int)source.MediaStreamType()));
    jsource.Insert(L"MediaSourceKind", JsonValue::CreateNumberValue((int)source.SourceKind()));
    jsource.Insert(L"Id", JsonValue::CreateStringValue(source.Id()));
    jsource.Insert(L"ProfileId", JsonValue::CreateStringValue(source.ProfileId()));
    jsource.Insert(L"VideoProfileMediaDescription", jvpmd);

    jsourceinfos.Insert(winrt::to_hstring(sourceInfo++), jsource);
    }

    JsonObject jvideoprofiles = JsonObject();
    uint32_t profileIndex = 0;
    for (auto const& profile : MediaCapture::FindAllVideoProfiles(sourceGroup.Id()))
    {
    JsonObject jdescription = JsonObject();
    uint32_t description = 0;

    for (auto const& md : profile.SupportedRecordMediaDescription())
    {
    JsonObject jmd = JsonObject();
    jmd.Insert(L"Width", JsonValue::CreateNumberValue(md.Width()));
    jmd.Insert(L"Height", JsonValue::CreateNumberValue(md.Height()));
    jmd.Insert(L"FrameRate", JsonValue::CreateNumberValue(md.FrameRate()));
    jmd.Insert(L"Subtype", JsonValue::CreateStringValue(md.Subtype()));

    jdescription.Insert(winrt::to_hstring(description++), jmd);
    }

    JsonObject jprofile = JsonObject();
    jprofile.Insert(L"Id", JsonValue::CreateStringValue(profile.Id()));
    jprofile.Insert(L"SupportedRecordMediaDescription", jdescription);

    jvideoprofiles.Insert(winrt::to_hstring(profileIndex++), jprofile);
    }

    JsonObject jsourcegroup = JsonObject();
    jsourcegroup.Insert(L"DisplayName", JsonValue::CreateStringValue(sourceGroup.DisplayName()));
    jsourcegroup.Insert(L"Id", JsonValue::CreateStringValue(sourceGroup.Id()));
    jsourcegroup.Insert(L"SourceInfos", jsourceinfos);
    jsourcegroup.Insert(L"VideoProfiles", jvideoprofiles);

    root.Insert(winrt::to_hstring(i), jsourcegroup);
    }

    return root.ToString();
}

// OK
void ExtendedVideo_CreateNamedMutex(wchar_t const* name)
{
    g_mutex.Create(name);
}

// OK
void ExtendedVideo_Open(MRCVideoOptions const& options)
{
    if (!g_mutex.Acquire(0)) { return; }

    MediaFrameSourceGroup sourceGroup = nullptr;
    winrt::hstring sourceId;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaCaptureInitializationSettings settings;
    bool ok;

    ok = ExtendedVideo_FindMediaSourceGroup(static_cast<uint32_t>(options.global_opacity), static_cast<uint32_t>(options.output_width), static_cast<uint32_t>(options.output_height), sourceGroup, sourceId, profile, description);
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

    g_mediaCapture.Failed({ ExtendedVideo_OnFailed });

    ok = ExtendedVideo_FindVideoSource(sourceId);
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
void ExtendedVideo_Close()
{
    g_ready = false;
    g_videoSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
    g_mutex.Release();
}

// OK
bool ExtendedVideo_Status()
{
    return g_ready;
}

// OK
bool ExtendedVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate, VideoSubtype& subtype)
{
    if (g_shared)
    {
    width     = static_cast<uint16_t>(g_videoSource.CurrentFormat().VideoFormat().Width());
    height    = static_cast<uint16_t>(g_videoSource.CurrentFormat().VideoFormat().Height());
    framerate = static_cast<uint8_t>(ExtendedVideo_RatioToDouble(g_videoSource.CurrentFormat().FrameRate()));

    return ExtendedVideo_ParseSubtype(g_videoSource.CurrentFormat().Subtype(), subtype);
    }

    for (auto const& format : g_videoSource.SupportedFormats())
    {
    if (format.VideoFormat().Width()                                           != width)     { continue; }
    if (format.VideoFormat().Height()                                          != height)    { continue; }
    if (static_cast<uint32_t>(ExtendedVideo_RatioToDouble(format.FrameRate())) != framerate) { continue; }    
    if (!ExtendedVideo_ParseSubtype(format.Subtype(), subtype))                              { continue; }

    g_videoSource.SetFormatAsync(format).get();

    return true;
    }

    return false;
}

// OK
void ExtendedVideo_ExecuteSensorLoop(MediaFrameReaderAcquisitionMode mode, HOOK_EV_PROC hook, void* param, HANDLE event_stop)
{
    g_hook  = hook;
    g_param = param;
    g_event = event_stop;

    auto reader = g_mediaCapture.CreateFrameReaderAsync(g_videoSource).get();

    reader.AcquisitionMode(mode);
    reader.FrameArrived(ExtendedVideo_OnFrameArrived);

    reader.StartAsync().get();
    WaitForSingleObject(event_stop, INFINITE);
    reader.StopAsync().get();

    reader.Close();

    g_event = NULL;
}
