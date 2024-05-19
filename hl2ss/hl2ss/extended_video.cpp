
#include "custom_media_types.h"
#include "custom_video_effect.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Data.Json.h>

using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Data::Json;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event = NULL;

static bool g_ready = false;
static bool g_shared = false;
static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_videoSource = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedVideo_OnFailed(MediaCapture const&, MediaCaptureFailedEventArgs const& b)
{
    ShowMessage(L"ExtendedVideo_OnFailed - 0x%X : '%s'", b.Code(), b.Message().c_str());
    if (g_event != NULL) { SetEvent(g_event); }
}

// OK
void ExtendedVideo_QueryDevices(winrt::hstring& out)
{
    std::vector<winrt::hstring> ids;

    GetVideoCaptureIds(ids);

    JsonObject root = JsonObject();
    for (uint32_t i = 0; i < ids.size(); ++i)
    {
    auto sourceGroup = MediaFrameSourceGroup::FromIdAsync(ids[i]).get();
    
    JsonObject jsourceinfos = JsonObject();
    uint32_t sourceInfo = 0;
    for (auto source : sourceGroup.SourceInfos())
    {   
    JsonObject jvpmd = JsonObject();
    uint32_t description = 0;

    for (auto md : source.VideoProfileMediaDescription())
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
    for (auto profile : MediaCapture::FindAllVideoProfiles(sourceGroup.Id()))
    {
    JsonObject jdescription = JsonObject();
    uint32_t description = 0;

    for (auto md : profile.SupportedRecordMediaDescription())
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

    out = root.ToString();
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
static bool ExtendedVideo_FindVideoSource(MediaCapture const& mediaCapture, winrt::hstring const& sourceId, MediaFrameSource& videoSource)
{
    auto sources = mediaCapture.FrameSources();
    if (!sources.HasKey(sourceId)) { return false; }
    videoSource = sources.Lookup(sourceId);
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
static bool ExtendedVideo_CopyVideoFormat(uint16_t& width, uint16_t& height, uint8_t& framerate, VideoSubtype& subtype)
{
    width     = (uint16_t)g_videoSource.CurrentFormat().VideoFormat().Width();
    height    = (uint16_t)g_videoSource.CurrentFormat().VideoFormat().Height();
    framerate = (uint8_t) ExtendedVideo_RatioToDouble(g_videoSource.CurrentFormat().FrameRate());

    return ExtendedVideo_ParseSubtype(g_videoSource.CurrentFormat().Subtype(), subtype);
}

// OK
static bool ExtendedVideo_FindVideoFormat(MediaFrameSource const& videoSource, uint32_t width, uint32_t height, uint32_t frameRate, VideoSubtype& subtype, MediaFrameFormat& selectedFormat)
{
    for (auto const& format : videoSource.SupportedFormats())
    {
    if (format.VideoFormat().Width()                              != width)     { continue; }
    if (format.VideoFormat().Height()                             != height)    { continue; }
    if ((uint32_t)ExtendedVideo_RatioToDouble(format.FrameRate()) != frameRate) { continue; }
    if (!ExtendedVideo_ParseSubtype(format.Subtype(), subtype))                 { continue; }

    selectedFormat = format;
    return true;
    }

    return false;
}

// OK
void ExtendedVideo_RegisterEvent(HANDLE h)
{
    g_event = h;
}

// OK
void ExtendedVideo_Open(MRCVideoOptions const& options)
{
    MediaFrameSourceGroup sourceGroup = nullptr;
    winrt::hstring sourceId;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaCaptureInitializationSettings settings;
    bool ok;

    ok = ExtendedVideo_FindMediaSourceGroup((uint32_t)options.global_opacity, (uint32_t)options.output_width, (uint32_t)options.output_height, sourceGroup, sourceId, profile, description);
    if (!ok) { return; }

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
    g_mediaCapture.InitializeAsync(settings).get();

    g_mediaCapture.Failed({ ExtendedVideo_OnFailed });
    ok = ExtendedVideo_FindVideoSource(g_mediaCapture, sourceId, g_videoSource);
    if (!ok)
    {
    g_videoSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
    return;
    }

    g_shared = options.shared;
    g_ready  = true;
}

// OK
void ExtendedVideo_Close()
{
    g_videoSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;

    g_ready = false;
}

// OK
bool ExtendedVideo_Status()
{
    return g_ready;
}

// OK
bool ExtendedVideo_SetFormat(uint16_t& width, uint16_t& height, uint8_t& framerate, VideoSubtype& subtype)
{
    MediaFrameFormat selectedFormat = nullptr;
    bool ok;

    if (g_shared)
    {
    ok = ExtendedVideo_CopyVideoFormat(width, height, framerate, subtype);
    return ok;
    }

    ok = ExtendedVideo_FindVideoFormat(g_videoSource, width, height, framerate, subtype, selectedFormat);
    if (!ok) { return false; }

    g_videoSource.SetFormatAsync(selectedFormat).get();
    return true;
}

// OK
MediaFrameReader ExtendedVideo_CreateFrameReader()
{
    return g_mediaCapture.CreateFrameReaderAsync(g_videoSource).get();
}
