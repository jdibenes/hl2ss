
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static bool PersonalVideo_FindMediaSourceGroup(uint32_t width, uint32_t height, double framerate, MediaFrameSourceGroup &sourceGroup, MediaCaptureVideoProfile &profile, MediaCaptureVideoProfileMediaDescription &description)
{
    auto mediaFrameSourceGroups = MediaFrameSourceGroup::FindAllAsync().get();

    for (auto const& mediaFrameSourceGroup : mediaFrameSourceGroups)
    {
    for (auto const& knownVideoProfile : MediaCapture::FindKnownVideoProfiles(mediaFrameSourceGroup.Id(), KnownVideoProfile::VideoConferencing))
    {
    for (auto const& supportedRecordMediaDescription : knownVideoProfile.SupportedRecordMediaDescription())
    {
    if (supportedRecordMediaDescription.Width()     != width)     { continue; }
    if (supportedRecordMediaDescription.Height()    != height)    { continue; }
    if (supportedRecordMediaDescription.FrameRate() != framerate) { continue; }

    sourceGroup = mediaFrameSourceGroup;
    profile = knownVideoProfile;
    description = supportedRecordMediaDescription;
    return true;
    }
    }
    }

    return false;
}

// OK
static bool PersonalVideo_FindVideoSource(MediaCapture const& mediaCapture, MediaFrameSource& videoSource)
{
    for (auto const& frameSource : mediaCapture.FrameSources())
    {
    auto const& frameSourceInfo = frameSource.Value().Info();
    if ((frameSourceInfo.MediaStreamType() != MediaStreamType::VideoRecord) || (frameSourceInfo.SourceKind() != MediaFrameSourceKind::Color)) { continue; }
    videoSource = frameSource.Value();
    return true;
    }

    return false;
}

// OK
static bool PersonalVideo_FindVideoFormat(MediaFrameSource const& videoSource, uint32_t width, uint32_t height, uint32_t frameRate, MediaFrameFormat& selectedFormat)
{
    for (auto const& format : videoSource.SupportedFormats())
    {
    if (format.VideoFormat().Width()   != width)     { continue; }
    if (format.VideoFormat().Height()  != height)    { continue; }    
    if (format.FrameRate().Numerator() != frameRate) { continue; } // assuming all denominators are 1
    selectedFormat = format;
    return true;
    }

    return false;
}

// OK
void PersonalVideo_Initialize(MediaCapture const& mediaCapture, MediaFrameSource& videoSource)
{
    uint32_t const width     = 1920;
    uint32_t const height    = 1080;
    double   const framerate = 30;

    MediaFrameSourceGroup sourceGroup = nullptr;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaCaptureInitializationSettings settings;

    PersonalVideo_FindMediaSourceGroup(width, height, framerate, sourceGroup, profile, description);

    settings.VideoProfile(profile);
    settings.RecordMediaDescription(description);
    settings.VideoDeviceId(sourceGroup.Id());
    settings.StreamingCaptureMode(StreamingCaptureMode::Video);
    settings.MemoryPreference(MediaCaptureMemoryPreference::Cpu);
    settings.SharingMode(MediaCaptureSharingMode::ExclusiveControl);
    settings.SourceGroup(sourceGroup);
    settings.MediaCategory(MediaCategory::Media);

    mediaCapture.InitializeAsync(settings).get();

    PersonalVideo_FindVideoSource(mediaCapture, videoSource);
}

// OK
bool PersonalVideo_SetFormat(MediaFrameSource const& videoSource, uint32_t width, uint32_t height, uint32_t framerate)
{
    MediaFrameFormat selectedFormat = nullptr;
    bool ok;

    ok = PersonalVideo_FindVideoFormat(videoSource, width, height, framerate, selectedFormat);
    if (!ok) { return false; }

    videoSource.SetFormatAsync(selectedFormat).get();
    return true;
}
