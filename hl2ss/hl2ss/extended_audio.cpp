
#include "custom_audio_effect.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_audioSource = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void ExtendedAudio_Open(MRCAudioOptions const& options)
{
    MediaCaptureInitializationSettings settings;

    g_mediaCapture = MediaCapture();

    settings.AudioDeviceId(MediaDevice::GetDefaultAudioCaptureId(AudioDeviceRole::Default));
    settings.StreamingCaptureMode(StreamingCaptureMode::Audio);
    settings.SharingMode(MediaCaptureSharingMode::SharedReadOnly);
    settings.MediaCategory(MediaCategory::Media);

    g_mediaCapture.InitializeAsync(settings).get();
    g_mediaCapture.AddAudioEffectAsync(MRCAudioEffect(options)).get();

    for (auto const& frameSource : g_mediaCapture.FrameSources())
    {
    auto const& frameSourceInfo = frameSource.Value().Info();
    if (frameSourceInfo.MediaStreamType() != MediaStreamType::Audio) { continue; }
    g_audioSource = frameSource.Value();
    }
}

// OK
void ExtendedAudio_Close()
{
    g_audioSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
}

// OK
MediaFrameReader ExtendedAudio_CreateFrameReader()
{
    return g_mediaCapture.CreateFrameReaderAsync(g_audioSource).get();
}
