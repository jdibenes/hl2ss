
#include <mfapi.h>
#include <combaseapi.h>
#include "custom_media_types.h"
#include "custom_audio_effect.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Devices.Enumeration.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Data.Json.h>

using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Data::Json;

struct source_format
{
    MediaFrameSource source;
    MediaFrameFormat format;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event = NULL;

static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_audioSource = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedAudio_OnFailed(MediaCapture const&, MediaCaptureFailedEventArgs const& b)
{
    ShowMessage(L"ExtendedAudio_OnFailed - 0x%X : '%s'", b.Code(), b.Message().c_str());
    if (g_event != NULL) { SetEvent(g_event); }
}

// OK
static bool ExtendedAudio_ParseSubtype(winrt::hstring const& s, AudioSubtype& v)
{
    if (s == L"PCM")   { v = AudioSubtype::AudioSubtype_S16; return true; }
    if (s == L"Float") { v = AudioSubtype::AudioSubtype_F32; return true; }
    return false;
}

// OK
static bool ExtendedAudio_FindAudioSource(MediaCapture const& mediaCapture, MediaFrameSource& audioSource)
{
    std::vector<source_format> sources[2][2];

    for (auto const& frameSource : mediaCapture.FrameSources())
    {
    auto source = frameSource.Value();
    if (source.Info().MediaStreamType() != MediaStreamType::Audio) { continue; }
    for (auto const& format : source.SupportedFormats())
    {
    auto aep = format.AudioEncodingProperties();
    if (aep.SampleRate() != 48000) { continue; }
    AudioSubtype subtype;
    bool ok = ExtendedAudio_ParseSubtype(aep.Subtype(), subtype);
    if (!ok) { continue; }
    uint32_t channels = aep.ChannelCount();
    if ((channels != 1) && (channels != 2)) { continue; }
    sources[subtype][2 - channels].push_back({ source, format });
    }
    }

    for (int i = 0; i < 2; ++i)
    {
    for (int j = 0; j < 2; ++j)
    {
    if (sources[i][j].size() <= 0) { continue; }
    auto const& sf = sources[i][j][0];
    audioSource = sf.source;
    audioSource.SetFormatAsync(sf.format).get();
    return true;
    }
    }

    return false;
}

// OK
void ExtendedAudio_QueryDevices(winrt::hstring& out)
{
    std::vector<winrt::hstring> ids;
    std::vector<winrt::hstring> names;

    GetAudioCaptureIdsAndNames(ids, names);

    JsonObject root = JsonObject();
    for (uint32_t i = 0; i < ids.size(); ++i)
    {
    JsonObject jsourceinfos = JsonObject();
    jsourceinfos.Insert(L"Id", JsonValue::CreateStringValue(ids[i]));
    jsourceinfos.Insert(L"Name", JsonValue::CreateStringValue(names[i]));

    root.Insert(winrt::to_hstring(i), jsourceinfos);
    }

    out = root.ToString();
}

// OK
void ExtendedAudio_RegisterEvent(HANDLE h)
{
    g_event = h;
}

// OK
bool ExtendedAudio_Open(MRCAudioOptions const& options)
{
    uint32_t index = (options.mixer_mode & 0x7FFFFFFC) >> 2;
    MediaCaptureInitializationSettings settings;
    std::vector<winrt::hstring> ids;
    winrt::hstring id;
    bool ok;

    if (index <= 0)
    {
    id = MediaDevice::GetDefaultAudioCaptureId(AudioDeviceRole::Default);
    }
    else
    {
    index--;
    GetAudioCaptureIds(ids);
    if (index >= ids.size()) { return false; }
    id = ids[index];
    }

    settings.AudioDeviceId(id);
    settings.StreamingCaptureMode(StreamingCaptureMode::Audio);
    settings.SharingMode(MediaCaptureSharingMode::SharedReadOnly);
    settings.MediaCategory(MediaCategory::Media);

    g_mediaCapture = MediaCapture();
    g_mediaCapture.InitializeAsync(settings).get();
    g_mediaCapture.AddAudioEffectAsync(MRCAudioEffect(options)).get();

    g_mediaCapture.Failed({ ExtendedAudio_OnFailed });
    ok = ExtendedAudio_FindAudioSource(g_mediaCapture, g_audioSource);
    if (!ok)
    {
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
    return false;
    }

    return true;
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
