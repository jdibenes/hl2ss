
#include "extended_execution.h"
#include "extended_audio.h"
#include "lock.h"
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Data.Json.h>

using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Data::Json;

struct EA_SourceFormat
{
    MediaFrameSource source;
    MediaFrameFormat format;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static MediaCapture g_mediaCapture = nullptr;
static MediaFrameSource g_audioSource = nullptr;
static bool g_ready = false;
static HANDLE g_event = NULL;
static HOOK_EA_PROC g_hook = nullptr;
static void* g_param = nullptr;

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
static void ExtendedAudio_OnFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    auto frame = sender.TryAcquireLatestFrame();
    if (frame) { g_hook(frame, g_param); }
}

// OK
static bool ExtendedAudio_ParseSubtype(winrt::hstring const& s, AudioSubtype& v)
{
    if (s == L"PCM")   { v = AudioSubtype::AudioSubtype_S16; return true; }
    if (s == L"Float") { v = AudioSubtype::AudioSubtype_F32; return true; }
    return false;
}

// OK
static bool ExtendedAudio_FindAudioSource()
{
    std::vector<EA_SourceFormat> sources[2][2];

    for (auto const& frameSource : g_mediaCapture.FrameSources())
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
    g_audioSource = sf.source;
    g_audioSource.SetFormatAsync(sf.format).get();
    return true;
    }
    }

    return false;
}

// OK
static bool ExtendedAudio_FindAudioSource(uint32_t source_index, uint32_t format_index)
{
    MediaFrameSource source = nullptr;
    MediaFrameFormat format = nullptr;

    for (auto const& frameSource : g_mediaCapture.FrameSources())
    {
    if (source_index-- != 0) { continue; }
    source = frameSource.Value();
    break;
    }

    if (!source) { return false; }

    if (source.Info().MediaStreamType() != MediaStreamType::Audio) { return false; }

    for (auto const& sourceFormat : source.SupportedFormats())
    {
    if (format_index-- != 0) { continue; }
    format = sourceFormat;
    break;
    }

    if (!format) { return false; }

    g_audioSource = source;
    g_audioSource.SetFormatAsync(format).get();

    return true;
}

// OK
static JsonObject ExtendedAudio_EnumerateAudioSources()
{
    JsonObject jfs = JsonObject();
    uint32_t source_index = 0;

    for (auto const& frameSource : g_mediaCapture.FrameSources())
    {
    auto source = frameSource.Value();
    auto sourceInfo = source.Info();

    JsonObject jsf = JsonObject();
    uint32_t format_index = 0;

    for (auto const& format : source.SupportedFormats())
    {
    auto aep = format.AudioEncodingProperties();

    JsonObject jaep = JsonObject();
    jaep.Insert(L"SampleRate", JsonValue::CreateNumberValue(aep.SampleRate()));
    jaep.Insert(L"ChannelCount", JsonValue::CreateNumberValue(aep.ChannelCount()));
    jaep.Insert(L"Subtype", JsonValue::CreateStringValue(aep.Subtype()));
    jaep.Insert(L"BitsPerSample", JsonValue::CreateNumberValue(aep.BitsPerSample()));
    jaep.Insert(L"Bitrate", JsonValue::CreateNumberValue(aep.Bitrate()));    
    jaep.Insert(L"IsSpatial", JsonValue::CreateBooleanValue(aep.IsSpatial()));

    jsf.Insert(winrt::to_hstring(format_index++), jaep);
    }

    JsonObject jsi = JsonObject();
    jsi.Insert(L"MediaStreamType", JsonValue::CreateNumberValue((int)sourceInfo.MediaStreamType()));
    jsi.Insert(L"MediaSourceKind", JsonValue::CreateNumberValue((int)sourceInfo.SourceKind()));
    jsi.Insert(L"Id", JsonValue::CreateStringValue(sourceInfo.Id()));
    jsi.Insert(L"SupportedFormats", jsf);

    jfs.Insert(winrt::to_hstring(source_index++), jsi);
    }

    return jfs;
}

// OK
winrt::hstring ExtendedAudio_QueryDevices(MRCAudioOptions const& options, ExtendedAudio_Control const& control)
{
    ExtendedAudio_Control control_index = control;

    std::vector<winrt::hstring> ids;
    std::vector<winrt::hstring> names;

    GetAudioCaptureIdsAndNames(ids, names);

    JsonObject root = JsonObject();

    for (uint32_t i = 0; i < ids.size(); ++i)
    {
    JsonObject jdeviceinfos = JsonObject();

    jdeviceinfos.Insert(L"Id",   JsonValue::CreateStringValue(ids[i]));
    jdeviceinfos.Insert(L"Name", JsonValue::CreateStringValue(names[i]));

    control_index.device_index = i + 1;

    ExtendedAudio_Open(options, control_index);
    if (ExtendedAudio_Status())
    {
    jdeviceinfos.Insert(L"FrameSources", ExtendedAudio_EnumerateAudioSources());
    ExtendedAudio_Close();
    }
    else
    {
    jdeviceinfos.Insert(L"FrameSources", JsonValue::CreateNullValue());
    }

    root.Insert(winrt::to_hstring(i), jdeviceinfos);
    }

    return root.ToString();
}

// OK
void ExtendedAudio_Open(MRCAudioOptions const& options, ExtendedAudio_Control const& control)
{
    uint32_t index = control.device_index;
    MediaCaptureInitializationSettings settings;
    std::vector<winrt::hstring> ids;
    winrt::hstring id;
    MediaCategory category;
    bool ok;

    if (index <= 0)
    {
    id = MediaDevice::GetDefaultAudioCaptureId(AudioDeviceRole::Default);
    }
    else
    {
    index--;
    GetAudioCaptureIds(ids);
    if (index >= ids.size()) { return; }
    id = ids[index];
    }

    switch (control.media_category)
    {
    case 0:  category = MediaCategory::Other;          break;
    case 1:  category = MediaCategory::Communications; break;
    case 2:  category = MediaCategory::Media;          break;
    case 3:  category = MediaCategory::GameChat;       break;
    case 4:  category = MediaCategory::Speech;         break;
    default: category = MediaCategory::Media;          break;
    }

    settings.AudioDeviceId(id);
    settings.StreamingCaptureMode(StreamingCaptureMode::Audio);
    settings.SharingMode(control.shared ? MediaCaptureSharingMode::SharedReadOnly : MediaCaptureSharingMode::ExclusiveControl);
    settings.MediaCategory(category);
    settings.AudioProcessing(control.audio_raw ? AudioProcessing::Raw : AudioProcessing::Default);   

    g_mediaCapture = MediaCapture();
    
    try
    {
    Cleaner log_error_microphone([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedMicrophone); });
    g_mediaCapture.InitializeAsync(settings).get();
    log_error_microphone.Set(false);
    }
    catch (...)
    {
    goto _fail_open;
    }

    if (!control.query)
    {
    ok = control.enable_passthrough ? ExtendedAudio_FindAudioSource(control.source_index, control.format_index) : ExtendedAudio_FindAudioSource();
    if (!ok) { goto _fail_find; }

    g_mediaCapture.Failed({ ExtendedAudio_OnFailed });
    if (!control.disable_effect) { g_mediaCapture.AddAudioEffectAsync(MRCAudioEffect(options)).get(); }
    }

    g_ready = true;

    return;

_fail_find:
    g_mediaCapture.Close();

_fail_open:
    g_mediaCapture = nullptr;
}

// OK
void ExtendedAudio_Close()
{
    g_ready = false;
    g_audioSource = nullptr;
    g_mediaCapture.Close();
    g_mediaCapture = nullptr;
}

// OK
bool ExtendedAudio_Status()
{
    return g_ready;
}

// OK
void ExtendedAudio_GetCurrentFormat(AudioSubtype& subtype, uint32_t& channels, uint32_t& samplerate)
{
    auto aep = g_audioSource.CurrentFormat().AudioEncodingProperties();
    ExtendedAudio_ParseSubtype(aep.Subtype(), subtype);
    channels = aep.ChannelCount();
    samplerate = aep.SampleRate();
}

// OK
void ExtendedAudio_ExecuteSensorLoop(HOOK_EA_PROC hook, void* param, HANDLE event_stop)
{
    g_hook  = hook;
    g_param = param;
    g_event = event_stop;
    
    auto reader = g_mediaCapture.CreateFrameReaderAsync(g_audioSource).get();

    reader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);
    reader.FrameArrived(ExtendedAudio_OnFrameArrived);

    reader.StartAsync().get();
    WaitForSingleObject(event_stop, INFINITE);
    reader.StopAsync().get();

    reader.Close();
    
    g_event = NULL;
}
