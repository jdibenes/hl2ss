
#include <queue>
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffer.h"
#include "ports.h"
#include "utilities.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Devices.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_quitevent = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle

// Mode: 2
static HANDLE g_intrinsic_event = NULL; // CloseHandle
static float g_intrinsics[2 + 2 + 3 + 2 + 16];

// Mode: 1
static CRITICAL_SECTION g_location_cs; // DeleteCriticalSection
static std::queue<float4x4> g_location_queue;
static SpatialCoordinateSystem g_world_frame = nullptr;

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;
static HookCallbackSocketData g_sinkParam = { INVALID_SOCKET, NULL };

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static bool PV_FindMediaSourceGroup(uint32_t width, uint32_t height, double framerate, MediaFrameSourceGroup &sourceGroup, MediaCaptureVideoProfile &profile, MediaCaptureVideoProfileMediaDescription &description)
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
static bool PV_FindVideoSource(MediaCapture const& mediaCapture, MediaFrameSource& videoSource)
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
static bool PV_FindVideoFormat(MediaFrameSource const& videoSource, uint32_t width, uint32_t height, uint32_t frameRate, MediaFrameFormat& selectedFormat)
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
static void PV_InitializeSinkWriter(SOCKET clientsocket, HANDLE clientevent, H26xFormat const& format, HOOK_SINK_PROC callback)
{
    g_sinkParam.clientsocket = clientsocket;
    g_sinkParam.clientevent = clientevent;
    CreateSinkWriterNV12ToH26x(&g_pSinkWriter, &g_dwVideoIndex, format, callback, &g_sinkParam);
}

// OK
static void PV_CleanupSinkWriter()
{
    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();

    g_pSinkWriter = NULL;
    g_dwVideoIndex = 0;
    g_sinkParam.clientsocket = INVALID_SOCKET;
    g_sinkParam.clientevent = NULL;
}

// OK
static void PV_PushFrame_old(MediaFrameReference const& frame)
{
    SoftwareBitmapBuffer* pBuffer; // Release
    IMFSample* pSample; // Release

    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);
    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(frame.SystemRelativeTime().Value().count());

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
}

static void PV_PushFrame(MediaFrameReference const& frame)
{
    WrappedBuffer* pBuffer; // Release
    IMFSample* pSample; // Release

    WrappedBuffer::CreateInstance(&pBuffer, frame.BufferMediaFrame().Buffer().data(), frame.BufferMediaFrame().Buffer().Length());
    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(frame.SystemRelativeTime().Value().count());

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
}



// OK
static void PV_FrameReaderSequence(MediaFrameReader const& reader, HANDLE clientevent, void(*callback)(MediaFrameReader const&, MediaFrameArrivedEventArgs const&))
{
    reader.FrameArrived(callback);
    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    reader.StopAsync().get();
}

// OK
static void PV_OnVideoFrameArrived_Mode2(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    MediaFrameReference frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    if (WaitForSingleObject(g_intrinsic_event, 0) != WAIT_TIMEOUT) { return; }

    float2   f = frame.VideoMediaFrame().CameraIntrinsics().FocalLength();
    float2   c = frame.VideoMediaFrame().CameraIntrinsics().PrincipalPoint();
    float3   r = frame.VideoMediaFrame().CameraIntrinsics().RadialDistortion();
    float2   t = frame.VideoMediaFrame().CameraIntrinsics().TangentialDistortion();;
    float4x4 p = frame.VideoMediaFrame().CameraIntrinsics().UndistortedProjectionTransform();

    memcpy(&g_intrinsics[0], &f, sizeof(f));
    memcpy(&g_intrinsics[2], &c, sizeof(c));
    memcpy(&g_intrinsics[4], &r, sizeof(r));
    memcpy(&g_intrinsics[7], &t, sizeof(t));
    memcpy(&g_intrinsics[9], &p, sizeof(p));

    SetEvent(g_intrinsic_event);
}

// OK
static void PV_OnVideoFrameArrived_Mode1(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    MediaFrameReference frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    auto location = frame.CoordinateSystem().TryGetTransformTo(g_world_frame);
    {
    CriticalSection cs(&g_location_cs);
    if (location) { g_location_queue.push(location.Value()); } else { g_location_queue.push(float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)); }
    }

    PV_PushFrame(frame);
}

// OK
static void PV_OnVideoFrameArrived_Mode0(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    MediaFrameReference frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    PV_PushFrame(frame);
}

// OK
static void PV_SendSampleToSocket_Mode1(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    float4x4 location;
    WSABUF wsaBuf[4];
    HookCallbackSocketData* user;
    bool ok;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);

    {
    CriticalSection cs(&g_location_cs);
    location = g_location_queue.front();
    g_location_queue.pop();
    }

    pBuffer->Lock(&pBytes, NULL, &cbData);

    wsaBuf[0].buf = (char*)&sampletime; wsaBuf[0].len = sizeof(sampletime);
    wsaBuf[1].buf = (char*)&cbData;     wsaBuf[1].len = sizeof(cbData);
    wsaBuf[2].buf = (char*)pBytes;      wsaBuf[2].len = cbData;
    wsaBuf[3].buf = (char*)&location;   wsaBuf[3].len = sizeof(location);

    user = (HookCallbackSocketData*)param;
    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();

    pBuffer->Release();
}

// OK
static void PV_Stream_Mode2(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader)
{
    WSABUF data[1];

    g_intrinsic_event = clientevent;
    PV_FrameReaderSequence(reader, clientevent, PV_OnVideoFrameArrived_Mode2);

    data[0].buf = (char*)g_intrinsics; data[0].len = sizeof(g_intrinsics);

    send_multiple(clientsocket, data, sizeof(data) / sizeof(WSABUF));

    g_intrinsic_event = NULL;
    memset(g_intrinsics, 0, sizeof(g_intrinsics));
}

// OK
static void PV_Stream_Mode1(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat const &format)
{
    InitializeCriticalSection(&g_location_cs);
    PV_InitializeSinkWriter(clientsocket, clientevent, format, PV_SendSampleToSocket_Mode1);
    PV_FrameReaderSequence(reader, clientevent, PV_OnVideoFrameArrived_Mode1);
    PV_CleanupSinkWriter();
    DeleteCriticalSection(&g_location_cs);

    ZeroMemory(&g_location_cs, sizeof(g_location_cs));
    while (!g_location_queue.empty()) { g_location_queue.pop(); }
}

// OK
static void PV_Stream_Mode0(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat const& format)
{
    PV_InitializeSinkWriter(clientsocket, clientevent, format, SendSampleToSocket);
    PV_FrameReaderSequence(reader, clientevent, PV_OnVideoFrameArrived_Mode0);
    PV_CleanupSinkWriter();
}

// OK
static void PV_Stream(MediaCapture const& mediaCapture, MediaFrameSource const& videoSource, SOCKET clientsocket)
{
    MediaFrameFormat selectedFormat = nullptr;
    MediaFrameReader videoFrameReader = nullptr;
    HANDLE clientevent; // CloseHandle
    H26xFormat format;
    PVOperatingMode pvmode;    
    bool ok;

    ok = ReceiveVideoFormatH26x(clientsocket, format);
    if (!ok) { return; }

    ok = PV_FindVideoFormat(videoSource, format.width, format.height, format.framerate, selectedFormat);
    if (!ok) { return; }

    ok = ReceivePVOperatingMode(clientsocket, pvmode);
    if (!ok) { return; }

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    videoSource.SetFormatAsync(selectedFormat).get();
    videoFrameReader = mediaCapture.CreateFrameReaderAsync(videoSource).get();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);

    switch (pvmode)
    {
    case PVOperatingMode::PVOperatingMode_Video:            PV_Stream_Mode0(clientsocket, clientevent, videoFrameReader, format); break;
    case PVOperatingMode::PVOperatingMode_VideoAndLocation: PV_Stream_Mode1(clientsocket, clientevent, videoFrameReader, format); break;
    case PVOperatingMode::PVOperatingMode_Calibration:      PV_Stream_Mode2(clientsocket, clientevent, videoFrameReader);         break;
    }

    CloseHandle(clientevent);
}

// OK
static DWORD WINAPI PV_EntryPoint(void *)
{
    uint32_t const width = 1920;
    uint32_t const height = 1080;
    double const framerate = 30;

    MediaFrameSourceGroup sourceGroup = nullptr;
    MediaCaptureVideoProfile profile = nullptr;
    MediaCaptureVideoProfileMediaDescription description = nullptr;
    MediaFrameSource videoSource = nullptr;
    MediaCaptureInitializationSettings settings;
    MediaCapture mediaCapture;
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    PV_FindMediaSourceGroup(width, height, framerate, sourceGroup, profile, description);    

    settings.VideoProfile(profile);
    settings.RecordMediaDescription(description);
    settings.VideoDeviceId(sourceGroup.Id());
    settings.StreamingCaptureMode(StreamingCaptureMode::Video);
    settings.MemoryPreference(MediaCaptureMemoryPreference::Cpu);
    settings.SharingMode(MediaCaptureSharingMode::ExclusiveControl);
    settings.SourceGroup(sourceGroup);
    settings.MediaCategory(MediaCategory::Media);    

    mediaCapture.InitializeAsync(settings).get();

    PV_FindVideoSource(mediaCapture, videoSource);

    listensocket = CreateSocket(PORT_PV);
    ShowMessage("PV: Listening at port %s", PORT_PV);

    do
    {
    ShowMessage("PV: Waiting for client");
    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("PV: Client connected");
    PV_Stream(mediaCapture, videoSource, clientsocket);

    closesocket(clientsocket);
    ShowMessage("PV: Client disconnected");
    } 
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);
    ShowMessage("PV: Closed");

    return 0;
}

// OK
void PV_SetWorldFrame(SpatialCoordinateSystem const& world)
{
    g_world_frame = world;
}

// OK
void PV_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, PV_EntryPoint, NULL, 0, NULL);
}

// OK
void PV_Quit()
{
    SetEvent(g_quitevent);
}

// OK
void PV_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);
    
    g_thread = NULL;
    g_quitevent = NULL;
}
