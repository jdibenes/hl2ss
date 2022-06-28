
#include <queue>
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "personal_video.h"
#include "locator.h"
#include "utilities.h"
#include "ports.h"

#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_quitevent = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;

// Mode: 1
static SpatialCoordinateSystem g_world = nullptr;

// Mode: 2
static HANDLE g_intrinsic_event = NULL; // alias
static float g_intrinsics[2 + 2 + 3 + 2 + 16];

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<bool ENABLE_LOCATION>
void PV_OnVideoFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    
    MediaFrameReference frame = sender.TryAcquireLatestFrame();
    IMFSample* pSample; // Release
    SoftwareBitmapBuffer* pBuffer; // Release
    float4x4 pose;

    if (!frame) { return; }

    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(frame.SystemRelativeTime().Value().count());

    if constexpr(ENABLE_LOCATION)
    {
    pose = Locator_GetTransformTo(frame.CoordinateSystem(), g_world);
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(float4x4));
    }

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
}

// OK
static void PV_OnVideoFrameArrived_Instrinsics(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    MediaFrameReference frame = sender.TryAcquireLatestFrame();
    float2 f;
    float2 c;
    float3 r;
    float2 t;
    float4x4 p;
    DWORD status;

    if (!frame) { return; }

    status = WaitForSingleObject(g_intrinsic_event, 0);
    if (status != WAIT_TIMEOUT) { return; }

    auto intrinsics = frame.VideoMediaFrame().CameraIntrinsics();

    f = intrinsics.FocalLength();
    c = intrinsics.PrincipalPoint();
    r = intrinsics.RadialDistortion();
    t = intrinsics.TangentialDistortion();
    p = intrinsics.UndistortedProjectionTransform();

    memcpy(&g_intrinsics[0], &f, sizeof(f));
    memcpy(&g_intrinsics[2], &c, sizeof(c));
    memcpy(&g_intrinsics[4], &r, sizeof(r));
    memcpy(&g_intrinsics[7], &t, sizeof(t));
    memcpy(&g_intrinsics[9], &p, sizeof(p));

    SetEvent(g_intrinsic_event);
}

// OK
template<bool ENABLE_LOCATION>
void PV_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    float4x4 pose;
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];
    HookCallbackSocket* user;
    bool ok;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);

    pBuffer->Lock(&pBytes, NULL, &cbData);

    wsaBuf[0].buf = (char*)&sampletime;
    wsaBuf[0].len = sizeof(sampletime);

    wsaBuf[1].buf = (char*)&cbData;
    wsaBuf[1].len = sizeof(cbData);

    wsaBuf[2].buf = (char*)pBytes;
    wsaBuf[2].len = cbData;

    if constexpr(ENABLE_LOCATION)
    {    
    pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(pose), NULL);

    wsaBuf[3].buf = (char*)&pose;
    wsaBuf[3].len = sizeof(pose);
    }
    
    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();

    pBuffer->Release();
}

// OK
template<bool ENABLE_LOCATION>
void PV_Stream(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat const& format)
{
    HookCallbackSocket user;

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;

    CreateSinkWriterNV12ToH26x(&g_pSinkWriter, &g_dwVideoIndex, format, PV_SendSampleToSocket<ENABLE_LOCATION>, &user);

    reader.FrameArrived(PV_OnVideoFrameArrived<ENABLE_LOCATION>);

    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    reader.StopAsync().get();

    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();

    g_pSinkWriter = NULL;
    g_dwVideoIndex = 0;
}

// OK
static void PV_Intrinsics(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader)
{
    WSABUF data[1];

    g_intrinsic_event = clientevent;

    reader.FrameArrived(PV_OnVideoFrameArrived_Instrinsics);

    reader.StartAsync().get();
    WaitForSingleObject(g_intrinsic_event, INFINITE);
    reader.StopAsync().get();

    data[0].buf = (char*)g_intrinsics;
    data[0].len = sizeof(g_intrinsics);

    send_multiple(clientsocket, data, sizeof(data) / sizeof(WSABUF));

    g_intrinsic_event = NULL;
    memset(g_intrinsics, 0, sizeof(g_intrinsics));
}

// OK
static void PV_Stream(MediaCapture const& mediaCapture, MediaFrameSource const& videoSource, SOCKET clientsocket)
{    
    MediaFrameReader videoFrameReader = nullptr;
    HANDLE clientevent; // CloseHandle
    H26xFormat format;
    uint8_t mode;    
    bool ok;

    ok = recv_u8(clientsocket, mode);
    if (!ok) { return; }

    ok = ReceiveVideoFormatH26x(clientsocket, format);
    if (!ok) { return; }

    ok = PersonalVideo_SetFormat(videoSource, format.width, format.height, format.framerate);
    if (!ok) { return; }
    
    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    videoFrameReader = mediaCapture.CreateFrameReaderAsync(videoSource).get();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);

    switch (mode)
    {
    case 0: PV_Stream<false>(clientsocket, clientevent, videoFrameReader, format); break;
    case 1: PV_Stream<true>( clientsocket, clientevent, videoFrameReader, format); break;
    case 2: PV_Intrinsics(   clientsocket, clientevent, videoFrameReader);         break;
    }

    CloseHandle(clientevent);
}

// OK
static DWORD WINAPI PV_EntryPoint(void *)
{      
    MediaFrameSource videoSource = nullptr;
    MediaCapture mediaCapture;
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    PersonalVideo_Initialize(mediaCapture, videoSource);

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
    g_world = world;
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
