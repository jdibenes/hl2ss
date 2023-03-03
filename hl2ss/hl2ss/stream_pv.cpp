
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "personal_video.h"
#include "locator.h"
#include "log.h"
#include "ports.h"
#include "timestamps.h"
#include "ipc_sc.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

struct _PV_Projection
{
    float2 f;
    float2 c;
    float4x4 pose;
};

typedef struct _PV_Projection PV_Projection;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_quitevent = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;

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
    
    CameraIntrinsics intrinsics = nullptr;
    MediaFrameReference frame = nullptr;
    IMFSample* pSample; // Release
    SoftwareBitmapBuffer* pBuffer; // Release
    PV_Projection pj;
    int64_t timestamp;

    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    timestamp = frame.SystemRelativeTime().Value().count();

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    intrinsics = frame.VideoMediaFrame().CameraIntrinsics();

    pj.f = intrinsics.FocalLength();
    pj.c = intrinsics.PrincipalPoint();

    if constexpr(ENABLE_LOCATION)
    {
    pj.pose = Locator_GetTransformTo(frame.CoordinateSystem(), Locator_GetWorldCoordinateSystem(QPCTimestampToPerceptionTimestamp(timestamp)));
    }

    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pj, sizeof(pj));

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
}

// OK
static void PV_OnVideoFrameArrived_Intrinsics(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    CameraIntrinsics intrinsics = nullptr;
    MediaFrameReference frame = nullptr;
    float2 f;
    float2 c;
    float3 r;
    float2 t;
    float4x4 p;
    DWORD status;

    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    status = WaitForSingleObject(g_intrinsic_event, 0);
    if (status != WAIT_TIMEOUT) { return; }

    intrinsics = frame.VideoMediaFrame().CameraIntrinsics();

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
    DWORD cbDataEx;
    PV_Projection pj;
    WSABUF wsaBuf[ENABLE_LOCATION ? 6 : 5];
    HookCallbackSocket* user;
    bool ok;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);
    pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pj, sizeof(pj), NULL);

    pBuffer->Lock(&pBytes, NULL, &cbData);

    cbDataEx = cbData + sizeof(pj.f) + sizeof(pj.c);

    wsaBuf[0].buf = (char*)&sampletime;
    wsaBuf[0].len = sizeof(sampletime);

    wsaBuf[1].buf = (char*)&cbDataEx;
    wsaBuf[1].len = sizeof(cbDataEx);

    wsaBuf[2].buf = (char*)pBytes;
    wsaBuf[2].len = cbData;
      
    wsaBuf[3].buf = (char*)&pj.f;
    wsaBuf[3].len = sizeof(pj.f);

    wsaBuf[4].buf = (char*)&pj.c;
    wsaBuf[4].len = sizeof(pj.c);

    if constexpr(ENABLE_LOCATION)
    {
    wsaBuf[5].buf = (char*)&pj.pose;
    wsaBuf[5].len = sizeof(pj.pose);
    }
    
    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
template<bool ENABLE_LOCATION>
void PV_Stream(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat& format)
{
    CustomMediaSink* pSink; // Release
    HookCallbackSocket user;
    bool ok;

    ok = ReceiveVideoH26x(clientsocket, format);
    if (!ok) { return; }

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;
    user.data_profile = format.profile;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: CreateSinkWriterNV12ToNV12(&pSink, &g_pSinkWriter, &g_dwVideoIndex, format, PV_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            CreateSinkWriterNV12ToH26x(&pSink, &g_pSinkWriter, &g_dwVideoIndex, format, PV_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    reader.FrameArrived(PV_OnVideoFrameArrived<ENABLE_LOCATION>);

    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    reader.StopAsync().get();

    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    g_pSinkWriter = NULL;
    g_dwVideoIndex = 0;
}

// OK
static void PV_Intrinsics(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader)
{
    WSABUF wsaBuf;

    g_intrinsic_event = clientevent;

    reader.FrameArrived(PV_OnVideoFrameArrived_Intrinsics);

    reader.StartAsync().get();
    WaitForSingleObject(g_intrinsic_event, INFINITE);
    reader.StopAsync().get();

    wsaBuf.buf = (char*)g_intrinsics;
    wsaBuf.len = sizeof(g_intrinsics);

    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    g_intrinsic_event = NULL;
    memset(g_intrinsics, 0, sizeof(g_intrinsics));
}

// OK
static void PV_Stream(SOCKET clientsocket)
{
    MediaFrameReader videoFrameReader = nullptr;
    HANDLE clientevent; // CloseHandle
    H26xFormat format;
    uint8_t mode;    
    bool ok;

    ok = recv_u8(clientsocket, mode);
    if (!ok) { return; }

    if (!PersonalVideo_Status() && (mode & 4)) { PersonalVideo_Open(); }
    if (!PersonalVideo_Status()) { return; }
    
    ok = ReceiveVideoFormat(clientsocket, format);
    if (!ok) { return; }

    ok = PersonalVideo_SetFormat(format.width, format.height, format.framerate);
    if (!ok) { return; }
    
    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    videoFrameReader = PersonalVideo_CreateFrameReader();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);

    switch (mode & 3)
    {
    case 0: PV_Stream<false>(clientsocket, clientevent, videoFrameReader, format); break;
    case 1: PV_Stream<true>( clientsocket, clientevent, videoFrameReader, format); break;
    case 2: PV_Intrinsics(   clientsocket, clientevent, videoFrameReader);         break;
    }

    videoFrameReader.Close();

    CloseHandle(clientevent);

    if (mode & 8) { PersonalVideo_Close(); }
}

// OK
static DWORD WINAPI PV_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    listensocket = CreateSocket(PORT_NAME_PV);

    ShowMessage("PV: Listening at port %s", PORT_NAME_PV);

    do
    {
    ShowMessage("PV: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("PV: Client connected");

    PV_Stream(clientsocket);

    closesocket(clientsocket);

    ShowMessage("PV: Client disconnected");
    } 
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("PV: Closed");

    return 0;
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
