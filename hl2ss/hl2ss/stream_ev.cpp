
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "extended_video.h"
#include "log.h"
#include "ports.h"
#include "timestamps.h"
#include "ipc_sc.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;
using namespace winrt::Windows::Foundation::Numerics;

struct PV_Projection
{
    float2 f;
    float2 c;
    float4x4 pose;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle
static bool g_reader_status = false;

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;
static uint32_t g_counter = 0;
static uint32_t g_divisor = 1;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void EV_OnVideoFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    
    CameraIntrinsics intrinsics = nullptr;
    MediaFrameReference frame = nullptr;
    IMFSample* pSample; // Release
    SoftwareBitmapBuffer* pBuffer; // Release
    int64_t timestamp;

    if (!g_reader_status) { return; }
    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    if (g_counter == 0)
    {
    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    timestamp = frame.SystemRelativeTime().Value().count();

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
    }

    g_counter = (g_counter + 1) % g_divisor;
}

// OK
template<bool ENABLE_LOCATION>
void EV_SendSample(IMFSample* pSample, void* param)
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
    memset(&pj, 0, sizeof(pj));

    pBuffer->Lock(&pBytes, NULL, &cbData);

    cbDataEx = cbData + sizeof(pj.f) + sizeof(pj.c);

    pack_buffer(wsaBuf, 0, &sampletime, sizeof(sampletime));
    pack_buffer(wsaBuf, 1, &cbDataEx, sizeof(cbDataEx));
    pack_buffer(wsaBuf, 2, pBytes, cbData);
    pack_buffer(wsaBuf, 3, &pj.f, sizeof(pj.f));
    pack_buffer(wsaBuf, 4, &pj.c, sizeof(pj.c));

    if constexpr (ENABLE_LOCATION)
    {
    pack_buffer(wsaBuf, 5, &pj.pose, sizeof(pj.pose));
    }

    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
template<bool ENABLE_LOCATION>
void EV_Stream(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat& format, VideoSubtype subtype)
{
    CustomMediaSink* pSink; // Release
    std::vector<uint64_t> options;
    HookCallbackSocket user;
    bool ok;

    ok = ReceiveH26xFormat_Divisor(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveH26xEncoder_Options(clientsocket, options);
    if (!ok) { return; }

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;
    user.format       = &format;

    CreateSinkWriterVideo(&pSink, &g_pSinkWriter, &g_dwVideoIndex, subtype, format, options, EV_SendSample<ENABLE_LOCATION>, &user);

    reader.FrameArrived(EV_OnVideoFrameArrived);

    g_counter = 0;
    g_divisor = format.divisor;

    g_reader_status = true;
    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    g_reader_status = false;
    reader.StopAsync().get();

    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();
}

// OK
static void EV_DeviceQuery(SOCKET clientsocket)
{
    winrt::hstring query;
    uint32_t bytes;
    WSABUF wsaBuf[2];

    ExtendedVideo_QueryDevices(query);

    bytes = query.size() * sizeof(wchar_t);

    pack_buffer(wsaBuf, 0, &bytes, sizeof(bytes));
    pack_buffer(wsaBuf, 1, query.c_str(), bytes);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void EV_Stream(SOCKET clientsocket, uint8_t mode, H26xFormat& format)
{
    MediaFrameReader videoFrameReader = nullptr;
    HANDLE clientevent; // CloseHandle
    VideoSubtype subtype;
    bool ok;

    ok = ExtendedVideo_SetFormat(format.width, format.height, format.framerate, subtype);
    if (!ok) { return; }

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ExtendedVideo_RegisterEvent(clientevent);
    videoFrameReader = ExtendedVideo_CreateFrameReader();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);

    switch (mode & 3)
    {
    case 0: EV_Stream<false>(clientsocket, clientevent, videoFrameReader, format, subtype); break;
    case 1: EV_Stream<true>( clientsocket, clientevent, videoFrameReader, format, subtype); break;
    }

    videoFrameReader.Close();
    ExtendedVideo_RegisterEvent(NULL);

    CloseHandle(clientevent);
}

// OK
static void EV_Stream(SOCKET clientsocket)
{
    H26xFormat format;
    uint8_t mode;    
    bool ok;

    ok = recv_u8(clientsocket, mode);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Video(clientsocket, format);
    if (!ok) { return; }

    if ((mode & 3) == 2)
    {
    EV_DeviceQuery(clientsocket);
    return;
    }

    if (mode & 4)
    {
    MRCVideoOptions options;
    ok = ReceiveMRCVideoOptions(clientsocket, options);
    if (!ok) { return; }
    if (ExtendedVideo_Status()) { ExtendedVideo_Close(); }
    ExtendedVideo_Open(options);
    }

    if (!ExtendedVideo_Status()) { return; }

    if ((mode & 3) != 3) { EV_Stream(clientsocket, mode, format); }

    if (mode & 8) { ExtendedVideo_Close(); }
}

// OK
static DWORD WINAPI EV_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    listensocket = CreateSocket(PORT_NAME_EV);

    ShowMessage("EV: Listening at port %s", PORT_NAME_EV);

    do
    {
    ShowMessage("EV: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("EV: Client connected");

    EV_Stream(clientsocket);

    closesocket(clientsocket);

    ShowMessage("EV: Client disconnected");
    } 
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("EV: Closed");

    return 0;
}

// OK
void EV_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, EV_EntryPoint, NULL, 0, NULL);
}

// OK
void EV_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void EV_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_quit);
    
    g_thread = NULL;
    g_event_quit = NULL;
}
