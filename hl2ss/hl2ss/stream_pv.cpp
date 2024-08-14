
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "personal_video.h"
#include "locator.h"
#include "log.h"
#include "ports.h"
#include "timestamps.h"
#include "ipc_sc.h"
#include "research_mode.h"
#include "extended_execution.h"
#include "nfo.h"

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
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Perception::Spatial;

struct uint64x2
{
    uint64_t val[2];
};

struct float7
{
    float val[7];
};

struct PV_Projection
{
    float2   f;
    float2   c;
    uint64_t exposure_time;
    uint64x2 exposure_compensation;
    uint32_t lens_position;
    uint32_t focus_state;
    uint32_t iso_speed;
    uint32_t white_balance;
    float2   iso_gains;
    float3   white_balance_gains;
    uint32_t _reserved;
    uint64_t timestamp;
    float4x4 pose;
};

struct PV_Mode2
{
    float2   f;
    float2   c;
    float3   r;
    float2   t;
    float4x4 p;
    float4x4 extrinsics;
    float4   intrinsics_mf;
    float7   extrinsics_mf;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle
static SRWLOCK g_lock;

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;
static uint32_t g_counter = 0;
static uint32_t g_divisor = 1;
static PV_Projection g_pvp_sh;

// Mode: 2
static HANDLE g_event_intrinsic = NULL; // alias
static PV_Mode2 g_calibration;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<bool ENABLE_LOCATION>
void PV_OnVideoFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    
    IMFSample* pSample; // Release
    SoftwareBitmapBuffer* pBuffer; // Release
    PV_Projection pj;

    if (TryAcquireSRWLockShared(&g_lock) != 0)
    {
    auto const& frame = sender.TryAcquireLatestFrame();
    if (frame) 
    {
    if (g_counter == 0)
    {
    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    int64_t timestamp = frame.SystemRelativeTime().Value().count();

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    auto const& intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    auto const& metadata   = frame.Properties().Lookup(MFSampleExtension_CaptureMetadata).as<IMapView<winrt::guid, winrt::Windows::Foundation::IInspectable>>();

    pj.timestamp = timestamp;

    pj.f = intrinsics.FocalLength();
    pj.c = intrinsics.PrincipalPoint();

    pj.exposure_time         = metadata.Lookup(MF_CAPTURE_METADATA_EXPOSURE_TIME).as<uint64_t>();
    pj.exposure_compensation = *(uint64x2*)metadata.Lookup(MF_CAPTURE_METADATA_EXPOSURE_COMPENSATION).as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>().Value().begin();
    pj.iso_speed             = metadata.Lookup(MF_CAPTURE_METADATA_ISO_SPEED).as<uint32_t>();
    pj.iso_gains             = *(float2*)metadata.Lookup(MF_CAPTURE_METADATA_ISO_GAINS).as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>().Value().begin();
    pj.lens_position         = metadata.Lookup(MF_CAPTURE_METADATA_LENS_POSITION).as<uint32_t>();
    pj.focus_state           = metadata.Lookup(MF_CAPTURE_METADATA_FOCUSSTATE).as<uint32_t>();
    pj.white_balance         = metadata.Lookup(MF_CAPTURE_METADATA_WHITEBALANCE).as<uint32_t>();
    pj.white_balance_gains   = *(float3*)metadata.Lookup(MF_CAPTURE_METADATA_WHITEBALANCE_GAINS).as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>().Value().begin();

    if constexpr (ENABLE_LOCATION)
    {
    pj.pose = Locator_GetTransformTo(frame.CoordinateSystem(), Locator_GetWorldCoordinateSystem(QPCTimestampToPerceptionTimestamp(timestamp)));
    }

    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pj, sizeof(pj));

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
    }
    g_counter = (g_counter + 1) % g_divisor;
    }
    ReleaseSRWLockShared(&g_lock);
    }
}

// OK
static void PV_OnVideoFrameArrived_Intrinsics(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    if (TryAcquireSRWLockExclusive(&g_lock) != 0)
    {
    if (WaitForSingleObject(g_event_intrinsic, 0) == WAIT_TIMEOUT)
    {
    auto const& frame = sender.TryAcquireLatestFrame();
    if (frame) 
    {    
    auto const& intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    auto const& extrinsics = frame.Properties().Lookup(MFSampleExtension_CameraExtrinsics).as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>().Value();
    auto const& additional = frame.Format().Properties().Lookup(winrt::guid("86b6adbb-3735-447d-bee5-6fc23cb58d4a")).as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>().Value();

    g_calibration.f = intrinsics.FocalLength();
    g_calibration.c = intrinsics.PrincipalPoint();
    g_calibration.r = intrinsics.RadialDistortion();
    g_calibration.t = intrinsics.TangentialDistortion();
    g_calibration.p = intrinsics.UndistortedProjectionTransform();

    g_calibration.extrinsics = Locator_Locate(QPCTimestampToPerceptionTimestamp(frame.SystemRelativeTime().Value().count()), ResearchMode_GetLocator(), frame.CoordinateSystem());

    g_calibration.intrinsics_mf = *(float4*)&((float*)additional.begin())[3];
    g_calibration.extrinsics_mf = *(float7*)&((float*)extrinsics.begin())[5];

    SetEvent(g_event_intrinsic);
    }
    }
    ReleaseSRWLockExclusive(&g_lock);
    }
}

// OK
template<bool ENABLE_LOCATION>
void PV_SendSample(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    WSABUF wsaBuf[ENABLE_LOCATION ? 5 : 4];

    HookCallbackSocket* user = (HookCallbackSocket*)param;
    H26xFormat* format = (H26xFormat*)user->format;
    bool sh = format->profile != H26xProfile::H26xProfile_None;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);
    
    if (!sh) { pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&g_pvp_sh, sizeof(g_pvp_sh), NULL); }

    pBuffer->Lock(&pBytes, NULL, &cbData);

    int const metadata = sizeof(g_pvp_sh) - sizeof(g_pvp_sh.timestamp) - sizeof(g_pvp_sh.pose);
    DWORD cbDataEx = cbData + metadata;

    pack_buffer(wsaBuf, 0, &g_pvp_sh.timestamp, sizeof(g_pvp_sh.timestamp));
    pack_buffer(wsaBuf, 1, &cbDataEx, sizeof(cbDataEx));
    pack_buffer(wsaBuf, 2, pBytes, cbData);
    pack_buffer(wsaBuf, 3, &g_pvp_sh, metadata);

    if constexpr(ENABLE_LOCATION)
    {
    pack_buffer(wsaBuf, 4, &g_pvp_sh.pose, sizeof(g_pvp_sh.pose));
    }
    
    bool ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();

    if (sh) { pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&g_pvp_sh, sizeof(g_pvp_sh), NULL); }
}

// OK
template<bool ENABLE_LOCATION>
void PV_Stream(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader, H26xFormat& format)
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

    CreateSinkWriterVideo(&pSink, &g_pSinkWriter, &g_dwVideoIndex, VideoSubtype::VideoSubtype_NV12, format, options, PV_SendSample<ENABLE_LOCATION>, &user);

    reader.FrameArrived(PV_OnVideoFrameArrived<ENABLE_LOCATION>);

    g_counter = 0;
    g_divisor = format.divisor;    
    memset(&g_pvp_sh, 0, sizeof(g_pvp_sh));

    ReleaseSRWLockExclusive(&g_lock);
    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    reader.StopAsync().get();
    AcquireSRWLockExclusive(&g_lock);
    
    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();
}

// OK
static void PV_Intrinsics(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader)
{
    WSABUF wsaBuf[1];

    g_event_intrinsic = clientevent;

    reader.FrameArrived(PV_OnVideoFrameArrived_Intrinsics);

    ReleaseSRWLockExclusive(&g_lock);
    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    reader.StopAsync().get();
    AcquireSRWLockExclusive(&g_lock);

    pack_buffer(wsaBuf, 0, &g_calibration, sizeof(g_calibration));

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void PV_Stream(SOCKET clientsocket)
{
    HANDLE clientevent; // CloseHandle
    MRCVideoOptions options;
    H26xFormat format;
    uint8_t mode;    
    bool ok;

    ok = recv_u8(clientsocket, mode);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Video(clientsocket, format);
    if (!ok) { return; }

    if (mode & 4)
    {
    ok = ReceiveMRCVideoOptions(clientsocket, options);
    if (!ok) { return; }
    if (PersonalVideo_Status()) { PersonalVideo_Close(); }
    PersonalVideo_Open(options);
    }

    if (!PersonalVideo_Status()) { return; }

    ok = PersonalVideo_SetFormat(format.width, format.height, format.framerate);
    if (!ok) { return; }
    
    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    PersonalVideo_RegisterEvent(clientevent);
    auto const& videoFrameReader = PersonalVideo_CreateFrameReader();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);
    
    switch (mode & 3)
    {
    case 0: PV_Stream<false>(clientsocket, clientevent, videoFrameReader, format); break;
    case 1: PV_Stream<true>( clientsocket, clientevent, videoFrameReader, format); break;
    case 2: PV_Intrinsics(   clientsocket, clientevent, videoFrameReader);         break;
    }

    videoFrameReader.Close();
    PersonalVideo_RegisterEvent(NULL);

    CloseHandle(clientevent);

    if (mode & 8) { PersonalVideo_Close(); }
}

// OK
static DWORD WINAPI PV_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket
    int base_priority;

    listensocket = CreateSocket(PORT_NAME_PV);

    ShowMessage("PV: Listening at port %s", PORT_NAME_PV);

    AcquireSRWLockExclusive(&g_lock);

    base_priority = GetThreadPriority(GetCurrentThread());

    do
    {
    ShowMessage("PV: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("PV: Client connected");

    SetThreadPriority(GetCurrentThread(), ExtendedExecution_GetInterfacePriority(PORT_NUMBER_PV - PORT_NUMBER_BASE));

    PV_Stream(clientsocket);

    SetThreadPriority(GetCurrentThread(), base_priority);

    closesocket(clientsocket);

    ShowMessage("PV: Client disconnected");
    } 
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("PV: Closed");

    return 0;
}

// OK
void PV_Initialize()
{
    InitializeSRWLock(&g_lock);
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, PV_EntryPoint, NULL, 0, NULL);
}

// OK
void PV_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void PV_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_quit);
    
    g_thread = NULL;
    g_event_quit = NULL;
}
