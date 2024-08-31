
#include <mfapi.h>
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"
#include "ipc_sc.h"
#include "log.h"
#include "types.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

struct VLC_Metadata
{
    uint64_t sensor_ticks;
    uint64_t exposure;
    uint32_t gain;
    uint32_t _reserved;
    uint64_t timestamp;
    float4x4 pose;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static VLC_Metadata g_pose_sh[4];

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RM_VLC_TranslateEncoderOptions(std::vector<uint64_t> const& options, double& exposure_factor, int64_t& constant_factor)
{
    exposure_factor = 0.0;
    constant_factor = 0;

    for (int i = 0; i < (int)(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case 0xFFFFFFFFFFFFFFFE: constant_factor =   (int64_t)options[i + 1]; break;
    case 0xFFFFFFFFFFFFFFFF: exposure_factor = *(double*)&options[i + 1]; break;
    }
    }
}

// OK
template<bool ENABLE_LOCATION>
void RM_VLC_SendSample(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    WSABUF wsaBuf[ENABLE_LOCATION ? 5 : 4];

    HookCallbackSocket* user = (HookCallbackSocket*)param;
    H26xFormat* format = (H26xFormat*)user->format;
    bool sh = format->profile != H26xProfile::H26xProfile_None;
    int id = format->framerate;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);

    if (!sh) { pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&g_pose_sh[id], sizeof(g_pose_sh[id]), NULL); }

    pBuffer->Lock(&pBytes, NULL, &cbData);

    int const metadata = sizeof(g_pose_sh[id]) - sizeof(g_pose_sh[id].timestamp) - sizeof(g_pose_sh[id].pose);
    DWORD cbDataEx = cbData + metadata;

    pack_buffer(wsaBuf, 0, &g_pose_sh[id].timestamp, sizeof(g_pose_sh[id].timestamp));
    pack_buffer(wsaBuf, 1, &cbDataEx, sizeof(cbDataEx));
    pack_buffer(wsaBuf, 2, pBytes, cbData);
    pack_buffer(wsaBuf, 3, &g_pose_sh[id], metadata);

    if constexpr(ENABLE_LOCATION)
    {
    pack_buffer(wsaBuf, 4, &g_pose_sh[id].pose, sizeof(g_pose_sh[id].pose));
    }

    bool ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();

    if (sh) { pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&g_pose_sh[id], sizeof(g_pose_sh[id]), NULL); }
}

// OK
template <bool ENABLE_LOCATION>
void RM_VLC_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    uint32_t const width     = RM_VLC_WIDTH;
    uint32_t const height    = RM_VLC_HEIGHT;
    uint32_t const framerate = RM_VLC_FPS;
    uint32_t const lumasize  = width * height;
    LONGLONG const duration  = HNS_BASE / framerate;

    PerceptionTimestamp ts = nullptr;
    uint32_t f = 0;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorVLCFrame* pVLCFrame; // Release
    BYTE const* pImage;
    size_t length;
    BYTE* pDst;
    H26xFormat format;
    std::vector<uint64_t> options;
    CustomMediaSink* pSink; // Release
    IMFSinkWriter* pSinkWriter; // Release
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    DWORD dwVideoIndex;
    HookCallbackSocket user;
    HANDLE clientevent; // CloseHandle
    uint32_t chromasize;
    uint32_t framebytes;
    HRESULT hr;
    VideoSubtype subtype;
    double exposure_factor;
    int64_t constant_factor;
    UINT64 adjusted_timestamp;
    VLC_Metadata vlc_metadata;
    int id;
    bool ok;

    ok = ReceiveH26xFormat_Divisor(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveH26xEncoder_Options(clientsocket, options);
    if (!ok) { return; }

    id = sensor->GetSensorType();

    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;
    user.format       = &format;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: chromasize = 0;            subtype = VideoSubtype::VideoSubtype_L8;   break;
    default:                            chromasize = lumasize / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    }

    CreateSinkWriterVideo(&pSink, &pSinkWriter, &dwVideoIndex, subtype, format, options, RM_VLC_SendSample<ENABLE_LOCATION>, &user);

    format.framerate = (uint8_t)id;

    RM_VLC_TranslateEncoderOptions(options, exposure_factor, constant_factor);

    framebytes = lumasize + chromasize;

    memset(&g_pose_sh[id], 0, sizeof(g_pose_sh[id]));

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    if (f == 0)
    {
    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    pVLCFrame->GetBuffer(&pImage, &length);
    pVLCFrame->GetExposure(&vlc_metadata.exposure);
    pVLCFrame->GetGain(&vlc_metadata.gain);

    MFCreateMemoryBuffer(framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    memcpy(pDst, pImage, lumasize);
    memset(pDst + lumasize, NV12_ZERO_CHROMA, chromasize);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(framebytes);

    MFCreateSample(&pSample);

    adjusted_timestamp = timestamp.HostTicks + (int64_t)((exposure_factor * vlc_metadata.exposure) / 100.0) + constant_factor;
    vlc_metadata.timestamp = adjusted_timestamp;
    vlc_metadata.sensor_ticks = timestamp.SensorTicks;

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);
    pSample->SetSampleTime(adjusted_timestamp);

    if constexpr (ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(adjusted_timestamp);
    vlc_metadata.pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    }

    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&vlc_metadata, sizeof(vlc_metadata));

    pSinkWriter->WriteSample(dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
    pVLCFrame->Release();
    }

    f = (f + 1) % format.divisor;

    pSensorFrame->Release();
    }
    while (WaitForSingleObject(clientevent, 0) == WAIT_TIMEOUT);

    sensor->CloseStream();

    pSinkWriter->Flush(dwVideoIndex);
    pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    CloseHandle(clientevent);
}

// OK
void RM_VLC_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_VLC_Stream<false>(sensor, clientsocket, nullptr);
}

// OK
void RM_VLC_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_VLC_Stream<true>(sensor, clientsocket, locator);
}

// OK
void RM_VLC_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[6];

    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(sensor, extrinsics);

    pack_buffer(wsaBuf, 0, uv2x.data(), (ULONG)(uv2x.size() * sizeof(float)));
    pack_buffer(wsaBuf, 1, uv2y.data(), (ULONG)(uv2y.size() * sizeof(float)));
    pack_buffer(wsaBuf, 2, extrinsics.m, sizeof(extrinsics.m));
    pack_buffer(wsaBuf, 3, mapx.data(), (ULONG)(mapx.size() * sizeof(float)));
    pack_buffer(wsaBuf, 4, mapy.data(), (ULONG)(mapy.size() * sizeof(float)));
    pack_buffer(wsaBuf, 5, K, sizeof(K));

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}
