
#include <mfapi.h>
#include <MemoryBuffer.h>
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"
#include "neon.h"
#include "ipc_sc.h"
#include "zdepth.hpp"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace Windows::Foundation;

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Storage;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

typedef void(*RM_ZAB_KERNEL)(u16 const*, u8*);

struct RM_ZAB_Blob
{
    std::vector<uint8_t>* z;
    float4x4 pose;
};

// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/133
// https://github.com/microsoft/HoloLens2ForCV/issues/142

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RM_ZHT_AbPT(uint16_t const *pAb, uint8_t *out)
{
    memcpy(out, pAb, RM_ZHT_ABSIZE);
}

// OK
template<bool ENABLE_LOCATION>
void RM_ZHT_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer *pBuffer; // Release
    HookCallbackSocket* user;
    int64_t sampletime;
    BYTE* pAb;
    DWORD cbAb;
    uint32_t ab_size;
    uint32_t z_size;
    uint32_t cbData;
    RM_ZAB_Blob blob;
    WSABUF wsaBuf[ENABLE_LOCATION ? 7 : 6];
    bool ok;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);
    pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&blob, sizeof(blob), NULL);

    pBuffer->Lock(&pAb, NULL, &cbAb);

    ab_size = (uint32_t)cbAb;
    z_size = (uint32_t)blob.z->size();

    cbData = sizeof(ab_size) + sizeof(z_size) + ab_size + z_size;

    pack_buffer(wsaBuf, 0, &sampletime, sizeof(sampletime));
    pack_buffer(wsaBuf, 1, &cbData, sizeof(cbData));
    pack_buffer(wsaBuf, 2, &ab_size, sizeof(ab_size));
    pack_buffer(wsaBuf, 3, &z_size, sizeof(z_size));
    pack_buffer(wsaBuf, 4, pAb, ab_size);
    pack_buffer(wsaBuf, 5, blob.z->data(), z_size);

    if constexpr (ENABLE_LOCATION)
    {
    pack_buffer(wsaBuf, 6, &blob.pose, sizeof(blob.pose));
    }

    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();

    delete blob.z;
}

// OK
template<bool ENABLE_LOCATION>
void RM_ZHT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    uint32_t const width     = RM_ZHT_WIDTH;
    uint32_t const height    = RM_ZHT_HEIGHT;
    uint32_t const framerate = RM_ZHT_FPS;
    uint32_t const duration  = HNS_BASE / framerate;

    PerceptionTimestamp ts = nullptr;
    uint32_t f = 0;
    uint32_t g = 0;
    RM_ZAB_Blob blob;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    UINT16 const* pDepth;
    UINT16 const* pAbImage;
    size_t nDepthCount;
    size_t nAbCount;
    BYTE* pDst;
    H26xFormat format;
    CustomMediaSink* pSink; // Release
    IMFSinkWriter* pSinkWriter; // Release
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    DWORD dwVideoIndex;
    HookCallbackSocket user;
    HANDLE clientevent; // CloseHandle
    uint32_t framebytes;
    RM_ZAB_KERNEL kernel;
    HRESULT hr;
    VideoSubtype subtype;
    zdepth::DepthCompressor compressor;
    bool ok;

    ok = ReceiveH26xFormat_Divisor(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(clientsocket, format);
    if (!ok) { return; }

    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;
    user.format       = &format;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: kernel = RM_ZHT_AbPT;   framebytes =  width * height * 2;      subtype = VideoSubtype::VideoSubtype_L16;  break;
    default:                            kernel = Neon_AbToNV12; framebytes = (width * height * 3) / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    }

    CreateSinkWriterVideo(&pSink, &pSinkWriter, &dwVideoIndex, subtype, format, RM_ZHT_SendSampleToSocket<ENABLE_LOCATION>, &user);

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame);
    if (FAILED(hr)) { break; }

    if (f == 0)
    {
    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    Neon_ZHTInvalidate(pDepth, const_cast<UINT16*>(pDepth));
    blob.z = new (std::nothrow) std::vector<uint8_t>(); // delete    
    compressor.Compress(width, height, pDepth, *blob.z, g == 0);
    g = (g + 1) % format.gop_size;

    MFCreateMemoryBuffer(framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    kernel(pAbImage, pDst);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(framebytes);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);
    pSample->SetSampleTime(timestamp.HostTicks);

    if constexpr (ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    blob.pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    }

    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&blob, sizeof(blob));

    pSinkWriter->WriteSample(dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
    pDepthFrame->Release();
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
void RM_ZHT_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_ZHT_Stream<false>(sensor, clientsocket, nullptr);
}

// OK
void RM_ZHT_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_ZHT_Stream<true>(sensor, clientsocket, locator);
}

// OK
void RM_ZHT_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    float const scale = 1000.0f;
    float const alias = 1055.0f;

    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[8];

    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(sensor, extrinsics);

    pack_buffer(wsaBuf, 0, uv2x.data(), (ULONG)(uv2x.size() * sizeof(float)));
    pack_buffer(wsaBuf, 1, uv2y.data(), (ULONG)(uv2y.size() * sizeof(float)));
    pack_buffer(wsaBuf, 2, &extrinsics.m[0][0], sizeof(extrinsics.m));
    pack_buffer(wsaBuf, 3, &scale, sizeof(scale));
    pack_buffer(wsaBuf, 4, &alias, sizeof(alias));
    pack_buffer(wsaBuf, 5, mapx.data(), (ULONG)(mapx.size() * sizeof(float)));
    pack_buffer(wsaBuf, 6, mapy.data(), (ULONG)(mapy.size() * sizeof(float)));
    pack_buffer(wsaBuf, 7, K, sizeof(K));

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}
