
#include <mfapi.h>
#include <MemoryBuffer.h>
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"
#include "neon.h"
#include "ipc_sc.h"

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

typedef void(*ZHT_KERNEL)(u16 const*, u16 const*, u8*);

// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/133
// https://github.com/microsoft/HoloLens2ForCV/issues/142

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// ZHT ************************************************************************

// OK
static void RM_ZHT_Stack(u16 const* pDepth, u16 const* pAb, u8 *out)
{
    int const block = RM_ZHT_WIDTH * RM_ZHT_HEIGHT * sizeof(u16);
    
    memcpy(out,         pDepth, block);
    memcpy(out + block, pAb,    block);
}

// OK
template<bool ENABLE_LOCATION>
void RM_ZHT_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer *pBuffer; // Release
    HookCallbackSocket* user;
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    float4x4 pose;    
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];
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
void RM_ZHT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    uint32_t const width      = RM_ZHT_WIDTH;
    uint32_t const height     = RM_ZHT_HEIGHT;
    uint32_t const framerate  = RM_ZHT_FPS;
    uint32_t const duration   = HNS_BASE / framerate;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
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
    ZHT_KERNEL kernel;
    HRESULT hr;
    bool ok;

    ok = ReceiveVideoH26x(clientsocket, format);
    if (!ok) { return; }

    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.clientsocket = clientsocket;
    user.clientevent  = clientevent;
    user.data_profile = format.profile;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: kernel = RM_ZHT_Stack;   framebytes =  width * height * 2  * 2; CreateSinkWriterARGBToARGB(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_ZHT_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            kernel = Neon_ZHTToNV12; framebytes = (width * height * 3) / 2; CreateSinkWriterNV12ToH26x(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_ZHT_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame);
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    MFCreateMemoryBuffer(framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    kernel(pDepth, pAbImage, pDst);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(framebytes);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);
    pSample->SetSampleTime(timestamp.HostTicks);

    if constexpr (ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(float4x4));
    }

    pSinkWriter->WriteSample(dwVideoIndex, pSample);

    pDepthFrame->Release();
    pSensorFrame->Release();
    pSample->Release();
    pBuffer->Release();
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
void RM_ZHT_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_ZHT_Stream<false>(sensor, clientsocket, nullptr);
}

// OK
void RM_ZHT_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_ZHT_Stream<true>(sensor, clientsocket, locator);
}

// OK
void RM_ZHT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    float const scale = 1000.0f;
    float const alias =  999.0f;

    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[8];

    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(sensor, extrinsics);

    wsaBuf[0].buf = (char*)uv2x.data();
    wsaBuf[0].len = (ULONG)(uv2x.size() * sizeof(float));

    wsaBuf[1].buf = (char*)uv2y.data();
    wsaBuf[1].len = (ULONG)(uv2y.size() * sizeof(float));

    wsaBuf[2].buf = (char*)&extrinsics.m[0][0];
    wsaBuf[2].len = sizeof(extrinsics.m);

    wsaBuf[3].buf = (char*)&scale;
    wsaBuf[3].len = sizeof(scale);

    wsaBuf[4].buf = (char*)&alias;
    wsaBuf[4].len = sizeof(alias);

    wsaBuf[5].buf = (char*)mapx.data();
    wsaBuf[5].len = (ULONG)(mapx.size() * sizeof(float));

    wsaBuf[6].buf = (char*)mapy.data();
    wsaBuf[6].len = (ULONG)(mapy.size() * sizeof(float));

    wsaBuf[7].buf = (char*)K;
    wsaBuf[7].len = sizeof(K);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// ZLT ************************************************************************

// OK
template<bool ENABLE_LOCATION>
void RM_ZLT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    int const width  = RM_ZLT_WIDTH;
    int const height = RM_ZLT_HEIGHT;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    BYTE const* pSigma;
    UINT16 const* pDepth;
    UINT16 const* pAbImage;
    size_t nSigmaCount;
    size_t nDepthCount;
    size_t nAbCount;
    BYTE* pixelBufferData;
    UINT32 pixelBufferDataLength;
    BitmapPropertySet pngProperties;
    PngFilterMode filter;
    uint32_t streamSize;
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];
    HRESULT hr;
    bool ok;

    ok = ReceivePNGFilter(clientsocket, filter);
    if (!ok) { return; }

    pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(filter), winrt::Windows::Foundation::PropertyType::UInt8));
    
    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    auto softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Bgra8, width, height, BitmapAlphaMode::Straight);
    {
    auto bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    auto spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();
    spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    Neon_ZLTToBGRA8(pSigma, pDepth, pAbImage, (u32*)pixelBufferData);
    }

    auto stream = InMemoryRandomAccessStream();
    auto encoder = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, pngProperties).get();

    encoder.SetSoftwareBitmap(softwareBitmap);
    encoder.FlushAsync().get();

    streamSize = (uint32_t)stream.Size();
    auto streamBuf = Buffer(streamSize);

    stream.ReadAsync(streamBuf, streamSize, InputStreamOptions::None).get();

    wsaBuf[0].buf = (char*)&timestamp.HostTicks;
    wsaBuf[0].len = sizeof(timestamp.HostTicks);
    
    wsaBuf[1].buf = (char*)&streamSize;
    wsaBuf[1].len = sizeof(streamSize);

    wsaBuf[2].buf = (char*)streamBuf.data();
    wsaBuf[2].len = streamSize;

    if constexpr(ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));

    wsaBuf[3].buf = (char*)&pose;
    wsaBuf[3].len = sizeof(pose);
    }

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pDepthFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
void RM_ZLT_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_ZLT_Stream<false>(sensor, clientsocket, nullptr);
}

// OK
void RM_ZLT_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_ZLT_Stream<true>(sensor, clientsocket, locator);
}

// OK
void RM_ZLT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    float const scale = 1000.0f;

    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[7];

    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(sensor, extrinsics);

    wsaBuf[0].buf = (char*)uv2x.data();
    wsaBuf[0].len = (ULONG)(uv2x.size() * sizeof(float));

    wsaBuf[1].buf = (char*)uv2y.data();
    wsaBuf[1].len = (ULONG)(uv2y.size() * sizeof(float));

    wsaBuf[2].buf = (char*)&extrinsics.m[0][0];
    wsaBuf[2].len = sizeof(extrinsics.m);

    wsaBuf[3].buf = (char*)&scale;
    wsaBuf[3].len = sizeof(scale);

    wsaBuf[4].buf = (char*)mapx.data();
    wsaBuf[4].len = (ULONG)(mapx.size() * sizeof(float));

    wsaBuf[5].buf = (char*)mapy.data();
    wsaBuf[5].len = (ULONG)(mapy.size() * sizeof(float));

    wsaBuf[6].buf = (char*)K;
    wsaBuf[6].len = sizeof(K);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}
