
// TODO: AHAT

#include <MemoryBuffer.h>
#include "research_mode.h"
#include "server.h"
#include "utilities.h"
#include "locator.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace Windows::Foundation;

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Storage;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// LT *************************************************************************

// OK
template<bool ENABLE_LOCATION>
void RM_ZLT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    int     const width = RM_LT_WIDTH;
    int     const height = RM_LT_HEIGHT;
    int     const pixels = width * height;
    int     const n32ByteVectors = (width * height * 4) / 32;
    uint8_t const depthinvalid = 0x80;

    IResearchModeSensorFrame* pSensorFrame; // Release
    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    BitmapPropertySet pngProperties;
    BYTE const* pSigma;
    size_t nSigmaCount;
    UINT16* pDepth;
    size_t nDepthCount;
    UINT16 const* pAbImage;
    size_t nAbCount;
    BYTE* pixelBufferData;
    UINT32 pixelBufferDataLength;
    uint32_t streamSize;
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];
    float4x4 pose;
    bool ok;

    pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(PngFilterMode::Adaptive), winrt::Windows::Foundation::PropertyType::UInt8));
    
    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
    pDepthFrame->GetBuffer(const_cast<const UINT16**>(&pDepth), &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    auto stream = InMemoryRandomAccessStream();
    auto opEncoderCreate = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, pngProperties);

    for (int i = 0; i < pixels; ++i) { if (pSigma[i] & depthinvalid) { pDepth[i] = 0; } }

    auto softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Bgra8, width, height, BitmapAlphaMode::Straight);
    {
    auto bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    auto spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();
    spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    PackUINT16toUINT32((BYTE*)pDepth, (BYTE*)pAbImage, pixelBufferData, n32ByteVectors);
    }

    auto encoder = opEncoderCreate.get();

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
    pose = Locator_Locate(QPCTimestampToPerceptionTimestamp(timestamp.HostTicks), locator, world);

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
    RM_ZLT_Stream<false>(sensor, clientsocket, nullptr, nullptr);
}

// OK
void RM_ZLT_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    RM_ZLT_Stream<true>(sensor, clientsocket, locator, world);
}

// OK
void RM_ZLT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    float const scale = 1000.0f;

    std::vector<float> uv2x;
    std::vector<float> uv2y;
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[4];

    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y);
    ResearchMode_GetExtrinsics(sensor, extrinsics);

    wsaBuf[0].buf = (char*)uv2x.data();
    wsaBuf[0].len = (ULONG)(uv2x.size() * sizeof(float));

    wsaBuf[1].buf = (char*)uv2y.data();
    wsaBuf[1].len = (ULONG)(uv2y.size() * sizeof(float));

    wsaBuf[2].buf = (char*)&extrinsics.m[0][0];
    wsaBuf[2].len = sizeof(extrinsics.m);

    wsaBuf[3].buf = (char*)&scale;
    wsaBuf[3].len = sizeof(scale);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// AHAT ***********************************************************************

void RM_ZHT_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    (void)sensor;
    (void)clientsocket;
}

void RM_ZHT_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    (void)sensor;
    (void)clientsocket;
    (void)locator;
    (void)world;
}

void RM_ZHT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    (void)sensor;
    (void)clientsocket;
}
