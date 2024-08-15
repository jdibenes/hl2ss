
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

// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/142

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<bool ENABLE_LOCATION>
void RM_ZLT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    int const width  = RM_ZLT_WIDTH;
    int const height = RM_ZLT_HEIGHT;

    PerceptionTimestamp ts = nullptr;
    uint32_t f = 0;
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
    ZABFormat zabFormat;
    uint32_t streamSize;
    WSABUF wsaBuf[ENABLE_LOCATION ? 5 : 4];
    HRESULT hr;
    H26xFormat format;
    bool ok;

    ok = ReceiveH26xFormat_Divisor(clientsocket, format);
    if (!ok) { return; }

    ok = ReceiveZABFormat_PNGFilter(clientsocket, zabFormat);
    if (!ok) { return; }

    pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(zabFormat.filter), winrt::Windows::Foundation::PropertyType::UInt8));
    
    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    if (f == 0)
    {
    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    auto const& softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Bgra8, width, height, BitmapAlphaMode::Straight);
    {
    auto const& bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    auto const& spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();
    spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    Neon_ZLTToBGRA8(pSigma, pDepth, pAbImage, (u32*)pixelBufferData);
    }

    auto const& stream = InMemoryRandomAccessStream();
    auto const& encoder = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, pngProperties).get();

    encoder.SetSoftwareBitmap(softwareBitmap);
    encoder.FlushAsync().get();

    streamSize = (uint32_t)stream.Size();
    auto const& streamBuf = Buffer(streamSize);

    stream.ReadAsync(streamBuf, streamSize, InputStreamOptions::None).get();

    DWORD streamSizeEx = streamSize + sizeof(timestamp.SensorTicks);

    pack_buffer(wsaBuf, 0, &timestamp.HostTicks, sizeof(timestamp.HostTicks));
    pack_buffer(wsaBuf, 1, &streamSizeEx, sizeof(streamSizeEx));
    pack_buffer(wsaBuf, 2, streamBuf.data(), streamSize);
    pack_buffer(wsaBuf, 3, &timestamp.SensorTicks, sizeof(timestamp.SensorTicks));

    if constexpr (ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));

    pack_buffer(wsaBuf, 4, &pose, sizeof(pose));
    }

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pDepthFrame->Release();
    }

    f = (f + 1) % format.divisor;

    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
void RM_ZLT_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_ZLT_Stream<false>(sensor, clientsocket, nullptr);
}

// OK
void RM_ZLT_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_ZLT_Stream<true>(sensor, clientsocket, locator);
}

// OK
void RM_ZLT_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
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

    pack_buffer(wsaBuf, 0, uv2x.data(), (ULONG)(uv2x.size() * sizeof(float)));
    pack_buffer(wsaBuf, 1, uv2y.data(), (ULONG)(uv2y.size() * sizeof(float)));
    pack_buffer(wsaBuf, 2, extrinsics.m, sizeof(extrinsics.m));
    pack_buffer(wsaBuf, 3, &scale, sizeof(scale));
    pack_buffer(wsaBuf, 4, mapx.data(), (ULONG)(mapx.size() * sizeof(float)));
    pack_buffer(wsaBuf, 5, mapy.data(), (ULONG)(mapy.size() * sizeof(float)));
    pack_buffer(wsaBuf, 6, K, sizeof(K));

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}
