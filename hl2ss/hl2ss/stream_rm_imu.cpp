
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// Common *********************************************************************

// OK
template<class IResearchModeIMUFrame, class IMUDataStruct, bool ENABLE_LOCATION>
void RM_IMU_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    int const chunksize = 28;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeIMUFrame* pSensorIMUFrame; // Release    
    IMUDataStruct const* pIMUBuffer;
    size_t nIMUSamples;
    std::vector<BYTE> sampleBuffer;
    int bufSize;
    BYTE* pDst;
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];    
    HRESULT hr;
    bool ok;

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorIMUFrame));

    if constexpr(std::is_same_v<IResearchModeIMUFrame, IResearchModeAccelFrame>)
    {
    pSensorIMUFrame->GetCalibratedAccelarationSamples(&pIMUBuffer, &nIMUSamples);
    }
    else if constexpr(std::is_same_v<IResearchModeIMUFrame, IResearchModeGyroFrame>)
    {
    pSensorIMUFrame->GetCalibratedGyroSamples(&pIMUBuffer, &nIMUSamples);
    }
    else if constexpr(std::is_same_v<IResearchModeIMUFrame, IResearchModeMagFrame>)
    {
    pSensorIMUFrame->GetMagnetometerSamples(&pIMUBuffer, &nIMUSamples);
    }

    bufSize = (int)(nIMUSamples * chunksize);
    if (sampleBuffer.size() < bufSize) { sampleBuffer.resize(bufSize); }
    pDst = sampleBuffer.data();

    for (int i = 0; i < (int)nIMUSamples; ++i) { memcpy(pDst + (i * chunksize), &(pIMUBuffer[i]), chunksize); }

    wsaBuf[0].buf = (char*)&timestamp.HostTicks;
    wsaBuf[0].len = sizeof(timestamp.HostTicks);
    
    wsaBuf[1].buf = (char*)&bufSize;
    wsaBuf[1].len = sizeof(bufSize);
    
    wsaBuf[2].buf = (char*)pDst;
    wsaBuf[2].len = bufSize;

    if constexpr(ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    
    wsaBuf[3].buf = (char*)&pose;
    wsaBuf[3].len = sizeof(pose);
    }

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pSensorIMUFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
static void RM_IMU_Extrinsics(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf;

    ResearchMode_GetExtrinsics(sensor, extrinsics);

    wsaBuf.buf = (char*)&extrinsics.m[0][0];
    wsaBuf.len = sizeof(extrinsics.m);

    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// ACC ************************************************************************

// OK
void RM_ACC_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Stream<IResearchModeAccelFrame, AccelDataStruct, false>(sensor, clientsocket, nullptr);
}

// OK
void RM_ACC_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_IMU_Stream<IResearchModeAccelFrame, AccelDataStruct, true>(sensor, clientsocket, locator);
}

// OK
void RM_ACC_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Extrinsics(sensor, clientsocket);
}

// GYR ************************************************************************

// OK
void RM_GYR_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Stream<IResearchModeGyroFrame, GyroDataStruct, false>(sensor, clientsocket, nullptr);
}

// OK
void RM_GYR_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_IMU_Stream<IResearchModeGyroFrame, GyroDataStruct, true>(sensor, clientsocket, locator);
}

// OK
void RM_GYR_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Extrinsics(sensor, clientsocket);
}

// MAG ************************************************************************

// OK
void RM_MAG_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Stream<IResearchModeMagFrame, MagDataStruct, false>(sensor, clientsocket, nullptr);
}

// OK
void RM_MAG_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    RM_IMU_Stream<IResearchModeMagFrame, MagDataStruct, true>(sensor, clientsocket, locator);
}

// OK
void RM_MAG_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    (void)sensor;
    (void)clientsocket;

    // no interface
}
