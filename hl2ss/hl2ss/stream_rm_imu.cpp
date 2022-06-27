
#include "research_mode.h"
#include "server.h"
#include "locator.h"

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// Common *********************************************************************

// OK
template<class IResearchModeIMUFrame, class IMUDataStruct, bool ENABLE_LOCATION>
void RM_IMU_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    int const chunksize = 20;

    IResearchModeSensorFrame* pSensorFrame; // Release
    IResearchModeIMUFrame* pSensorIMUFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IMUDataStruct const* pIMUBuffer;
    size_t nIMUSamples;
    std::vector<BYTE> sampleBuffer;
    int bufSize;
    BYTE* pDst;
    WSABUF wsaBuf[ENABLE_LOCATION ? 4 : 3];
    float4x4 pose;
    bool ok;

    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

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

    for (int i = 0; i < (int)nIMUSamples; ++i) { memcpy(pDst + (i * chunksize), &(pIMUBuffer[i].SocTicks), chunksize); }

if constexpr(ENABLE_LOCATION)
{
    pose = Locator_Locate(timestamp.HostTicks, locator, world);
}

    wsaBuf[0].buf = (char*)&timestamp.HostTicks; wsaBuf[0].len = sizeof(timestamp.HostTicks);
    wsaBuf[1].buf = (char*)&bufSize;             wsaBuf[1].len = sizeof(bufSize);
    wsaBuf[2].buf = (char*)pDst;                 wsaBuf[2].len = bufSize;

if constexpr(ENABLE_LOCATION)
{
    wsaBuf[3].buf = (char*)&pose;                wsaBuf[3].len = sizeof(pose);
}

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pSensorIMUFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
void RM_IMU_Extrinsics(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[1];

    ResearchMode_GetExtrinsics(sensor, extrinsics);

    wsaBuf[0].buf = (char*)&extrinsics.m[0][0]; wsaBuf[0].len = sizeof(extrinsics.m);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// ACC ************************************************************************

// OK
void RM_ACC_Stream_Mode0(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    RM_IMU_Stream<IResearchModeAccelFrame, AccelDataStruct, false>(sensor, clientsocket, nullptr, nullptr);
}

// OK
void RM_ACC_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    RM_IMU_Stream<IResearchModeAccelFrame, AccelDataStruct, true>(sensor, clientsocket, locator, world);
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
    RM_IMU_Stream<IResearchModeGyroFrame, GyroDataStruct, false>(sensor, clientsocket, nullptr, nullptr);
}

// OK
void RM_GYR_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    RM_IMU_Stream<IResearchModeGyroFrame, GyroDataStruct, true>(sensor, clientsocket, locator, world);
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
    RM_IMU_Stream<IResearchModeMagFrame, MagDataStruct, false>(sensor, clientsocket, nullptr, nullptr);
}

// OK
void RM_MAG_Stream_Mode1(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator, SpatialCoordinateSystem const& world)
{
    RM_IMU_Stream<IResearchModeMagFrame, MagDataStruct, true>(sensor, clientsocket, locator, world);
}

// OK
void RM_MAG_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    (void)sensor;
    (void)clientsocket;

    // no interface
}
