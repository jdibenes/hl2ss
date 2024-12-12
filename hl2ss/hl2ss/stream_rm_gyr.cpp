
#include "research_mode.h"
#include "server_channel.h"
#include "server_settings.h"

#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Foundation::Numerics;

class Channel_RM_GYR : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    bool m_enable_location;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(IResearchModeSensorFrame* frame);
    void OnFrameProcess(GyroDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks);

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(GyroDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self);

public:
    Channel_RM_GYR(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_GYR> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_GYR::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_GYR*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_GYR::Thunk_Sample(GyroDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self)
{
    static_cast<Channel_RM_GYR*>(self)->OnFrameProcess(imu_buffer, imu_samples, host_ticks, sensor_ticks);
}

// OK
void Channel_RM_GYR::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    ResearchMode_ProcessSample_GYR(frame, Thunk_Sample, this);
}

// OK
void Channel_RM_GYR::OnFrameProcess(GyroDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks)
{
    (void)sensor_ticks;

    ULONG full_size = static_cast<ULONG>(imu_samples * sizeof(GyroDataStruct));
    float4x4 pose = ResearchMode_GetRigNodeWorldPose(host_ticks);
    WSABUF wsaBuf[4];

    pack_buffer(wsaBuf, 0, &host_ticks, sizeof(host_ticks));
    pack_buffer(wsaBuf, 1, &full_size,  sizeof(full_size));
    pack_buffer(wsaBuf, 2, imu_buffer,  full_size);
    pack_buffer(wsaBuf, 3, &pose,       sizeof(pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_RM_GYR::Execute_Mode0(bool enable_location)
{
    m_enable_location = enable_location;

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);
}

// OK
void Channel_RM_GYR::Execute_Mode2()
{
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[1];

    ResearchMode_GetExtrinsics(m_sensor, extrinsics);

    pack_buffer(wsaBuf, 0, extrinsics.m, sizeof(extrinsics.m));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_RM_GYR::Channel_RM_GYR(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
    m_sensor = ResearchMode_GetSensor(ResearchModeSensorType::IMU_GYRO);
}

// OK
bool Channel_RM_GYR::Startup()
{
    SetNoDelay(true);
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_GYR::Run()
{
    uint8_t mode;
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, m_event_client, mode);
    if (!ok) { return; }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    case 2: Execute_Mode2();      break;
    }
}

// OK
void Channel_RM_GYR::Cleanup()
{
}

// OK
void RM_GYR_Startup()
{
    g_channel = std::make_unique<Channel_RM_GYR>("RM_GYR", PORT_NAME_RM_GYR, PORT_ID_RM_GYR);
}

// OK
void RM_GYR_Cleanup()
{
    g_channel.reset();
}
