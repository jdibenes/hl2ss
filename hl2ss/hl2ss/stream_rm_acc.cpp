
#include "research_mode.h"
#include "locator.h"
#include "channel.h"
#include "ipc_sc.h"
#include "ports.h"
#include "timestamps.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

class Channel_RM_ACC : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    SpatialLocator m_locator = nullptr;
    bool m_enable_location;

    bool Startup();
    void Run();   
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(IResearchModeSensorFrame* frame);
    void OnFrameProcess(AccelDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks);

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(AccelDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self);

public:
    Channel_RM_ACC(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_ACC> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_ACC::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_ACC*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_ACC::Thunk_Sample(AccelDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self)
{
    static_cast<Channel_RM_ACC*>(self)->OnFrameProcess(imu_buffer, imu_samples, host_ticks, sensor_ticks);
}

// OK
void Channel_RM_ACC::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    ResearchMode_ProcessSample_ACC(frame, Thunk_Sample, this);
}

// OK
void Channel_RM_ACC::OnFrameProcess(AccelDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks)
{
    (void)sensor_ticks;

    ULONG full_size = static_cast<ULONG>(imu_samples * sizeof(AccelDataStruct));
    PerceptionTimestamp ts = QPCTimestampToPerceptionTimestamp(host_ticks);
    float4x4 pose = Locator_Locate(ts, m_locator, Locator_GetWorldCoordinateSystem(ts));
    WSABUF wsaBuf[4];

    pack_buffer(wsaBuf, 0, &host_ticks, sizeof(host_ticks));
    pack_buffer(wsaBuf, 1, &full_size,  sizeof(full_size));
    pack_buffer(wsaBuf, 2, imu_buffer,  full_size);
    pack_buffer(wsaBuf, 3, &pose,       sizeof(pose) * m_enable_location);

    bool ok = send_multiple(m_socket_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(m_event_client); }
}

// OK
void Channel_RM_ACC::Execute_Mode0(bool enable_location)
{
    m_enable_location = enable_location;

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);
}

// OK
void Channel_RM_ACC::Execute_Mode2()
{
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[1];

    ResearchMode_GetExtrinsics(m_sensor, extrinsics);

    pack_buffer(wsaBuf, 0, extrinsics.m, sizeof(extrinsics.m));

    send_multiple(m_socket_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_RM_ACC::Channel_RM_ACC(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
    m_sensor  = ResearchMode_GetSensor(ResearchModeSensorType::IMU_ACCEL);
    m_locator = ResearchMode_GetLocator();
}

// OK
bool Channel_RM_ACC::Startup()
{
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_ACC::Run()
{
    uint8_t mode;
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, mode);
    if (!ok) { return; }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    case 2: Execute_Mode2();      break;
    }
}

// OK
void Channel_RM_ACC::Cleanup()
{
}

// OK
void RM_ACC_Initialize()
{
    g_channel = std::make_unique<Channel_RM_ACC>("RM_ACC", PORT_NAME_RM_ACC, PORT_NUMBER_RM_ACC - PORT_NUMBER_BASE);
}

// OK
void RM_ACC_Cleanup()
{
    g_channel.reset();
}