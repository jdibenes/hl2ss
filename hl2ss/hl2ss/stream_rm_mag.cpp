
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

class Channel_RM_MAG : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    SpatialLocator m_locator = nullptr;
    bool m_enable_location;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);

    void OnFrameArrived(IResearchModeSensorFrame* frame);
    void OnFrameProcess(MagDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks);    

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(MagDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self);

public:
    Channel_RM_MAG(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_MAG> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_MAG::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_MAG*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_MAG::Thunk_Sample(MagDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks, void* self)
{
    static_cast<Channel_RM_MAG*>(self)->OnFrameProcess(imu_buffer, imu_samples, host_ticks, sensor_ticks);
}

// OK
void Channel_RM_MAG::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    ResearchMode_ProcessSample_MAG(frame, Thunk_Sample, this);
}

// OK
void Channel_RM_MAG::OnFrameProcess(MagDataStruct const* imu_buffer, size_t imu_samples, UINT64 host_ticks, UINT64 sensor_ticks)
{
    (void)sensor_ticks;

    ULONG full_size = static_cast<ULONG>(imu_samples * sizeof(MagDataStruct));
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
void Channel_RM_MAG::Execute_Mode0(bool enable_location)
{
    m_enable_location = enable_location;

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);
}

// OK
Channel_RM_MAG::Channel_RM_MAG(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
    m_sensor  = ResearchMode_GetSensor(ResearchModeSensorType::IMU_MAG);
    m_locator = ResearchMode_GetLocator();
}

// OK
bool Channel_RM_MAG::Startup()
{
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_MAG::Run()
{
    uint8_t mode;
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, mode);
    if (!ok) { return; }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    // No Mode2 for RM MAG
    }
}

// OK
void Channel_RM_MAG::Cleanup()
{
}

// OK
void RM_MAG_Initialize()
{
    g_channel = std::make_unique<Channel_RM_MAG>("RM_MAG", PORT_NAME_RM_MAG, PORT_NUMBER_RM_MAG - PORT_NUMBER_BASE);
}

// OK
void RM_MAG_Cleanup()
{
    g_channel.reset();
}
