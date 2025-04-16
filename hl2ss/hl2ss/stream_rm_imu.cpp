
#include "research_mode.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_rm_imu.h"

#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Foundation::Numerics;

typedef void(*RM_IMU_PROC)(IResearchModeSensorFrame*, HOOK_RM_IMU_PROC, void*);

class Channel_RM_IMU : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    RM_IMU_PROC m_process_sample;
    std::unique_ptr<Encoder_RM_IMU> m_pEncoder;
    bool m_enable_location;

    bool Startup();
    void Run();   
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(IResearchModeSensorFrame* frame);
    void OnFrameProcess(void const* imu_buffer, size_t imu_bytes, UINT64 host_ticks, UINT64 sensor_ticks);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(void const* imu_buffer, size_t imu_bytes, UINT64 host_ticks, UINT64 sensor_ticks, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_RM_IMU(char const* name, char const* port, uint32_t id, ResearchModeSensorType kind, RM_IMU_PROC process_sample);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_IMU> g_channel_acc;
static std::unique_ptr<Channel_RM_IMU> g_channel_gyr;
static std::unique_ptr<Channel_RM_IMU> g_channel_mag;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_IMU::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_IMU*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_IMU::Thunk_Sample(void const* imu_buffer, size_t imu_bytes, UINT64 host_ticks, UINT64 sensor_ticks, void* self)
{
    static_cast<Channel_RM_IMU*>(self)->OnFrameProcess(imu_buffer, imu_bytes, host_ticks, sensor_ticks);
}

// OK
void Channel_RM_IMU::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_RM_IMU*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_RM_IMU::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    m_process_sample(frame, Thunk_Sample, this);
}

// OK
void Channel_RM_IMU::OnFrameProcess(void const* imu_buffer, size_t imu_bytes, UINT64 host_ticks, UINT64 sensor_ticks)
{
    (void)sensor_ticks;

    RM_IMU_Metadata metadata;

    metadata.timestamp = host_ticks;

    if (m_enable_location)
    {
    metadata.pose = ResearchMode_GetRigNodeWorldPose(host_ticks);
    }

    m_pEncoder->WriteSample(static_cast<BYTE const*>(imu_buffer), static_cast<uint32_t>(imu_bytes), host_ticks, &metadata);
}

// OK
void Channel_RM_IMU::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    RM_IMU_Metadata* p = static_cast<RM_IMU_Metadata*>(metadata);
    WSABUF wsaBuf[4];

    pack_buffer(wsaBuf, 0, &p->timestamp, sizeof(p->timestamp));
    pack_buffer(wsaBuf, 1, &encoded_size, sizeof(encoded_size));
    pack_buffer(wsaBuf, 2, encoded, encoded_size);
    pack_buffer(wsaBuf, 3, &p->pose, sizeof(p->pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_RM_IMU::Execute_Mode0(bool enable_location)
{
    m_pEncoder        = std::make_unique<Encoder_RM_IMU>(Thunk_Encoder, this);
    m_enable_location = enable_location;

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
void Channel_RM_IMU::Execute_Mode2()
{
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[1];
    bool ok;

    ok = ResearchMode_GetExtrinsics(m_sensor, extrinsics);
    if (!ok) { return; } // No Mode2 for RM_MAG

    pack_buffer(wsaBuf, 0, extrinsics.m, sizeof(extrinsics.m));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_RM_IMU::Channel_RM_IMU(char const* name, char const* port, uint32_t id, ResearchModeSensorType kind, RM_IMU_PROC process_sample) :
Channel(name, port, id)
{
    m_sensor = ResearchMode_GetSensor(kind);
    m_process_sample = process_sample;
}

// OK
bool Channel_RM_IMU::Startup()
{
    SetNoDelay(true);
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_IMU::Run()
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
void Channel_RM_IMU::Cleanup()
{
}

// OK
void RM_ACC_Startup()
{
    g_channel_acc = std::make_unique<Channel_RM_IMU>("RM_ACC", PORT_NAME_RM_ACC, PORT_ID_RM_ACC, ResearchModeSensorType::IMU_ACCEL, ResearchMode_ProcessSample_ACC);
}

// OK
void RM_GYR_Startup()
{
    g_channel_gyr = std::make_unique<Channel_RM_IMU>("RM_GYR", PORT_NAME_RM_GYR, PORT_ID_RM_GYR, ResearchModeSensorType::IMU_GYRO, ResearchMode_ProcessSample_GYR);
}

// OK
void RM_MAG_Startup()
{
    g_channel_mag = std::make_unique<Channel_RM_IMU>("RM_MAG", PORT_NAME_RM_MAG, PORT_ID_RM_MAG, ResearchModeSensorType::IMU_MAG, ResearchMode_ProcessSample_MAG);
}

// OK
void RM_ACC_Cleanup()
{
    g_channel_acc.reset();
}

// OK
void RM_GYR_Cleanup()
{
    g_channel_gyr.reset();
}

// OK
void RM_MAG_Cleanup()
{
    g_channel_mag.reset();
}
