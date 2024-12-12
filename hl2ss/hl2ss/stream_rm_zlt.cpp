
// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/142

#include "research_mode.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_rm_zlt.h"

class Channel_RM_ZLT : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    std::unique_ptr<Encoder_RM_ZLT> m_pEncoder;
    bool m_enable_location;
    uint32_t m_counter;
    uint32_t m_divisor;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(IResearchModeSensorFrame* frame);
    void OnFrameProcess(UINT8 const* sigma, UINT16 const* depth, UINT16 const* ab, UINT64 host_ticks, UINT64 sensor_ticks);
    void OnEncodingComplete(void* encoded_frame, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(UINT8 const* sigma, UINT16 const* depth, UINT16 const* ab, UINT64 host_ticks, UINT64 sensor_ticks, void* self);
    static void Thunk_Encoder(void* encoded_frame, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_RM_ZLT(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_ZLT> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_ZLT::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_ZLT*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_ZLT::Thunk_Sample(UINT8 const* sigma, UINT16 const* depth, UINT16 const* ab, UINT64 host_ticks, UINT64 sensor_ticks, void* self)
{
    static_cast<Channel_RM_ZLT*>(self)->OnFrameProcess(sigma, depth, ab, host_ticks, sensor_ticks);
}

// OK
void Channel_RM_ZLT::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_RM_ZLT*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_RM_ZLT::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    if (m_counter == 0) { ResearchMode_ProcessSample_ZLT(frame, Thunk_Sample, this); }
    m_counter = (m_counter + 1) % m_divisor;
}

// OK
void Channel_RM_ZLT::OnFrameProcess(UINT8 const* sigma, UINT16 const* depth, UINT16 const* ab, UINT64 host_ticks, UINT64 sensor_ticks)
{
    RM_ZLT_Metadata metadata;

    metadata.timestamp    = host_ticks;
    metadata.sensor_ticks = sensor_ticks;    
    metadata.pose         = ResearchMode_GetRigNodeWorldPose(host_ticks);

    m_pEncoder->WriteSample(sigma, depth, ab, host_ticks, &metadata);
}

// OK
void Channel_RM_ZLT::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    RM_ZLT_Metadata* p = static_cast<RM_ZLT_Metadata*>(metadata);
    ULONG full_size = encoded_size + sizeof(p->sensor_ticks);
    WSABUF wsaBuf[5];

    pack_buffer(wsaBuf, 0, &p->timestamp,    sizeof(p->timestamp));
    pack_buffer(wsaBuf, 1, &full_size,       sizeof(full_size));
    pack_buffer(wsaBuf, 2, encoded,          encoded_size);
    pack_buffer(wsaBuf, 3, &p->sensor_ticks, sizeof(p->sensor_ticks));
    pack_buffer(wsaBuf, 4, &p->pose,         sizeof(p->pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_RM_ZLT::Execute_Mode0(bool enable_location)
{
    H26xFormat format;
    ZABFormat zabFormat;
    bool ok;

    Encoder_RM_ZLT::SetH26xFormat(format);

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveZABFormat_PNGFilter(m_socket_client, m_event_client, zabFormat);
    if (!ok) { return; }

    m_pEncoder        = std::make_unique<Encoder_RM_ZLT>(Thunk_Encoder, this, format, zabFormat);
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
void Channel_RM_ZLT::Execute_Mode2()
{
    float const scale = 1000.0f;

    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[7];

    ResearchMode_GetIntrinsics(m_sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(m_sensor, extrinsics);

    pack_buffer(wsaBuf, 0, uv2x.data(),  (ULONG)(uv2x.size() * sizeof(float)));
    pack_buffer(wsaBuf, 1, uv2y.data(),  (ULONG)(uv2y.size() * sizeof(float)));
    pack_buffer(wsaBuf, 2, extrinsics.m, sizeof(extrinsics.m));
    pack_buffer(wsaBuf, 3, &scale,       sizeof(scale));
    pack_buffer(wsaBuf, 4, mapx.data(),  (ULONG)(mapx.size() * sizeof(float)));
    pack_buffer(wsaBuf, 5, mapy.data(),  (ULONG)(mapy.size() * sizeof(float)));
    pack_buffer(wsaBuf, 6, K,            sizeof(K));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_RM_ZLT::Channel_RM_ZLT(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
    m_sensor = ResearchMode_GetSensor(ResearchModeSensorType::DEPTH_LONG_THROW);
}

// OK
bool Channel_RM_ZLT::Startup()
{
    SetNoDelay(true);
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_ZLT::Run()
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
void Channel_RM_ZLT::Cleanup()
{
}

// OK
void RM_ZLT_Startup()
{
    g_channel = std::make_unique<Channel_RM_ZLT>("RM_ZLT", PORT_NAME_RM_ZLT, PORT_ID_RM_ZLT);
}

// OK
void RM_ZLT_Cleanup()
{
    g_channel.reset();
}
