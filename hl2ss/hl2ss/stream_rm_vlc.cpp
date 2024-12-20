
// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/86
// https://github.com/microsoft/HoloLens2ForCV/issues/117
// https://github.com/microsoft/HoloLens2ForCV/issues/134

#include "research_mode.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_rm_vlc.h"

class Channel_RM_VLC : public Channel
{
private:
    IResearchModeSensor* m_sensor;
    std::unique_ptr<Encoder_RM_VLC> m_pEncoder;
    bool m_enable_location;
    uint32_t m_counter;
    uint32_t m_divisor;
    double m_exposure_factor;
    int64_t m_constant_factor;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived(IResearchModeSensorFrame* sensor);
    void OnFrameProcess(BYTE const* image, UINT64 host_ticks, UINT64 sensor_ticks, UINT64 exposure, UINT32 gain);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);
    
    static void TranslateEncoderOptions(std::vector<uint64_t> const& options, double& exposure_factor, int64_t& constant_factor);

    static void Thunk_Sensor(IResearchModeSensorFrame* frame, void* self);
    static void Thunk_Sample(BYTE const* image, UINT64 host_ticks, UINT64 sensor_ticks, UINT64 exposure, UINT32 gain, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_RM_VLC(char const* name, char const* port, uint32_t id, ResearchModeSensorType kind);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_VLC> g_channel_lf;
static std::unique_ptr<Channel_RM_VLC> g_channel_ll;
static std::unique_ptr<Channel_RM_VLC> g_channel_rf;
static std::unique_ptr<Channel_RM_VLC> g_channel_rr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_VLC::TranslateEncoderOptions(std::vector<uint64_t> const& options, double& exposure_factor, int64_t& constant_factor)
{
    exposure_factor = 0.0;
    constant_factor = 0;

    for (int i = 0; i < static_cast<int>(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case HL2SSAPI::HL2SSAPI_VLCHostTicksOffsetConstant: constant_factor =       static_cast<int64_t>(       options[i + 1]); break;
    case HL2SSAPI::HL2SSAPI_VLCHostTicksOffsetExposure: exposure_factor = *reinterpret_cast<double const*>(&options[i + 1]); break;
    }
    }
}

// OK
void Channel_RM_VLC::Thunk_Sensor(IResearchModeSensorFrame* frame, void* self)
{
    static_cast<Channel_RM_VLC*>(self)->OnFrameArrived(frame);
}

// OK
void Channel_RM_VLC::Thunk_Sample(BYTE const* image, UINT64 host_ticks, UINT64 sensor_ticks, UINT64 exposure, UINT32 gain, void* self)
{
    static_cast<Channel_RM_VLC*>(self)->OnFrameProcess(image, host_ticks, sensor_ticks, exposure, gain);
}

// OK
void Channel_RM_VLC::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_RM_VLC*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_RM_VLC::OnFrameArrived(IResearchModeSensorFrame* frame)
{
    if (m_counter == 0) { ResearchMode_ProcessSample_VLC(frame, Thunk_Sample, this); }
    m_counter = (m_counter + 1) % m_divisor;
}

// OK
void Channel_RM_VLC::OnFrameProcess(BYTE const* image, UINT64 host_ticks, UINT64 sensor_ticks, UINT64 exposure, UINT32 gain)
{
    int64_t adjusted_timestamp = host_ticks + (int64_t)((m_exposure_factor * exposure) / 100.0) + m_constant_factor;
    RM_VLC_Metadata metadata;

    metadata.timestamp    = adjusted_timestamp;
    metadata.sensor_ticks = sensor_ticks;
    metadata.exposure     = exposure;
    metadata.gain         = gain;  
    metadata._reserved    = 0;
    metadata.pose         = ResearchMode_GetRigNodeWorldPose(adjusted_timestamp);

    m_pEncoder->WriteSample(image, adjusted_timestamp, &metadata);
}

// OK
void Channel_RM_VLC::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    ULONG const embed_size = sizeof(RM_VLC_Metadata) - sizeof(RM_VLC_Metadata::timestamp) - sizeof(RM_VLC_Metadata::pose);

    RM_VLC_Metadata* p = static_cast<RM_VLC_Metadata*>(metadata);    
    ULONG full_size = encoded_size + embed_size;
    WSABUF wsaBuf[5];

    pack_buffer(wsaBuf, 0, &p->timestamp, sizeof(p->timestamp));
    pack_buffer(wsaBuf, 1, &full_size,    sizeof(full_size));
    pack_buffer(wsaBuf, 2, encoded,       encoded_size);
    pack_buffer(wsaBuf, 3, p,             embed_size);
    pack_buffer(wsaBuf, 4, &p->pose,      sizeof(p->pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_RM_VLC::Execute_Mode0(bool enable_location)
{
    H26xFormat format;
    std::vector<uint64_t> options;
    bool ok;

    Encoder_RM_VLC::SetH26xFormat(format);

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveEncoderOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    m_pEncoder        = std::make_unique<Encoder_RM_VLC>(Thunk_Encoder, this, format, options);
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;    

    TranslateEncoderOptions(options, m_exposure_factor, m_constant_factor);

    ResearchMode_ExecuteSensorLoop(m_sensor, Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
}

// OK
void Channel_RM_VLC::Execute_Mode2()
{
    std::vector<float> uv2x;
    std::vector<float> uv2y;
    std::vector<float> mapx;
    std::vector<float> mapy;
    float K[4];
    DirectX::XMFLOAT4X4 extrinsics;
    WSABUF wsaBuf[6];

    ResearchMode_GetIntrinsics(m_sensor, uv2x, uv2y, mapx, mapy, K);
    ResearchMode_GetExtrinsics(m_sensor, extrinsics);

    pack_buffer(wsaBuf, 0, uv2x.data(),  (ULONG)(uv2x.size() * sizeof(float)));
    pack_buffer(wsaBuf, 1, uv2y.data(),  (ULONG)(uv2y.size() * sizeof(float)));
    pack_buffer(wsaBuf, 2, extrinsics.m, sizeof(extrinsics.m));
    pack_buffer(wsaBuf, 3, mapx.data(),  (ULONG)(mapx.size() * sizeof(float)));
    pack_buffer(wsaBuf, 4, mapy.data(),  (ULONG)(mapy.size() * sizeof(float)));
    pack_buffer(wsaBuf, 5, K,            sizeof(K));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_RM_VLC::Channel_RM_VLC(char const* name, char const* port, uint32_t id, ResearchModeSensorType kind) : 
Channel(name, port, id)
{
    m_sensor = ResearchMode_GetSensor(kind);
}

// OK
bool Channel_RM_VLC::Startup()
{
    SetNoDelay(true);
    return ResearchMode_WaitForConsent(m_sensor);
}

// OK
void Channel_RM_VLC::Run()
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
void Channel_RM_VLC::Cleanup()
{
}

// OK
void RM_VLF_Startup() 
{ 
    g_channel_lf = std::make_unique<Channel_RM_VLC>("RM_VLF", PORT_NAME_RM_VLF, PORT_ID_RM_VLF, ResearchModeSensorType::LEFT_FRONT);
}

// OK
void RM_VLL_Startup()
{ 
    g_channel_ll = std::make_unique<Channel_RM_VLC>("RM_VLL", PORT_NAME_RM_VLL, PORT_ID_RM_VLL, ResearchModeSensorType::LEFT_LEFT);
}

// OK
void RM_VRF_Startup()
{
    g_channel_rf = std::make_unique<Channel_RM_VLC>("RM_VRF", PORT_NAME_RM_VRF, PORT_ID_RM_VRF, ResearchModeSensorType::RIGHT_FRONT);
}

// OK
void RM_VRR_Startup()
{ 
    g_channel_rr = std::make_unique<Channel_RM_VLC>("RM_VRR", PORT_NAME_RM_VRR, PORT_ID_RM_VRR, ResearchModeSensorType::RIGHT_RIGHT);
}

// OK
void RM_VLF_Cleanup() 
{ 
    g_channel_lf.reset();
}

// OK
void RM_VLL_Cleanup()
{ 
    g_channel_ll.reset();
}

// OK
void RM_VRF_Cleanup() 
{ 
    g_channel_rf.reset();
}

// OK
void RM_VRR_Cleanup() 
{ 
    g_channel_rr.reset();
}
