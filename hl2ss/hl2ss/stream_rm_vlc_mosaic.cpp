
#include "research_mode.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_rm_vlc_mosaic.h"
#include "types.h"

class Channel_RM_VLC_Mosaic : public Channel
{
private:
    IResearchModeSensor* m_sensor[4];
    std::unique_ptr<Encoder_RM_VLC_Mosaic> m_pEncoder;
    bool m_enable_location;
    int m_count;
    uint32_t m_counter;
    uint32_t m_divisor;
    double m_exposure_factor;
    int64_t m_constant_factor;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(uint32_t streams, bool enable_location);

    void OnFrameProcess(BYTE const* const* image, UINT64 const* const* host_ticks, UINT64 const* const* sensor_ticks, UINT64 const* const* exposure, UINT32 const* const* gain, int count);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void TranslateEncoderOptions(std::vector<uint64_t> const& options, double& exposure_factor, int64_t& constant_factor);

    static void Thunk_Sample(BYTE const* const* image, UINT64 const* const* host_ticks, UINT64 const* const* sensor_ticks, UINT64 const* const* exposure, UINT32 const* const* gain, int count, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_RM_VLC_Mosaic(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RM_VLC_Mosaic> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_RM_VLC_Mosaic::TranslateEncoderOptions(std::vector<uint64_t> const& options, double& exposure_factor, int64_t& constant_factor)
{
    exposure_factor = 0.0;
    constant_factor = 0;

    for (int i = 0; i < static_cast<int>(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case HL2SSAPI::HL2SSAPI_VLCHostTicksOffsetConstant: constant_factor = static_cast<int64_t>(options[i + 1]); break;
    case HL2SSAPI::HL2SSAPI_VLCHostTicksOffsetExposure: exposure_factor = *reinterpret_cast<double const*>(&options[i + 1]); break;
    }
    }
}

// OK
void Channel_RM_VLC_Mosaic::Thunk_Sample(BYTE const* const* image, UINT64 const* const* host_ticks, UINT64 const* const* sensor_ticks, UINT64 const* const* exposure, UINT32 const* const* gain, int count, void* self)
{
    static_cast<Channel_RM_VLC_Mosaic*>(self)->OnFrameProcess(image, host_ticks, sensor_ticks, exposure, gain, count);
}

// OK
void Channel_RM_VLC_Mosaic::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_RM_VLC_Mosaic*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_RM_VLC_Mosaic::OnFrameProcess(BYTE const* const* image, UINT64 const* const* host_ticks, UINT64 const* const* sensor_ticks, UINT64 const* const* exposure, UINT32 const* const* gain, int count)
{
    RM_VLC_Mosaic_Metadata metadata;

    memset(&metadata, 0, sizeof(metadata));

    for (int i = 0; i < count; ++i)
    {
    metadata.host_ticks[i]   = *host_ticks[i] + (int64_t)((m_exposure_factor * *exposure[i]) / 100.0) + m_constant_factor;
    metadata.sensor_ticks[i] = *sensor_ticks[i];
    metadata.exposure[i]     = *exposure[i];
    metadata.gain[i]         = *gain[i];
    }

    metadata.timestamp = metadata.host_ticks[0];

    if (m_enable_location)
    {
    metadata.pose = ResearchMode_GetRigNodeWorldPose(metadata.timestamp);
    }

    m_pEncoder->WriteSample(image, count, metadata.timestamp, &metadata);
}

// OK
void Channel_RM_VLC_Mosaic::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    ULONG const embed_size = sizeof(RM_VLC_Mosaic_Metadata) - sizeof(RM_VLC_Mosaic_Metadata::timestamp) - sizeof(RM_VLC_Mosaic_Metadata::pose);

    RM_VLC_Mosaic_Metadata* p = static_cast<RM_VLC_Mosaic_Metadata*>(metadata);
    ULONG full_size = encoded_size + embed_size;
    WSABUF wsaBuf[5];

    pack_buffer(wsaBuf, 0, &p->timestamp, sizeof(p->timestamp));
    pack_buffer(wsaBuf, 1, &full_size, sizeof(full_size));
    pack_buffer(wsaBuf, 2, encoded, encoded_size);
    pack_buffer(wsaBuf, 3, p, embed_size);
    pack_buffer(wsaBuf, 4, &p->pose, sizeof(p->pose) * m_enable_location);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_RM_VLC_Mosaic::Execute_Mode0(uint32_t streams, bool enable_location)
{
    H26xFormat format;
    std::vector<uint64_t> options;
    std::vector<IResearchModeSensor*> sensor;
    int count;
    bool ok;

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveEncoderOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    for (int i = 0; i < 4; ++i) { if (bit_test(streams, i)) { sensor.push_back(m_sensor[i]); } }
    count = static_cast<int>(sensor.size());

    Encoder_RM_VLC_Mosaic::SetH26xFormat(format, count);

    m_pEncoder        = std::make_unique<Encoder_RM_VLC_Mosaic>(Thunk_Encoder, this, format, options);
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;

    TranslateEncoderOptions(options, m_exposure_factor, m_constant_factor);

    ResearchMode_ExecuteSensorLoop_VLC_Mosaic(sensor.data(), count, Thunk_Sample, this, m_event_client);

    m_pEncoder.reset();
}

// OK
Channel_RM_VLC_Mosaic::Channel_RM_VLC_Mosaic(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
    m_sensor[0] = ResearchMode_GetSensor(ResearchModeSensorType::LEFT_FRONT);
    m_sensor[1] = ResearchMode_GetSensor(ResearchModeSensorType::LEFT_LEFT);
    m_sensor[2] = ResearchMode_GetSensor(ResearchModeSensorType::RIGHT_FRONT);
    m_sensor[3] = ResearchMode_GetSensor(ResearchModeSensorType::RIGHT_RIGHT);
}

// OK
bool Channel_RM_VLC_Mosaic::Startup()
{
    SetNoDelay(true);
    return ResearchMode_WaitForConsent(m_sensor[0]) && ResearchMode_WaitForConsent(m_sensor[1]) && ResearchMode_WaitForConsent(m_sensor[2]) && ResearchMode_WaitForConsent(m_sensor[3]);
}

// OK
void Channel_RM_VLC_Mosaic::Run()
{
    uint8_t mode;
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, m_event_client, mode);
    if (!ok) { return; }

    uint32_t streams = bit_field(mode, 4, 0xF);

    switch (mode & 3)
    {
    case 0: Execute_Mode0(streams, false); break;
    case 1: Execute_Mode0(streams, true);  break;
    }
}

// OK
void Channel_RM_VLC_Mosaic::Cleanup()
{
}

// OK
void RM_VMU_Startup()
{
    g_channel = std::make_unique<Channel_RM_VLC_Mosaic>("RM_VMU", PORT_NAME_RM_VMU, PORT_ID_RM_VMU);
}

// OK
void RM_VMU_Cleanup()
{
    g_channel.reset();
}
