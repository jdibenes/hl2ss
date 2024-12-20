
#include <mfapi.h>
#include "personal_video.h"
#include "server_channel.h"
#include "server_settings.h"
#include "encoder_pv.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;

struct PV_Mode2
{
    float2   f;
    float2   c;
    float3   r;
    float2   t;
    float4x4 p;
    float4x4 extrinsics;
    float4   intrinsics_mf;
    float7   extrinsics_mf;
};

class Channel_PV : public Channel
{
private:
    std::unique_ptr<Encoder_PV> m_pEncoder;
    bool m_enable_location;
    uint32_t m_counter;
    uint32_t m_divisor;
    PV_Mode2 m_calibration;
    uint16_t m_width;
    uint16_t m_height;
    
    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0(bool enable_location);
    void Execute_Mode2();

    void OnFrameArrived_Mode0(MediaFrameReference const& frame);
    void OnFrameArrived_Mode2(MediaFrameReference const& frame);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void TranslateEncoderOptions(std::vector<uint64_t> const& options, MediaFrameReaderAcquisitionMode& acquisition_mode);
    
    static void Thunk_Sensor_Mode0(MediaFrameReference const& frame, void* self);
    static void Thunk_Sensor_Mode2(MediaFrameReference const& frame, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

public:
    Channel_PV(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_PV> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Channel_PV::TranslateEncoderOptions(std::vector<uint64_t> const& options, MediaFrameReaderAcquisitionMode& acquisition_mode)
{
    acquisition_mode = MediaFrameReaderAcquisitionMode::Buffered;

    for (int i = 0; i < static_cast<int>(options.size() & ~1ULL); i += 2)
    {
    switch (options[i])
    {
    case HL2SSAPI::HL2SSAPI_AcquisitionMode: acquisition_mode = (options[i + 1] & 1) ? MediaFrameReaderAcquisitionMode::Buffered : MediaFrameReaderAcquisitionMode::Realtime; break;
    }
    }
}

// OK
void Channel_PV::Thunk_Sensor_Mode0(MediaFrameReference const& frame, void* self)
{
    static_cast<Channel_PV*>(self)->OnFrameArrived_Mode0(frame);
}

// OK
void Channel_PV::Thunk_Sensor_Mode2(MediaFrameReference const& frame, void* self)
{
    static_cast<Channel_PV*>(self)->OnFrameArrived_Mode2(frame);
}

// OK
void Channel_PV::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_PV*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_PV::OnFrameArrived_Mode0(MediaFrameReference const& frame)
{
    PV_Metadata p;

    if (m_counter == 0)
    {
    auto intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    auto metadata   = frame.Properties().Lookup(MFSampleExtension_CaptureMetadata).as<IMapView<winrt::guid, IInspectable>>();

    p.f                     = intrinsics.FocalLength();
    p.c                     = intrinsics.PrincipalPoint();
    p.exposure_time         = metadata.Lookup(MF_CAPTURE_METADATA_EXPOSURE_TIME).as<uint64_t>();
    p.exposure_compensation = *reinterpret_cast<uint64x2*>(metadata.Lookup(MF_CAPTURE_METADATA_EXPOSURE_COMPENSATION).as<IReferenceArray<uint8_t>>().Value().begin());
    p.iso_speed             = metadata.Lookup(MF_CAPTURE_METADATA_ISO_SPEED).as<uint32_t>();
    p.iso_gains             = *reinterpret_cast<float2*>(metadata.Lookup(MF_CAPTURE_METADATA_ISO_GAINS).as<IReferenceArray<uint8_t>>().Value().begin());
    p.lens_position         = metadata.Lookup(MF_CAPTURE_METADATA_LENS_POSITION).as<uint32_t>();
    p.focus_state           = metadata.Lookup(MF_CAPTURE_METADATA_FOCUSSTATE).as<uint32_t>();
    p.white_balance         = metadata.Lookup(MF_CAPTURE_METADATA_WHITEBALANCE).as<uint32_t>();
    p.white_balance_gains   = *reinterpret_cast<float3*>(metadata.Lookup(MF_CAPTURE_METADATA_WHITEBALANCE_GAINS).as<IReferenceArray<uint8_t>>().Value().begin());
    p.resolution            = (static_cast<uint32_t>(m_height) << 16) | static_cast<uint32_t>(m_width);
    p.timestamp             = frame.SystemRelativeTime().Value().count();
    p.pose                  = PersonalVideo_GetFrameWorldPose(frame);

    m_pEncoder->WriteSample(frame, &p);
    }
    m_counter = (m_counter + 1) % m_divisor;
}

// OK
void Channel_PV::OnFrameArrived_Mode2(MediaFrameReference const& frame)
{
    if (WaitForSingleObject(m_event_client, 0) == WAIT_TIMEOUT)
    {
    auto intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    auto extrinsics = frame.Properties().Lookup(MFSampleExtension_CameraExtrinsics).as<IReferenceArray<uint8_t>>().Value();
    auto additional = frame.Format().Properties().Lookup(winrt::guid("86b6adbb-3735-447d-bee5-6fc23cb58d4a")).as<IReferenceArray<uint8_t>>().Value();

    float* mf_intrinsics = reinterpret_cast<float*>(additional.begin()) + 3;
    float* mf_extrinsics = reinterpret_cast<float*>(extrinsics.begin()) + 5;

    m_calibration.f             = intrinsics.FocalLength();
    m_calibration.c             = intrinsics.PrincipalPoint();
    m_calibration.r             = intrinsics.RadialDistortion();
    m_calibration.t             = intrinsics.TangentialDistortion();
    m_calibration.p             = intrinsics.UndistortedProjectionTransform();
    m_calibration.extrinsics    = make_float4x4_translation(-(*reinterpret_cast<float3*>(mf_extrinsics + 0))) * make_float4x4_from_quaternion(conjugate(*reinterpret_cast<quaternion*>(mf_extrinsics + 3)));
    m_calibration.intrinsics_mf = *reinterpret_cast<float4*>(mf_intrinsics);
    m_calibration.extrinsics_mf = *reinterpret_cast<float7*>(mf_extrinsics);

    SetEvent(m_event_client);
    }
}

// OK
void Channel_PV::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata_size;

    ULONG const embed_size = sizeof(PV_Metadata) - sizeof(PV_Metadata::timestamp) - sizeof(PV_Metadata::pose);

    PV_Metadata* p = static_cast<PV_Metadata*>(metadata);
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
void Channel_PV::Execute_Mode0(bool enable_location)
{
    MediaFrameReaderAcquisitionMode acquisition_mode;
    H26xFormat format;
    std::vector<uint64_t> options;
    bool ok;

    if (!PersonalVideo_Status()) { return; }

    ok = ReceiveH26xFormat_Video(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = PersonalVideo_SetFormat(format.width, format.height, format.framerate);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Divisor(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveH26xFormat_Profile(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = ReceiveEncoderOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    TranslateEncoderOptions(options, acquisition_mode);

    m_pEncoder        = std::make_unique<Encoder_PV>(Thunk_Encoder, this, VideoSubtype::VideoSubtype_NV12, format, PersonalVideo_GetStride(format.width), options);
    m_enable_location = enable_location;
    m_counter         = 0;
    m_divisor         = format.divisor;
    m_width           = format.width;
    m_height          = format.height;

    PersonalVideo_ExecuteSensorLoop(acquisition_mode, Thunk_Sensor_Mode0, this, m_event_client);

    m_pEncoder.reset();
}

// OK
void Channel_PV::Execute_Mode2()
{
    H26xFormat format;
    WSABUF wsaBuf[1];
    bool ok;

    if (!PersonalVideo_Status()) { return; }
    
    ok = ReceiveH26xFormat_Video(m_socket_client, m_event_client, format);
    if (!ok) { return; }

    ok = PersonalVideo_SetFormat(format.width, format.height, format.framerate);
    if (!ok) { return; }

    PersonalVideo_ExecuteSensorLoop(MediaFrameReaderAcquisitionMode::Realtime, Thunk_Sensor_Mode2, this, m_event_client);

    pack_buffer(wsaBuf, 0, &m_calibration, sizeof(m_calibration));

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
Channel_PV::Channel_PV(char const* name, char const* port, uint32_t id) : 
Channel(name, port, id)
{
}

// OK
bool Channel_PV::Startup()
{
    SetNoDelay(true);
    return true;
}

// OK
void Channel_PV::Run()
{
    MRCVideoOptions options;
    uint8_t mode;    
    bool ok;

    ok = ReceiveOperatingMode(m_socket_client, m_event_client, mode);
    if (!ok) { return; }

    if (mode & 4)
    {
    ok = ReceiveMRCVideoOptions(m_socket_client, m_event_client, options);
    if (!ok) { return; }

    if (PersonalVideo_Status()) { PersonalVideo_Close(); }

    PersonalVideo_Open(options);
    }

    switch (mode & 3)
    {
    case 0: Execute_Mode0(false); break;
    case 1: Execute_Mode0(true);  break;
    case 2: Execute_Mode2();      break;
    }

    if (mode & 8) 
    { 
    if (PersonalVideo_Status()) { PersonalVideo_Close(); }
    }    
}

// OK
void Channel_PV::Cleanup()
{
}

// OK
void PV_Startup()
{
    g_channel = std::make_unique<Channel_PV>("PV", PORT_NAME_PV, PORT_ID_PV);
}

// OK
void PV_Cleanup()
{
    g_channel.reset();
}
