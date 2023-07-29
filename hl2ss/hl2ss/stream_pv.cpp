
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "personal_video.h"
#include "locator.h"
#include "log.h"
#include "ports.h"
#include "timestamps.h"
#include "ipc_sc.h"
#include <chrono>

#include "zenoh.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2H26xVideoStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;

struct PV_Projection
{
    uint16_t width;
    uint16_t height;
    float2 f;
    float2 c;
    float4x4 pose;
    float3 rd;
    float2 td;
};

struct PV_Context {
    std::string client_id;
    z_session_t session;
    H26xFormat format;
    bool enable_location{ true };
    bool valid{ false };
    bool first_frame_sent{ false };
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_thread = NULL; // CloseHandle
static bool g_reader_status = false;

// Mode: 0, 1
static IMFSinkWriter* g_pSinkWriter = NULL; // Release
static DWORD g_dwVideoIndex = 0;

// Mode: 2
//static HANDLE g_event_intrinsic = NULL; // alias
//static float g_intrinsics[2 + 2 + 3 + 2 + 16];

static PV_Context* g_zenoh_context = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<bool ENABLE_LOCATION>
void PV_OnVideoFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;
    
    CameraIntrinsics intrinsics = nullptr;
    MediaFrameReference frame = nullptr;
    IMFSample* pSample; // Release
    SoftwareBitmapBuffer* pBuffer; // Release
    PV_Projection pj;
    int64_t timestamp;

    if (!g_reader_status) { return; }
    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    timestamp = frame.SystemRelativeTime().Value().count();

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    pj.width = static_cast<uint16_t>(intrinsics.ImageWidth());
    pj.height = static_cast<uint16_t>(intrinsics.ImageHeight());
    pj.f = intrinsics.FocalLength();
    pj.c = intrinsics.PrincipalPoint();
    pj.rd = intrinsics.RadialDistortion();
    pj.td = intrinsics.TangentialDistortion();
    //intrinsics.UndistortedProjectionTransform();

    if constexpr(ENABLE_LOCATION)
    {
        pj.pose = Locator_GetTransformTo(frame.CoordinateSystem(), Locator_GetWorldCoordinateSystem(QPCTimestampToPerceptionTimestamp(timestamp)));
    }

    if (!g_zenoh_context->first_frame_sent) {
        g_zenoh_context->first_frame_sent = true;

        pcpd_msgs::msg::Hololens2StreamDescriptor desc{};

        desc.stream_topic("hl2/sensor/vid/pv/" + g_zenoh_context->client_id);
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::PERSONAL_VIDEO);
        desc.frame_rate(g_zenoh_context->format.framerate);
        desc.image_width(g_zenoh_context->format.width);
        desc.image_height(g_zenoh_context->format.height);

        // ignore for now - don't know how to set this for NV12...
        desc.image_step(0);

        switch (g_zenoh_context->format.profile) {
        case H26xProfile_None:
            desc.h26x_profile(pcpd_msgs::msg::H26xProfile_None);
            desc.image_compression(pcpd_msgs::msg::CompressionType_Raw);
            break;
        case H264Profile_Base:
            desc.h26x_profile(pcpd_msgs::msg::H264Profile_Base);
            desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
            break;
        case H264Profile_Main:
            desc.h26x_profile(pcpd_msgs::msg::H264Profile_Main);
            desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
            break;
        case H264Profile_High:
            desc.h26x_profile(pcpd_msgs::msg::H264Profile_High);
            desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
            break;
        case H265Profile_Main:
            desc.h26x_profile(pcpd_msgs::msg::H265Profile_Main);
            desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
            break;
        }

        desc.h26x_bitrate(g_zenoh_context->format.bitrate);

        if constexpr (ENABLE_LOCATION)
        {
            // add flag to note it is enabled?
            float3 scale;
            quaternion rotation;
            float3 translation;

            if (decompose(pj.pose, &scale, &rotation, &translation)) {
                desc.position().x(translation.x);
                desc.position().y(translation.y);
                desc.position().z(translation.z);

                desc.orientation().x(rotation.x);
                desc.orientation().y(rotation.y);
                desc.orientation().z(rotation.z);
                desc.orientation().w(rotation.w);
            }
        }

        eprosima::fastcdr::FastBuffer buffer{};
        eprosima::fastcdr::Cdr buffer_cdr(buffer);

        buffer_cdr.reset();
        desc.serialize(buffer_cdr);

        // put message to zenoh
        {
            std::string keyexpr1 = "hl2/cfg/vid/pv/" + g_zenoh_context->client_id;
            z_put_options_t options1 = z_put_options_default();
            options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
            int res = z_put(g_zenoh_context->session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
            if (res > 0) {
                ShowMessage("PV: Error putting info");
            }
            else {
                ShowMessage("PV: put info");
            }
        }

    }

    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pj, sizeof(pj));

    g_pSinkWriter->WriteSample(g_dwVideoIndex, pSample);

    pSample->Release();
    pBuffer->Release();
}

/*
// OK
static void PV_OnVideoFrameArrived_Intrinsics(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    CameraIntrinsics intrinsics = nullptr;
    MediaFrameReference frame = nullptr;
    float2 f;
    float2 c;
    float3 r;
    float2 t;
    float4x4 p;
    DWORD status;

    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    status = WaitForSingleObject(g_event_intrinsic, 0);
    if (status != WAIT_TIMEOUT) { return; }

    intrinsics = frame.VideoMediaFrame().CameraIntrinsics();
    f = intrinsics.FocalLength();
    c = intrinsics.PrincipalPoint();
    r = intrinsics.RadialDistortion();
    t = intrinsics.TangentialDistortion();
    p = intrinsics.UndistortedProjectionTransform();

    memcpy(&g_intrinsics[0], &f, sizeof(f));
    memcpy(&g_intrinsics[2], &c, sizeof(c));
    memcpy(&g_intrinsics[4], &r, sizeof(r));
    memcpy(&g_intrinsics[7], &t, sizeof(t));
    memcpy(&g_intrinsics[9], &p, sizeof(p));

    SetEvent(g_event_intrinsic);
}
*/


// OK
template<bool ENABLE_LOCATION>
void PV_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    PV_Projection pj;
    HookCallbackSocket* user;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);
    pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pj, sizeof(pj), NULL);

    pBuffer->Lock(&pBytes, NULL, &cbData);

    // serialization
    
    // can we cache them so that we do not allocate new memory every image ?
    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer);

    pcpd_msgs::msg::Hololens2H26xVideoStream value{};

    {
        using namespace std::chrono;
        auto ts_ = nanoseconds(sampletime * 100);
        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

        value.header().stamp().sec(ts_sec);
        value.header().stamp().nanosec(ts_nsec);

        value.header().frame_id(g_zenoh_context->client_id);
    }

    if constexpr (ENABLE_LOCATION)
    {
        // add flag to note it is enabled?
        float3 scale;
        quaternion rotation;
        float3 translation;

        if (decompose(pj.pose, &scale, &rotation, &translation)) {
            value.position().x(translation.x);
            value.position().y(translation.y);
            value.position().z(translation.z);

            value.orientation().x(rotation.x);
            value.orientation().y(rotation.y);
            value.orientation().z(rotation.z);
            value.orientation().w(rotation.w);

        }

    }
    value.camera_focal_length({ pj.f.x, pj.f.y});
    value.camera_principal_point({ pj.c.x, pj.c.y });
    value.camera_radial_distortion({ pj.rd.x, pj.rd.y, pj.rd.z});
    value.camera_tangential_distortion({ pj.td.x, pj.td.y});

    value.data_size(cbData);

    // this copies the buffer .. is there another way?
    std::vector<uint8_t> bsbuf(cbData);
    bsbuf.assign(pBytes, pBytes + cbData);
    value.data(std::move(bsbuf));


    buffer_cdr.reset();
    value.serialize(buffer_cdr);

    if (z_publisher_put(user->publisher, (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(user->options))) {
        ShowMessage("PV: Error publishing message");
        SetEvent(user->clientevent);
    }
    else {
        //ShowMessage("PV: published frame");
    }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
template<bool ENABLE_LOCATION>
void PV_Stream(HANDLE clientevent, MediaFrameReader const& reader, H26xFormat& format)
{

    if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
        ShowMessage("PV: Error invalid context");
        return;
    }

    std::string keyexpr = "hl2/sensor/vid/pv/" + g_zenoh_context->client_id;
    ShowMessage("PV: publish on: %s", keyexpr.c_str());

    z_owned_publisher_t pub = z_declare_publisher(g_zenoh_context->session, z_keyexpr(keyexpr.c_str()), NULL);

    if (!z_check(pub)) {
        ShowMessage("PV: Error creating publisher");
        return;
    }

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);


    CustomMediaSink* pSink; // Release
    HookCallbackSocket user;

    user.publisher = z_loan(pub);
    user.clientevent  = clientevent;
    user.data_profile = format.profile;
    user.options = options;
    user.client_id = g_zenoh_context->client_id.c_str();

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: CreateSinkWriterNV12ToNV12(&pSink, &g_pSinkWriter, &g_dwVideoIndex, format, PV_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            CreateSinkWriterNV12ToH26x(&pSink, &g_pSinkWriter, &g_dwVideoIndex, format, PV_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    reader.FrameArrived(PV_OnVideoFrameArrived<ENABLE_LOCATION>);

    g_reader_status = true;
    reader.StartAsync().get();
    WaitForSingleObject(clientevent, INFINITE);
    g_reader_status = false;
    reader.StopAsync().get();

    g_pSinkWriter->Flush(g_dwVideoIndex);
    g_pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    g_pSinkWriter = NULL;
    g_dwVideoIndex = 0;

    z_undeclare_publisher(z_move(pub));

}

/*
// OK
static void PV_Intrinsics(SOCKET clientsocket, HANDLE clientevent, MediaFrameReader const& reader)
{
    WSABUF wsaBuf;

    g_event_intrinsic = clientevent;

    reader.FrameArrived(PV_OnVideoFrameArrived_Intrinsics);

    reader.StartAsync().get();
    WaitForSingleObject(g_event_intrinsic, INFINITE);
    reader.StopAsync().get();

    wsaBuf.buf = (char*)g_intrinsics;
    wsaBuf.len = sizeof(g_intrinsics);

    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    g_event_intrinsic = NULL;
    memset(g_intrinsics, 0, sizeof(g_intrinsics));
}
*/

// OK
static void PV_Stream()
{

    if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
        ShowMessage("PV: Invalid Zenoh Context");
        return;
    }

    MediaFrameReader videoFrameReader = nullptr;
    HANDLE clientevent; // CloseHandle
    H26xFormat format = g_zenoh_context->format;
    bool ok;

    if (!PersonalVideo_Status()) { PersonalVideo_Open(); }
    if (!PersonalVideo_Status()) { 
        ShowMessage("PV: Error opening.");
        return; 
    }

    ok = PersonalVideo_SetFormat(format.width, format.height, format.framerate);
    if (!ok) { 
        ShowMessage("PV: Error setting format.");
        return; 
    }
    
    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    videoFrameReader = PersonalVideo_CreateFrameReader();
    videoFrameReader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);

    if (g_zenoh_context->enable_location) {
        PV_Stream<true>(clientevent, videoFrameReader, format);
    }
    else {
        PV_Stream<false>(clientevent, videoFrameReader, format);
    }

    videoFrameReader.Close();

    CloseHandle(clientevent);

    // maybe this should be configurable too?
    if (true) { PersonalVideo_Close(); }
}

// OK
static DWORD WINAPI PV_EntryPoint(void *param)
{
    (void)param;

    ShowMessage("PV: Start PV Stream");

    PV_Stream();

    //while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    ShowMessage("PV: Finished PV Stream");

    return 0;
}

// OK
void PV_Initialize(const char* client_id, z_session_t session, bool enable_location, H26xFormat format)
{

    g_zenoh_context = new PV_Context(); // release
    g_zenoh_context->client_id = std::string(client_id);
    g_zenoh_context->session = session;
    g_zenoh_context->enable_location = enable_location;
    g_zenoh_context->format = std::move(format);
    g_zenoh_context->valid = true;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, PV_EntryPoint, NULL, 0, NULL);
}

// OK
void PV_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void PV_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_quit);
    
    g_thread = NULL;
    g_event_quit = NULL;

    free(g_zenoh_context);
    g_zenoh_context = NULL;
}
