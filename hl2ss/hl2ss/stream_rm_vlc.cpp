
#include <mfapi.h>
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"
#include "ipc_sc.h"
#include "types.h"
#include "log.h"
#include <chrono>

#include "hl2ss_network.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2VideoStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"


#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<bool ENABLE_LOCATION>
void RM_VLC_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    float4x4 pose;
    HookCallbackSocket* user;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);

    pBuffer->Lock(&pBytes, NULL, &cbData);


    // serialization

    // can we cache them so that we do not allocate new memory every image ?
    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer);

    pcpd_msgs::msg::Hololens2VideoStream value{};

    {
        using namespace std::chrono;
        auto ts_ = nanoseconds(sampletime * 100);
        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

        value.header().stamp().sec(ts_sec);
        value.header().stamp().nanosec(ts_nsec);

        value.header().frame_id(user->client_id);
    }

    if constexpr (ENABLE_LOCATION)
    {
        pSample->GetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(pose), NULL);
        // add flag to note it is enabled?
        float3 scale;
        quaternion rotation;
        float3 translation;

        if (decompose(pose, &scale, &rotation, &translation)) {
            value.position().x(translation.x);
            value.position().y(translation.y);
            value.position().z(translation.z);

            value.orientation().x(rotation.x);
            value.orientation().y(rotation.y);
            value.orientation().z(rotation.z);
            value.orientation().w(rotation.w);
        }
    }

    value.image_bytes(cbData);

    // this allocates and copies the buffer .. is there another way?
    std::vector<uint8_t> bsbuf(cbData);
    bsbuf.assign(pBytes, pBytes + cbData);
    value.image(std::move(bsbuf));


    buffer_cdr.reset();
    value.serialize(buffer_cdr);

    if (z_publisher_put(user->publisher, (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(user->options))) {
        ShowMessage("RM_VLC: Error publishing message");
        SetEvent(user->clientevent);
    }
    else {
        //ShowMessage("PV: published frame");
    }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
template <bool ENABLE_LOCATION>
void RM_VLC_Stream(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format, SpatialLocator const& locator)
{

    std::string sub_path;
    pcpd_msgs::msg::Hololens2StreamDescriptor desc{};

    switch (sensor->GetSensorType()) {
    case LEFT_FRONT:
        sub_path = "rm/vlc_lf/";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_LEFT_FRONT);
        break;
    case LEFT_LEFT:
        sub_path = "rm/vlc_ll/";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_LEFT_LEFT);
        break;
    case RIGHT_FRONT:
        sub_path = "rm/vlc_rf/";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_RIGHT_FRONT);
        break;
    case RIGHT_RIGHT:
        sub_path = "rm/vlc_rr/";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_RIGHT_RIGHT);
        break;
    default:
        ShowMessage("RM_VLC: invalid config");
        return;
    }

    desc.image_width(RM_VLC_WIDTH);
    desc.image_height(RM_VLC_HEIGHT);
    desc.frame_rate(RM_VLC_FPS);

    desc.image_format(pcpd_msgs::msg::PixelFormat_L8);
    switch (format.profile) {
    case H26xProfile_None:
        desc.h26x_profile(pcpd_msgs::msg::H26xProfile_None);
        desc.image_compression(pcpd_msgs::msg::CompressionType_Raw);
        desc.image_step(RM_VLC_WIDTH * 2);
        break;
    case H264Profile_Base:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_Base);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_VLC_WIDTH * 2 / 3);
        break;
    case H264Profile_Main:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_Main);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_VLC_WIDTH * 2 / 3);
        break;
    case H264Profile_High:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_High);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_VLC_WIDTH * 2 / 3);
        break;
    case H265Profile_Main:
        desc.h26x_profile(pcpd_msgs::msg::H265Profile_Main);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_VLC_WIDTH * 2 / 3);
        break;

    }

    std::string keyexpr = "hl2/sensor/" + sub_path + std::string(client_id);
    desc.stream_topic(keyexpr);

    // publish streamdescriptor
    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer);

    // put message to zenoh
    {
        buffer_cdr.reset();
        desc.serialize(buffer_cdr);

        std::string keyexpr1 = "hl2/cfg/" + sub_path + std::string(client_id);
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            ShowMessage("RM_VLC: Error putting info");
        }
        else {
            ShowMessage("RM_VLC: put info");
        }
    }

    ShowMessage("RM_VLC: publish on: %s", keyexpr.c_str());

    z_publisher_options_t publisher_options = z_publisher_options_default();
    publisher_options.priority = Z_PRIORITY_REAL_TIME;

    z_owned_publisher_t pub = z_declare_publisher(session, z_keyexpr(keyexpr.c_str()), &publisher_options);

    if (!z_check(pub)) {
        ShowMessage("RM_VLC: Error creating publisher");
        return;
    }

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);


    uint32_t const width      = RM_VLC_WIDTH;
    uint32_t const height     = RM_VLC_HEIGHT;
    uint32_t const framerate  = RM_VLC_FPS;
    uint32_t const lumasize   = width * height;
    LONGLONG const duration   = HNS_BASE / framerate;
    uint8_t  const zerochroma = 0x80;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorVLCFrame* pVLCFrame; // Release
    BYTE const* pImage;
    size_t length;
    BYTE* pDst;
    CustomMediaSink* pSink; // Release
    IMFSinkWriter* pSinkWriter; // Release
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    DWORD dwVideoIndex;
    HookCallbackSocket user;
    HANDLE clientevent; // CloseHandle
    uint32_t chromasize;
    uint32_t framebytes;
    HRESULT hr;
    
    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.publisher = z_loan(pub);
    user.clientevent = clientevent;
    user.data_profile = format.profile;
    user.options = options;
    user.client_id = client_id;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: chromasize = 0;            CreateSinkWriterL8ToL8(    &pSink, &pSinkWriter, &dwVideoIndex, format, RM_VLC_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            chromasize = lumasize / 2; CreateSinkWriterNV12ToH26x(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_VLC_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    framebytes = lumasize + chromasize;

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    pVLCFrame->GetBuffer(&pImage, &length);

    MFCreateMemoryBuffer(framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    memcpy(pDst, pImage, lumasize);
    memset(pDst + lumasize, zerochroma, chromasize);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(framebytes);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);
    pSample->SetSampleTime(timestamp.HostTicks);

    if constexpr(ENABLE_LOCATION)
    {
    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(float4x4));
    }

    pSinkWriter->WriteSample(dwVideoIndex, pSample);

    pVLCFrame->Release();
    pSensorFrame->Release();
    pSample->Release();
    pBuffer->Release();
    }
    while (WaitForSingleObject(clientevent, 0) == WAIT_TIMEOUT);

    sensor->CloseStream();

    pSinkWriter->Flush(dwVideoIndex);
    pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    z_undeclare_publisher(z_move(pub));

    CloseHandle(clientevent);
}

// OK
void RM_VLC_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format)
{
    RM_VLC_Stream<false>(sensor, session, client_id, format, nullptr);
}

// OK
void RM_VLC_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format, SpatialLocator const& locator)
{
    RM_VLC_Stream<true>(sensor, session, client_id, format, locator);
}

// OK
//void RM_VLC_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
//{
//    std::vector<float> uv2x;
//    std::vector<float> uv2y;
//    std::vector<float> mapx;
//    std::vector<float> mapy;
//    float K[4];
//    DirectX::XMFLOAT4X4 extrinsics;
//    WSABUF wsaBuf[6];
//
//    ResearchMode_GetIntrinsics(sensor, uv2x, uv2y, mapx, mapy, K);
//    ResearchMode_GetExtrinsics(sensor, extrinsics);
//
//    wsaBuf[0].buf = (char*)uv2x.data();
//    wsaBuf[0].len = (ULONG)(uv2x.size() * sizeof(float));
//    
//    wsaBuf[1].buf = (char*)uv2y.data();
//    wsaBuf[1].len = (ULONG)(uv2y.size() * sizeof(float));
//    
//    wsaBuf[2].buf = (char*)extrinsics.m;
//    wsaBuf[2].len = sizeof(extrinsics.m);
//
//    wsaBuf[3].buf = (char*)mapx.data();
//    wsaBuf[3].len = (ULONG)(mapx.size() * sizeof(float));
//
//    wsaBuf[4].buf = (char*)mapy.data();
//    wsaBuf[4].len = (ULONG)(mapy.size() * sizeof(float));
//
//    wsaBuf[5].buf = (char*)K;
//    wsaBuf[5].len = sizeof(K);
//
//    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
//}
