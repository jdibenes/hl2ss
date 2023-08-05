
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
    eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

    pcpd_msgs::msg::Hololens2VideoStream value{};

    {
        using namespace std::chrono;
        auto ts_ = nanoseconds(sampletime * 100);
        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

        value.header().stamp().sec(ts_sec);
        value.header().stamp().nanosec(ts_nsec);

        value.header().frame_id(user->topic_prefix);
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
    buffer_cdr.serialize_encapsulation();
    value.serialize(buffer_cdr);

    if (z_publisher_put(user->publisher, (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(user->options))) {
        SPDLOG_INFO("RM_VLC: Error publishing message");
        SetEvent(user->clientevent);
    }
    else {
        //SPDLOG_INFO("PV: published frame");
    }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
template <bool ENABLE_LOCATION>
void RM_VLC_Stream(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, H26xFormat format, SpatialLocator const& locator, const bool& should_exit)
{

    std::string sub_path;
    pcpd_msgs::msg::Hololens2StreamDescriptor desc{};

    switch (sensor->GetSensorType()) {
    case LEFT_FRONT:
        sub_path = "vlc_lf";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_LEFT_FRONT);
        break;
    case LEFT_LEFT:
        sub_path = "vlc_ll";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_LEFT_LEFT);
        break;
    case RIGHT_FRONT:
        sub_path = "vlc_rf";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_RIGHT_FRONT);
        break;
    case RIGHT_RIGHT:
        sub_path = "vlc_rr";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_RIGHT_RIGHT);
        break;
    default:
        SPDLOG_INFO("RM_VLC: invalid config");
        return;
    }

    desc.image_width(RM_VLC_WIDTH);
    desc.image_height(RM_VLC_HEIGHT);
    desc.frame_rate(RM_VLC_FPS);
    desc.h26x_bitrate(format.bitrate);

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

    std::string keyexpr = std::string(topic_prefix) + "/str/" + sub_path;
    desc.stream_topic(keyexpr);
    desc.calib_topic(std::string(topic_prefix) + "/cfg/cal/" + sub_path);


    SPDLOG_DEBUG("RM_VLC: Start stream with parameters: {0}x{1}@{2} [{3}] compression: {4} format: {5} bitrate: {6}",
        desc.image_width(), desc.image_height(),
        desc.frame_rate(), desc.image_step(),
        static_cast<int>(desc.image_compression()),
        static_cast<int>(desc.image_format()),
        desc.h26x_bitrate());

    // publish streamdescriptor
    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

    // put message to zenoh
    {
        buffer_cdr.reset();
        buffer_cdr.serialize_encapsulation();
        desc.serialize(buffer_cdr);

        std::string keyexpr1 = std::string(topic_prefix) + "/cfg/desc/" + sub_path;
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_INFO("RM_VLC: Error putting info");
        }
        else {
            SPDLOG_INFO("RM_VLC: put info: {}", keyexpr1);
        }
    }

    // publish calibration
    pcpd_msgs::msg::Hololens2SensorInfoVLC calib{};
    {
        using namespace std::chrono;
        auto ts_ = nanoseconds(GetCurrentQPCTimestamp() * 100);
        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

        calib.header().stamp().sec(ts_sec);
        calib.header().stamp().nanosec(ts_nsec);

        calib.header().frame_id(topic_prefix);
    }

    {
        std::vector<float> uv2x;
        std::vector<float> uv2y;
        std::vector<float> mapx;
        std::vector<float> mapy;
        float K[4];

        ResearchMode_GetIntrinsics(sensor, calib.uv2x(), calib.uv2y(), calib.mapx(), calib.mapy(), K);
        calib.K({K[0],K[1], K[2], K[3]});

        DirectX::XMFLOAT4X4 e;
        ResearchMode_GetExtrinsics(sensor, e);
        DirectX::FXMMATRIX e1 = XMLoadFloat4x4(&e);;

        float4x4 extrinsics;
        XMStoreFloat4x4(&extrinsics, e1);

        // add flag to note it is enabled?
        float3 scale;
        quaternion rotation;
        float3 translation;

        if (decompose(extrinsics, &scale, &rotation, &translation)) {
            calib.position().x(translation.x);
            calib.position().y(translation.y);
            calib.position().z(translation.z);

            calib.orientation().x(rotation.x);
            calib.orientation().y(rotation.y);
            calib.orientation().z(rotation.z);
            calib.orientation().w(rotation.w);
        }

    }

    // put message to zenoh
    {
        buffer_cdr.reset();
        buffer_cdr.serialize_encapsulation();
        calib.serialize(buffer_cdr);

        std::string keyexpr1 = std::string(topic_prefix) + "/cfg/cal/" + sub_path;
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_INFO("RM_VLC: Error putting calib");
        }
        else {
            SPDLOG_INFO("RM_VLC: put calib: {}", keyexpr1);
        }
    }

    SPDLOG_INFO("RM_VLC: publish on: {0}", keyexpr.c_str());

    z_publisher_options_t publisher_options = z_publisher_options_default();
    publisher_options.priority = Z_PRIORITY_REAL_TIME;

    z_owned_publisher_t pub = z_declare_publisher(session, z_keyexpr(keyexpr.c_str()), &publisher_options);

    if (!z_check(pub)) {
        SPDLOG_INFO("RM_VLC: Error creating publisher");
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
    user.topic_prefix = topic_prefix;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: chromasize = 0;            CreateSinkWriterL8ToL8(    &pSink, &pSinkWriter, &dwVideoIndex, format, RM_VLC_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            chromasize = lumasize / 2; CreateSinkWriterNV12ToH26x(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_VLC_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    framebytes = lumasize + chromasize;

    sensor->OpenStream();

    do
    {
        try {
            hr = sensor->GetNextBuffer(&pSensorFrame); // block
            if (FAILED(hr)) { break; }

            pSensorFrame->GetTimeStamp(&timestamp);
            pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

            pVLCFrame->GetBuffer(&pImage, &length);

            MFCreateMemoryBuffer(framebytes, &pBuffer);
            if (pBuffer) {
                pBuffer->Lock(&pDst, NULL, NULL);
                memcpy(pDst, pImage, lumasize);
                memset(pDst + lumasize, zerochroma, chromasize);
                pBuffer->Unlock();
                pBuffer->SetCurrentLength(framebytes);

                MFCreateSample(&pSample);

                pSample->AddBuffer(pBuffer);
                pSample->SetSampleDuration(duration);
                pSample->SetSampleTime(timestamp.HostTicks);

                if constexpr (ENABLE_LOCATION)
                {
                    ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
                    pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
                    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)&pose, sizeof(float4x4));
                }

                pSinkWriter->WriteSample(dwVideoIndex, pSample);

                pSample->Release();
                pBuffer->Release();

            }
            else {
                SPDLOG_ERROR("RM_VLC: Invalid Buffer received ..");
            }


            pVLCFrame->Release();
            pSensorFrame->Release();
        }
        catch (const std::exception& e) {
            SPDLOG_ERROR("RM_VLC: Error during frame processing: {0}", e.what());
        }
    }
    while (WaitForSingleObject(clientevent, 0) == WAIT_TIMEOUT && !should_exit);

    sensor->CloseStream();

    pSinkWriter->Flush(dwVideoIndex);
    pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    z_undeclare_publisher(z_move(pub));

    CloseHandle(clientevent);
}

// OK
void RM_VLC_Stream_Mode0(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, H26xFormat format, const bool& should_exit)
{
    RM_VLC_Stream<false>(sensor, session, topic_prefix, format, nullptr, should_exit);
}

// OK
void RM_VLC_Stream_Mode1(IResearchModeSensor* sensor, z_session_t session, const char* topic_prefix, H26xFormat format, SpatialLocator const& locator, const bool& should_exit)
{
    RM_VLC_Stream<true>(sensor, session, topic_prefix, format, locator, should_exit);
}
