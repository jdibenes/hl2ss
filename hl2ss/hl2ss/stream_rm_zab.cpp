
#include <mfapi.h>
#include <MemoryBuffer.h>
#include "research_mode.h"
#include "server.h"
#include "locator.h"
#include "timestamps.h"
#include "neon.h"
#include "ipc_sc.h"
#include "log.h"
#include <chrono>

#include "zenoh.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2VideoStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

#define ZDEPTH_BUILD_STATIC
#include "zdepth.hpp"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>

using namespace Windows::Foundation;

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Storage;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;

typedef void(*ZHT_KERNEL)(u16 const*, u16 const*, u8*);

// Notes
// https://github.com/microsoft/HoloLens2ForCV/issues/133
// https://github.com/microsoft/HoloLens2ForCV/issues/142

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// ZHT ************************************************************************

// OK
static void RM_ZHT_Stack(u16 const* pDepth, u16 const* pAb, u8 *out)
{
    int const block = RM_ZHT_WIDTH * RM_ZHT_HEIGHT * sizeof(u16);
    
    memcpy(out,         pDepth, block);
    memcpy(out + block, pAb,    block);

}

// OK
template<bool ENABLE_LOCATION>
void RM_ZHT_SendSampleToSocket(IMFSample* pSample, void* param)
{
    IMFMediaBuffer *pBuffer; // Release
    HookCallbackSocket* user;
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    float4x4 pose;    

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
        ShowMessage("RM_ZHT: Error publishing message");
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
void RM_ZHT_Stream(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format, SpatialLocator const& locator)
{

    std::string sub_path;
    pcpd_msgs::msg::Hololens2StreamDescriptor desc{};

    switch (sensor->GetSensorType()) {
    case DEPTH_AHAT:
        sub_path = "rm/zht/";
        desc.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_DEPTH_AHAT);
        break;
    default:
        ShowMessage("RM_ZHT: invalid config");
        return;
    }

    std::string keyexpr = "hl2/sensor/" + sub_path + std::string(client_id);
    desc.stream_topic(keyexpr);

    desc.image_width(RM_ZHT_WIDTH);
    desc.image_height(RM_ZHT_HEIGHT);
    desc.frame_rate(RM_ZHT_FPS);

    desc.image_format(pcpd_msgs::msg::PixelFormat_L16);
    switch (format.profile) {
    case H26xProfile_None:
        desc.h26x_profile(pcpd_msgs::msg::H26xProfile_None);
        desc.image_compression(pcpd_msgs::msg::CompressionType_Raw);
        desc.image_step(RM_ZHT_WIDTH * 2 * 2);
        break;
    case H264Profile_Base:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_Base);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_ZHT_WIDTH * 3 / 2);
        break;
    case H264Profile_Main:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_Main);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_ZHT_WIDTH * 3 / 2);
        break;
    case H264Profile_High:
        desc.h26x_profile(pcpd_msgs::msg::H264Profile_High);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_ZHT_WIDTH * 3 / 2);
        break;
    case H265Profile_Main:
        desc.h26x_profile(pcpd_msgs::msg::H265Profile_Main);
        desc.image_compression(pcpd_msgs::msg::CompressionType_H26x);
        desc.image_step(RM_ZHT_WIDTH * 3 / 2);
        break;

    }

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
            ShowMessage("RM_ZHT: Error putting info");
        }
        else {
            ShowMessage("RM_ZHT: put info");
        }
    }


    ShowMessage("PV: publish on: %s", keyexpr.c_str());

    z_owned_publisher_t pub = z_declare_publisher(session, z_keyexpr(keyexpr.c_str()), NULL);

    if (!z_check(pub)) {
        ShowMessage("RM_ZHT: Error creating publisher");
        return;
    }

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);

    uint32_t const width      = RM_ZHT_WIDTH;
    uint32_t const height     = RM_ZHT_HEIGHT;
    uint32_t const framerate  = RM_ZHT_FPS;
    uint32_t const duration   = HNS_BASE / framerate;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    UINT16 const* pDepth;
    UINT16 const* pAbImage;
    size_t nDepthCount;
    size_t nAbCount;
    BYTE* pDst;
    CustomMediaSink* pSink; // Release
    IMFSinkWriter* pSinkWriter; // Release
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    DWORD dwVideoIndex;
    HookCallbackSocket user;
    HANDLE clientevent; // CloseHandle
    uint32_t framebytes;
    ZHT_KERNEL kernel;
    HRESULT hr;

    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.publisher = z_loan(pub);
    user.clientevent  = clientevent;
    user.data_profile = format.profile;
    user.options = options;
    user.client_id = client_id;

    switch (format.profile)
    {
    case H26xProfile::H26xProfile_None: kernel = RM_ZHT_Stack;   framebytes =  width * height * 2 * 2; CreateSinkWriterARGBToARGB(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_ZHT_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    default:                            kernel = Neon_ZHTToNV12; framebytes = (width * height * 3) / 2; CreateSinkWriterNV12ToH26x(&pSink, &pSinkWriter, &dwVideoIndex, format, RM_ZHT_SendSampleToSocket<ENABLE_LOCATION>, &user); break;
    }

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame);
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    MFCreateMemoryBuffer(framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    kernel(pDepth, pAbImage, pDst);
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

    pDepthFrame->Release();
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
void RM_ZHT_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format)
{
    RM_ZHT_Stream<false>(sensor, session, client_id, format, nullptr);
}

// OK
void RM_ZHT_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, H26xFormat format, SpatialLocator const& locator)
{
    RM_ZHT_Stream<true>(sensor, session, client_id, format, locator);
}

// OK
//void RM_ZHT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
//{
//    float const scale = 1000.0f;
//    float const alias = 1055.0f;
//
//    std::vector<float> uv2x;
//    std::vector<float> uv2y;
//    std::vector<float> mapx;
//    std::vector<float> mapy;
//    float K[4];
//    DirectX::XMFLOAT4X4 extrinsics;
//    WSABUF wsaBuf[8];
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
//    wsaBuf[2].buf = (char*)&extrinsics.m[0][0];
//    wsaBuf[2].len = sizeof(extrinsics.m);
//
//    wsaBuf[3].buf = (char*)&scale;
//    wsaBuf[3].len = sizeof(scale);
//
//    wsaBuf[4].buf = (char*)&alias;
//    wsaBuf[4].len = sizeof(alias);
//
//    wsaBuf[5].buf = (char*)mapx.data();
//    wsaBuf[5].len = (ULONG)(mapx.size() * sizeof(float));
//
//    wsaBuf[6].buf = (char*)mapy.data();
//    wsaBuf[6].len = (ULONG)(mapy.size() * sizeof(float));
//
//    wsaBuf[7].buf = (char*)K;
//    wsaBuf[7].len = sizeof(K);
//
//    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
//}

// ZLT ************************************************************************

// OK
template<bool ENABLE_LOCATION>
void RM_ZLT_Stream(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, SpatialLocator const& locator)
{

    std::string sub_path;
    pcpd_msgs::msg::Hololens2StreamDescriptor desc_depth{};
    pcpd_msgs::msg::Hololens2StreamDescriptor desc_ir{};

    switch (sensor->GetSensorType()) {
    case DEPTH_LONG_THROW:
        sub_path = "rm/zlt/";
        desc_depth.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_DEPTH_LONG_THROW);
        desc_ir.sensor_type(pcpd_msgs::msg::Hololens2SensorType::RM_DEPTH_LONG_THROW);
        break;
    default:
        ShowMessage("RM_ZLT: invalid config");
        return;
    }


    desc_depth.image_width(RM_ZLT_WIDTH);
    desc_depth.image_height(RM_ZLT_HEIGHT);
    desc_depth.image_step(RM_ZLT_WIDTH * 2);
    desc_depth.frame_rate(5); // between 1-5 fps .. how is it determined/configured?

    desc_ir.image_width(RM_ZLT_WIDTH);
    desc_ir.image_height(RM_ZLT_HEIGHT);
    desc_ir.image_step(RM_ZLT_WIDTH * 2);
    desc_ir.frame_rate(5); // between 1-5 fps .. how is it determined/configured?

    desc_depth.image_format(pcpd_msgs::msg::PixelFormat_L16);
    desc_depth.image_compression(pcpd_msgs::msg::CompressionType_Zdepth);

    desc_ir.image_format(pcpd_msgs::msg::PixelFormat_L16);
    desc_ir.image_compression(pcpd_msgs::msg::CompressionType_Png);

    std::string keyexpr_depth = "hl2/sensor/" + sub_path + "depth/" + std::string(client_id);
    std::string keyexpr_ir = "hl2/sensor/" + sub_path + "ir/" + std::string(client_id);
    desc_depth.stream_topic(keyexpr_depth);
    desc_ir.stream_topic(keyexpr_ir);

    // publish streamdescriptor
    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer);

    // put message to zenoh - depth
    {
        buffer_cdr.reset();
        desc_depth.serialize(buffer_cdr);

        std::string keyexpr1 = "hl2/cfg/" + sub_path + "depth/" + std::string(client_id);
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            ShowMessage("RM_ZLT_DEPTH: Error putting info");
        }
        else {
            ShowMessage("RM_ZLT_DEPTH: put info");
        }
    }

    // put message to zenoh - ir
    {
 /*       buffer_cdr.reset();
        desc_ir.serialize(buffer_cdr);

        std::string keyexpr1 = "hl2/cfg/" + sub_path + "ir/" + std::string(client_id);
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            ShowMessage("RM_ZLT_IR: Error putting info");
        }
        else {
            ShowMessage("RM_ZLT_IR: put info");
        }*/
    }

    // should put another message for extrinsics (need idl for it)

    ShowMessage("RM_ZLT_DEPTH: publish on: %s", keyexpr_depth.c_str());
    //ShowMessage("RM_ZLT_IR: publish on: %s", keyexpr_ir.c_str());

    z_owned_publisher_t pub_depth = z_declare_publisher(session, z_keyexpr(keyexpr_depth.c_str()), NULL);

    if (!z_check(pub_depth)) {
        ShowMessage("RM_ZLT_DEPTH: Error creating publisher");
        return;
    }

    //z_owned_publisher_t pub_ir = z_declare_publisher(session, z_keyexpr(keyexpr_ir.c_str()), NULL);

    //if (!z_check(pub_ir)) {
    //    ShowMessage("RM_ZLT_IR: Error creating publisher");
    //    return;
    //}

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);

    int const width  = RM_ZLT_WIDTH;
    int const height = RM_ZLT_HEIGHT;

    PerceptionTimestamp ts = nullptr;
    float4x4 pose;
    IResearchModeSensorFrame* pSensorFrame; // Release
    ResearchModeSensorTimestamp timestamp;
    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    BYTE const* pSigma;
    UINT16 const* pDepth;
    UINT16 const* pAbImage;
    size_t nSigmaCount;
    size_t nDepthCount;
    size_t nAbCount;
    //BYTE* pixelBufferData;
    //UINT32 pixelBufferDataLength;
    //BitmapPropertySet pngProperties;
    //PngFilterMode filter = PngFilterMode::Paeth; // temporary, we want zdepth later 
    //uint32_t streamSize;
    HRESULT hr;
    bool ok{ true };

    //pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(filter), winrt::Windows::Foundation::PropertyType::UInt8));


    zdepth::DepthCompressor compressor{};

    sensor->OpenStream();

    do
    {
    hr = sensor->GetNextBuffer(&pSensorFrame); // block
    if (FAILED(hr)) { break; }

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);


    if constexpr (ENABLE_LOCATION)
    {
        ts = QPCTimestampToPerceptionTimestamp(timestamp.HostTicks);
        pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
    }
    // process depth with zdepth - first filter only reliable depth measurements
    std::vector<uint16_t> temp_buffer(width*height);
    Neon_FilterDepth(pSigma, pDepth, &(temp_buffer[0]));

    std::vector<uint8_t> zdepth_buffer;
    // maybe later we can optimize this further to lower the bandwidth requirements..
    const bool is_keyframe{ true };
    if (compressor.Compress(width, height, &(temp_buffer[0]), zdepth_buffer, is_keyframe) == zdepth::DepthResult::Success) {
        // send depth buffer
        pcpd_msgs::msg::Hololens2VideoStream value{};

        {
            using namespace std::chrono;
            auto ts_ = nanoseconds(timestamp.HostTicks * 100);
            auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
            auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

            value.header().stamp().sec(ts_sec);
            value.header().stamp().nanosec(ts_nsec);

            value.header().frame_id(client_id);
        }

        if constexpr (ENABLE_LOCATION)
        {
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

        value.image_bytes(zdepth_buffer.size());
        value.image(std::move(zdepth_buffer));

        buffer_cdr.reset();
        value.serialize(buffer_cdr);

        if (z_publisher_put(z_loan(pub_depth), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(options))) {
            ShowMessage("RM_ZHT_DEPTH: Error publishing message");
            ok = false;
        }
        else {
            //ShowMessage("RM_ZHT_DEPTH: published frame");
        }

    }
    else {
        ShowMessage("RM_ZLT_DEPTH: Error compressing depth.");
    }



    // process ir with Png
    //auto softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Gray16, width, height);
    //{
    //auto bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    //auto spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();
    //spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    //memcpy(pixelBufferData, pAbImage, pixelBufferDataLength);
    ////Neon_ZLTToBGRA8(pSigma, pDepth, pAbImage, (u32*)pixelBufferData);
    //}

    //softwareBitmap = SoftwareBitmap::Convert(softwareBitmap, BitmapPixelFormat::Rgba16);

    //auto stream = InMemoryRandomAccessStream();
    //auto encoder = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, pngProperties).get();

    //encoder.SetSoftwareBitmap(softwareBitmap);
    //encoder.FlushAsync().get();

    //streamSize = (uint32_t)stream.Size();
    //auto streamBuf = Buffer(streamSize);

    //stream.ReadAsync(streamBuf, streamSize, InputStreamOptions::None).get();

    //{
    //    // serialization

    //    pcpd_msgs::msg::Hololens2VideoStream value{};

    //    {
    //        using namespace std::chrono;
    //        auto ts_ = nanoseconds(timestamp.HostTicks * 100);
    //        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
    //        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

    //        value.header().stamp().sec(ts_sec);
    //        value.header().stamp().nanosec(ts_nsec);

    //        value.header().frame_id(client_id);
    //    }

    //    if constexpr (ENABLE_LOCATION)
    //    {
    //        // add flag to note it is enabled?
    //        float3 scale;
    //        quaternion rotation;
    //        float3 translation;

    //        if (decompose(pose, &scale, &rotation, &translation)) {
    //            value.position().x(translation.x);
    //            value.position().y(translation.y);
    //            value.position().z(translation.z);

    //            value.orientation().x(rotation.x);
    //            value.orientation().y(rotation.y);
    //            value.orientation().z(rotation.z);
    //            value.orientation().w(rotation.w);

    //        }

    //    }

    //    value.data_size(streamSize);

    //    // this copies the buffer .. is there another way?
    //    std::vector<uint8_t> bsbuf(streamSize);
    //    bsbuf.assign(streamBuf.data(), streamBuf.data() + streamSize);
    //    value.data(std::move(bsbuf));


    //    buffer_cdr.reset();
    //    value.serialize(buffer_cdr);

    //    if (z_publisher_put(z_loan(pub_ir), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(options))) {
    //        ShowMessage("RM_ZHT_IR: Error publishing message");
    //        ok = false;
    //    }
    //    else {
    //        //ShowMessage("RM_ZHT_IR: published frame");
    //    }
    //}

    pDepthFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();

    z_undeclare_publisher(z_move(pub_depth));
    //z_undeclare_publisher(z_move(pub_ir));

}

// OK
void RM_ZLT_Stream_Mode0(IResearchModeSensor* sensor, z_session_t& session, const char* client_id)
{
    RM_ZLT_Stream<false>(sensor, session, client_id, nullptr);
}

// OK
void RM_ZLT_Stream_Mode1(IResearchModeSensor* sensor, z_session_t& session, const char* client_id, SpatialLocator const& locator)
{
    RM_ZLT_Stream<true>(sensor, session, client_id, locator);
}

// OK
//void RM_ZLT_Stream_Mode2(IResearchModeSensor* sensor, SOCKET clientsocket)
//{
//    float const scale = 1000.0f;
//
//    std::vector<float> uv2x;
//    std::vector<float> uv2y;
//    std::vector<float> mapx;
//    std::vector<float> mapy;
//    float K[4];
//    DirectX::XMFLOAT4X4 extrinsics;
//    WSABUF wsaBuf[7];
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
//    wsaBuf[2].buf = (char*)&extrinsics.m[0][0];
//    wsaBuf[2].len = sizeof(extrinsics.m);
//
//    wsaBuf[3].buf = (char*)&scale;
//    wsaBuf[3].len = sizeof(scale);
//
//    wsaBuf[4].buf = (char*)mapx.data();
//    wsaBuf[4].len = (ULONG)(mapx.size() * sizeof(float));
//
//    wsaBuf[5].buf = (char*)mapy.data();
//    wsaBuf[5].len = (ULONG)(mapy.size() * sizeof(float));
//
//    wsaBuf[6].buf = (char*)K;
//    wsaBuf[6].len = sizeof(K);
//
//    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
//}
