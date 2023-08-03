
#include "research_mode.h"
#include "server.h"
#include "stream_rm.h"
#include "stream_rm_vlc.h"
#include "stream_rm_imu.h"
#include "stream_rm_zab.h"
#include "log.h"
#include "ports.h"
#include "types.h"

#include "hl2ss_network.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2VideoStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

typedef void(*RM_MODE0)(IResearchModeSensor*, SOCKET);
typedef void(*RM_MODE1)(IResearchModeSensor*, SOCKET, SpatialLocator const&);
typedef void(*RM_MODE2)(IResearchModeSensor*, SOCKET);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------


static HANDLE g_event_quit = NULL; // CloseHandle
static std::vector<HANDLE> g_threads; // CloseHandle

static HC_Context_Ptr g_zenoh_context;
static RMStreamConfig g_rm_stream_config = RMStreamConfig{};
static bool g_first_frame_sent = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------


// OK
static DWORD WINAPI RM_EntryPoint(void* param)
{
    SpatialLocator locator = nullptr;
    IResearchModeSensor* sensor;
    ResearchModeSensorType type;
    GUID nodeId;
    bool ok;

    sensor = (IResearchModeSensor*)param;
    type = sensor->GetSensorType();
    std::wstring sensor_name(sensor->GetFriendlyName());
    SPDLOG_INFO("RM{0} ({1}): Waiting for consent", (int)type, wide_string_to_string(sensor_name));

    ok = ResearchMode_WaitForConsent(sensor);
    if (!ok) { 
        SPDLOG_INFO("Consent was rejected");
        return false; 
    }

    nodeId = ResearchMode_GetRigNodeId();
    locator = SpatialGraphInteropPreview::CreateLocatorForNode(nodeId);

    SPDLOG_INFO("RM{0}: Start Stream", (int)type);
    auto& ctx = *g_zenoh_context;

    bool enable_location = g_rm_stream_config.enable_location;
    switch (type)
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      
    {
        if (enable_location) {
            RM_VLC_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_rm_stream_config.vlc_format, locator, g_zenoh_context->should_exit);
        }
        else {
            RM_VLC_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_rm_stream_config.vlc_format, g_zenoh_context->should_exit);
        }
        break;
    }
    case DEPTH_AHAT:       
    {
        if (enable_location) {
            RM_ZHT_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_rm_stream_config.depth_format, locator, g_zenoh_context->should_exit);
        }
        else {
            RM_ZHT_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_rm_stream_config.depth_format, g_zenoh_context->should_exit);
        }
        break;
    }
    case DEPTH_LONG_THROW:
    {
        if (enable_location) {
            RM_ZLT_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), locator, g_zenoh_context->should_exit);
        }
        else {
            RM_ZLT_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_zenoh_context->should_exit);
        }
        break;
    }
    case IMU_ACCEL:
    {
        if (enable_location) {
            RM_ACC_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), locator, g_zenoh_context->should_exit);
        }
        else {
            RM_ACC_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_zenoh_context->should_exit);
        }
        break;
    }
    case IMU_GYRO:
    {
        if (enable_location) {
            RM_GYR_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), locator, g_zenoh_context->should_exit);
        }
        else {
            RM_GYR_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_zenoh_context->should_exit);
        }
        break;
    }
    case IMU_MAG:
    {
        if (enable_location) {
            RM_MAG_Stream_Mode1(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), locator, g_zenoh_context->should_exit);
        }
        else {
            RM_MAG_Stream_Mode0(sensor, z_loan(ctx.session), ctx.topic_prefix.c_str(), g_zenoh_context->should_exit);
        }
        break;
    }
    }

    SPDLOG_INFO("RM{0}: Stream Finished", (int)type);
    //while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);
  
    return 0;
}

// OK
void RM_Initialize(HC_Context_Ptr& context, RMStreamConfig config)
{
    g_first_frame_sent = false;
    //ResearchModeSensorType const* sensortypes = ResearchMode_GetSensorTypes();
    int const sensorcount = ResearchMode_GetSensorTypeCount();

    if (sensorcount < 7) {
        // not on hololens2 ...
        config.enable_imu_accel = false;
        config.enable_imu_gyro = false;
        config.enable_imu_mag = false;
    }

    g_zenoh_context = context;
    g_rm_stream_config = std::move(config);

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    
    // check here if there might be a problem with the config (see readme about impossible combinations, like depth_ahat and depth_long_throw

    if (config.enable_left_front) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(LEFT_FRONT), NULL, NULL));
    }
    if (config.enable_left_left) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(LEFT_LEFT), NULL, NULL));
    }
    if (config.enable_right_front) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(RIGHT_FRONT), NULL, NULL));
    }
    if (config.enable_right_right) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(RIGHT_RIGHT), NULL, NULL));
    }
    if (config.enable_depth_ahat) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(DEPTH_AHAT), NULL, NULL));
    }
    if (config.enable_depth_long_throw) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(DEPTH_LONG_THROW), NULL, NULL));
    }
    if (config.enable_imu_accel) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(IMU_ACCEL), NULL, NULL));
    }
    if (config.enable_imu_gyro) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(IMU_GYRO), NULL, NULL));
    }
    if (config.enable_imu_mag) {
        g_threads.push_back(CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(IMU_MAG), NULL, NULL));
    }

}

// OK
void RM_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void RM_Cleanup()
{
    WaitForMultipleObjects((DWORD)g_threads.size(), g_threads.data(), TRUE, INFINITE);

    for (auto thread : g_threads) { CloseHandle(thread); }
    CloseHandle(g_event_quit);

    g_threads.clear();
    g_event_quit = NULL;

    g_zenoh_context.reset();
    g_first_frame_sent = false;
}
