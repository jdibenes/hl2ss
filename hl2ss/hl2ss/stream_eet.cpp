
#include "server.h"
#include "locator.h"
#include "extended_eye_tracking.h"
#include "ports.h"
#include "timestamps.h"
#include "log.h"
#include <chrono>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Microsoft.MixedReality.EyeTracking.h>

#include "hl2ss_network.h"

#include "pcpd_msgs/msg/Hololens2EyeTracking.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Microsoft::MixedReality::EyeTracking;


struct EET_Frame
{
    float3   c_origin;
    float3   c_direction;
    float3   l_origin;
    float3   l_direction;
    float3   r_origin;
    float3   r_direction;
    float    l_openness;
    float    r_openness;
    float    vergence_distance;
    uint32_t valid;
};

struct EET_Packet
{
    uint64_t  timestamp;
    uint32_t  size;
    uint32_t  reserved;
    EET_Frame frame;
    float4x4  pose;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL;
static HANDLE g_thread = NULL;

static HC_Context_Ptr g_zenoh_context;
static uint8_t g_eye_tracker_fps = 30;
static bool g_first_frame_sent = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void EET_Stream(SpatialLocator const &locator, uint64_t utc_offset)
{

    if (!g_zenoh_context || !g_zenoh_context->valid) {
        SPDLOG_INFO("EET: Error invalid context");
        return;
    }
    std::string& topic_prefix = g_zenoh_context->topic_prefix;
    std::string keyexpr = topic_prefix + "/str/eet";
    SPDLOG_INFO("EET: publish on: {0}", keyexpr.c_str());

    z_publisher_options_t publisher_options = z_publisher_options_default();
    publisher_options.priority = Z_PRIORITY_REAL_TIME;

    z_owned_publisher_t pub = z_declare_publisher(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr.c_str()), &publisher_options);

    if (!z_check(pub)) {
        SPDLOG_INFO("EET: Error creating publisher");
        return;
    }

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);

    PerceptionTimestamp ts = nullptr;
    EyeGazeTrackerReading egtr = nullptr;
    DateTime td;
    EET_Packet eet_packet{};
    uint8_t fps = g_eye_tracker_fps;
    uint64_t delay;
    int64_t max_delta;
    int fps_index;
    bool cg_valid{ false };
    bool lg_valid{ false };
    bool rg_valid{ false };
    bool lo_valid{ false };
    bool ro_valid{ false };
    bool vd_valid{ false };
    bool ec_valid{ false };
    bool ok{ true };

    switch (fps)
    {
    case 30: fps_index = 0; break;
    case 60: fps_index = 1; break;
    case 90: fps_index = 2; break;
    default: 
        SPDLOG_INFO("EET: invalid fps");
        return;
    }

    max_delta = HNS_BASE / fps;
    delay = max_delta * 1;

    ExtendedEyeTracking_SetTargetFrameRate(fps_index);

    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

    {
        pcpd_msgs::msg::Hololens2StreamDescriptor value{};

        value.stream_topic(g_zenoh_context->topic_prefix + "/str/eet/");
        value.sensor_type(pcpd_msgs::msg::Hololens2SensorType::EYE_TRACKING);
        value.frame_rate(fps);

        buffer_cdr.reset();        buffer_cdr.serialize_encapsulation();
        value.serialize(buffer_cdr);

        // put message to zenoh
        std::string keyexpr1 = g_zenoh_context->topic_prefix + "/cfg/eet";
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_INFO("EET: Error putting info");
        }
        else {
            SPDLOG_INFO("EET: put info");
        }
    }

    if (!g_first_frame_sent) {
        g_first_frame_sent = true;

        pcpd_msgs::msg::Hololens2StreamDescriptor value{};

        value.stream_topic(g_zenoh_context->topic_prefix + "/str/eet");
        value.sensor_type(pcpd_msgs::msg::Hololens2SensorType::EYE_TRACKING);
        value.frame_rate(g_eye_tracker_fps);

        eprosima::fastcdr::FastBuffer buffer{};
        eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

        buffer_cdr.reset();
        value.serialize(buffer_cdr);

        // put message to zenoh
        std::string keyexpr1 = g_zenoh_context->topic_prefix + "/cfg/eet";
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_INFO("EET: Error putting info");
        }
        else {
            SPDLOG_INFO("EET: put info: {}", keyexpr1);
        }

    }



    do
    {

        eet_packet.timestamp = GetCurrentUTCTimestamp() - delay;

        Sleep(1000 / fps);
    
        egtr = ExtendedEyeTracking_GetReading(DateTime(QPCTimestampToTimeSpan(eet_packet.timestamp)), max_delta);
        bool got_data{ false };
        if (egtr)
        {
            eet_packet.timestamp = egtr.Timestamp().time_since_epoch().count() - utc_offset;

            cg_valid = egtr.TryGetCombinedEyeGazeInTrackerSpace(eet_packet.frame.c_origin, eet_packet.frame.c_direction);
            lg_valid = egtr.TryGetLeftEyeGazeInTrackerSpace(eet_packet.frame.l_origin, eet_packet.frame.l_direction);
            rg_valid = egtr.TryGetRightEyeGazeInTrackerSpace(eet_packet.frame.r_origin, eet_packet.frame.r_direction);
            lo_valid = egtr.TryGetLeftEyeOpenness(eet_packet.frame.l_openness);
            ro_valid = egtr.TryGetRightEyeOpenness(eet_packet.frame.r_openness);
            vd_valid = egtr.TryGetVergenceDistance(eet_packet.frame.vergence_distance);
            ec_valid = egtr.IsCalibrationValid();

            eet_packet.frame.valid = (vd_valid << 6) | (ro_valid << 5) | (lo_valid << 4) | (rg_valid << 3) | (lg_valid << 2) | (cg_valid << 1) | (ec_valid << 0);

            ts = QPCTimestampToPerceptionTimestamp(eet_packet.timestamp);
            eet_packet.pose = Locator_Locate(ts, locator, Locator_GetWorldCoordinateSystem(ts));
            got_data = true;
        }
        else
        {
            eet_packet.timestamp -= utc_offset;
            eet_packet.frame.valid = 0;
        }

        if (got_data) {

            // serialize to CDR
            pcpd_msgs::msg::Hololens2EyeTracking value{};

            {
                using namespace std::chrono;
                auto ts_ = nanoseconds(eet_packet.timestamp * 100);
                auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
                auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

                value.header().stamp().sec(ts_sec);
                value.header().stamp().nanosec(ts_nsec);

                value.header().frame_id(topic_prefix);
            }

            {
                geometry_msgs::msg::Vector3 v_o{};
                v_o.x(eet_packet.frame.c_origin.x);
                v_o.y(eet_packet.frame.c_origin.y);
                v_o.z(eet_packet.frame.c_origin.z);
                value.c_origin(v_o);

                geometry_msgs::msg::Vector3 v_d{};
                v_d.x(eet_packet.frame.c_direction.x);
                v_d.y(eet_packet.frame.c_direction.y);
                v_d.z(eet_packet.frame.c_direction.z);
                value.c_direction(v_d);
            }

            {
                geometry_msgs::msg::Vector3 v_o{};
                v_o.x(eet_packet.frame.l_origin.x);
                v_o.y(eet_packet.frame.l_origin.y);
                v_o.z(eet_packet.frame.l_origin.z);
                value.l_origin(v_o);

                geometry_msgs::msg::Vector3 v_d{};
                v_d.x(eet_packet.frame.l_direction.x);
                v_d.y(eet_packet.frame.l_direction.y);
                v_d.z(eet_packet.frame.l_direction.z);
                value.l_direction(v_d);
            }

            {
                geometry_msgs::msg::Vector3 v_o{};
                v_o.x(eet_packet.frame.r_origin.x);
                v_o.y(eet_packet.frame.r_origin.y);
                v_o.z(eet_packet.frame.r_origin.z);
                value.r_origin(v_o);

                geometry_msgs::msg::Vector3 v_d{};
                v_d.x(eet_packet.frame.r_direction.x);
                v_d.y(eet_packet.frame.r_direction.y);
                v_d.z(eet_packet.frame.r_direction.z);
                value.r_direction(v_d);
            }

            value.l_openness(eet_packet.frame.l_openness);
            value.r_openness(eet_packet.frame.r_openness);
            value.valid(eet_packet.frame.valid);

            buffer_cdr.reset();
            buffer_cdr.serialize_encapsulation();
            value.serialize(buffer_cdr);

            // send message to zenoh
            if (z_publisher_put(z_loan(pub), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options)) {
                ok = false;
                SPDLOG_INFO("EET: Error publishing message");
            }
            else {
                //SPDLOG_INFO("EET: published frame");
            }
        }
        else {
            //SPDLOG_INFO("EET: no data");
        }
    }
    while (ok && WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT && !g_zenoh_context->should_exit);


    z_undeclare_publisher(z_move(pub));

}

// OK
static DWORD WINAPI EET_EntryPoint(void* param)
{
    (void)param;

    if (!g_zenoh_context) {
        return 1;
    }

    SpatialLocator locator = nullptr;
    uint64_t utc_offset;
    
    SPDLOG_INFO("EET: Waiting for consent");

    ExtendedEyeTracking_Initialize();
    
    ExtendedEyeTracking_QueryCapabilities();

    locator = ExtendedEyeTracking_CreateLocator();
    utc_offset = GetQPCToUTCOffset(32);

    EET_Stream(locator, utc_offset);

    SPDLOG_INFO("EET: publisher done");

    return 0;
}

// OK
void EET_Initialize(HC_Context_Ptr& context, uint8_t fps)
{
    g_first_frame_sent = false;
    g_zenoh_context = context;
    g_eye_tracker_fps = fps;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, EET_EntryPoint, NULL, 0, NULL);
}

// OK
void EET_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void EET_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_quit = NULL;

    g_zenoh_context.reset();
    g_first_frame_sent = false;
}
