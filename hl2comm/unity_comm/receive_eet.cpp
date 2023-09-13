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

#include "zenoh.h"

#include "receive_eet.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Microsoft::MixedReality::EyeTracking;



struct subscriber_context;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit = NULL;

static HC_Context_Ptr g_zenoh_context;

static subscriber_context* g_subs_ctx = NULL;

static z_owned_subscriber_t g_sub;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

struct subscriber_context {
    const char* name{ nullptr };
    EETSubscriptionCallback callback;

    void init_context() {}

    void data_handler(const z_sample_t* sample) {
        using namespace std::chrono;

        z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);

        char* buffer_start = const_cast<char*>(reinterpret_cast<const char*>(sample->payload.start)); // not sure why we need to const_cast here .. we won't modify the buffer ..
        eprosima::fastcdr::FastBuffer cdrbuffer(buffer_start, sample->payload.len);
        eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
        cdr_des.read_encapsulation();

        pcpd_msgs::msg::Hololens2EyeTracking msg{};
        cdr_des >> msg;

        EET_Struct eet_struct;

        eet_struct.timestamp = duration_cast<nanoseconds>(seconds(msg.header().stamp().sec()) + nanoseconds(msg.header().stamp().nanosec())).count();
        eet_struct.position.x = msg.position().x();
        eet_struct.position.y = msg.position().y();
        eet_struct.position.z = msg.position().z();
        eet_struct.orientation.x = msg.orientation().x();
        eet_struct.orientation.y = msg.orientation().y();
        eet_struct.orientation.z = msg.orientation().z();
        eet_struct.orientation.w = msg.orientation().w();
        eet_struct.center_origin.x = msg.c_origin().x();
        eet_struct.center_origin.y = msg.c_origin().y();
        eet_struct.center_origin.z = msg.c_origin().z();
        eet_struct.center_direction.x = msg.c_direction().x();
        eet_struct.center_direction.y = msg.c_direction().y();
        eet_struct.center_direction.z = msg.c_direction().z();
        eet_struct.left_origin.x = msg.l_origin().x();
        eet_struct.left_origin.y = msg.l_origin().y();
        eet_struct.left_origin.z = msg.l_origin().z();
        eet_struct.left_direction.x = msg.l_direction().x();
        eet_struct.left_direction.y = msg.l_direction().y();
        eet_struct.left_direction.z = msg.l_direction().z();
        eet_struct.right_origin.x = msg.r_origin().x();
        eet_struct.right_origin.y = msg.r_origin().y();
        eet_struct.right_origin.z = msg.r_origin().z();
        eet_struct.right_direction.x = msg.r_direction().x();
        eet_struct.right_direction.y = msg.r_direction().y();
        eet_struct.right_direction.z = msg.r_direction().z();
        eet_struct.vergence_dist = msg.vergence_distance();
        eet_struct.valid = msg.valid();


        callback(&eet_struct);

        z_drop(z_move(keystr));
    }

    void teardown_context() {}
};


void eet_handle_subscriber_callback(const z_sample_t* sample, void* context) {
    auto handle = static_cast<subscriber_context*>(context);
    if (handle != nullptr && sample != nullptr) {
        try {
            handle->data_handler(sample);
        }
        catch (const std::exception& e) {
            SPDLOG_ERROR("Error during subscription callback for: {0} -> {1}", handle->name, e.what());
        }
    }
    else {
        SPDLOG_ERROR("Invalid handle or sample during subscription callback.");
    }
}

void eet_free_subscriber_context(void* context) {
    if (context != nullptr) {
        free(context);
    }
}


// OK
void Receive_EET_Initialize(HC_Context_Ptr& context, EETSubscriptionCallback cb, const char* topic)
{
    g_zenoh_context = context;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_subs_ctx = new subscriber_context();
    g_subs_ctx->name = "eye_tracking_subscriber";
    g_subs_ctx->callback = cb;

    // zclosure macro does not work with c++17
    z_owned_closure_sample_t callback{};
    callback.call = &eet_handle_subscriber_callback;
    callback.context = g_subs_ctx;
    callback.drop = eet_free_subscriber_context;

    auto options = z_subscriber_options_default();

    g_subs_ctx->init_context();

    //std::string expr = g_zenoh_context->topic_prefix + "/cfg/desc/eet";
    //printf("Declaring Subscriber on '%s'...\n", expr);
    g_sub = z_declare_subscriber(z_loan(g_zenoh_context->session), z_keyexpr(topic), z_move(callback), &options);
    if (!z_check(g_sub)) {
        printf("Unable to declare subscriber.\n");
        return;
    }
}

// OK
void Receive_EET_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void Receive_EET_Cleanup()
{
    CloseHandle(g_event_quit);

    g_event_quit = NULL;

    z_undeclare_subscriber(z_move(g_sub));

    if (g_subs_ctx != NULL) {
        g_subs_ctx->teardown_context();
    }

    g_zenoh_context.reset();
}