
#include <mfapi.h>
#include "configuration.h"
#include "ipc.h"
#include "plugin.h"

#include "../hl2comm/server.h"
#include "../hl2comm/timestamps.h"
#include "../hl2comm/log.h"
#include "../hl2comm/nfo.h"
#include "../hl2comm/types.h"
#include "../hl2comm/locator.h"
#include "../hl2comm/research_mode.h"
#include "../hl2comm/spatial_input.h"
#include "../hl2comm/personal_video.h"
#include "../hl2comm/spatial_mapping.h"
#include "../hl2comm/scene_understanding.h"
#include "../hl2comm/voice_input.h"
#include "../hl2comm/stream_rm.h"
#include "../hl2comm/stream_mc.h"
#include "../hl2comm/stream_pv.h"
#include "../hl2comm/stream_si.h"
#include "../hl2comm/ipc_rc.h"
#include "../hl2comm/ipc_sm.h"
#include "../hl2comm/ipc_su.h"
#include "../hl2comm/ipc_vi.h"
#include "../hl2comm/stream_eet.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.ApplicationModel.Core.h>

#include <chrono>

#include "../hl2comm/hl2ss_network.h"
#include "../hl2comm/manager.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"


using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::ApplicationModel::Core;






// --------------------------------------------------------------------------
// Helper for Deferred Call

template<typename Func, typename... Args>
auto call_deferred(Func&& func, Args &&... args) {
    // capture our task into lambda with all its parameters
    auto capture = [func = std::forward<Func>(func),
        args = std::make_tuple(std::forward<Args>(args)...)]()
        mutable {
        return std::apply(std::move(func), std::move(args));
    };

    HANDLE event_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    CoreApplication::Views().GetAt(0).Dispatcher().RunAsync(CoreDispatcherPriority::High,
        [&capture, &event_done]() {
            capture();
            SetEvent(event_done);
        }
    ).get();
    WaitForSingleObject(event_done, INFINITE);
    CloseHandle(event_done);
    return true;
}


// --------------------------------------------------------------------------
// UnitySetInterfaces

static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

static IUnityInterfaces* s_UnityInterfaces = NULL;
static IUnityGraphics* s_Graphics = NULL;

static HC_Context_Ptr g_zenoh_context;

extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces * unityInterfaces)
{
    s_UnityInterfaces = unityInterfaces;
    s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
    s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);

#if SUPPORT_VULKAN
    if (s_Graphics->GetRenderer() == kUnityGfxRendererNull)
    {
        extern void NativeBufferAPI_Vulkan_OnPluginLoad(IUnityInterfaces*);
        NativeBufferAPI_Vulkan_OnPluginLoad(unityInterfaces);
    }
#endif // SUPPORT_VULKAN

    // Run OnGraphicsDeviceEvent(initialize) manually on plugin load
    OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
    if (s_Graphics) {
        s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
    }

    if (g_zenoh_context != nullptr) {
        MQ_Quit();
        MQ_Cleanup();
        StopManager(g_zenoh_context);
        g_zenoh_context.reset();
    }

}

// --------------------------------------------------------------------------
// GraphicsDeviceEvent


//static NativeBufferAPI* s_CurrentAPI = NULL;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;


static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
{
    // Create graphics API implementation upon initialization
    if (eventType == kUnityGfxDeviceEventInitialize)
    {
        //assert(s_CurrentAPI == NULL);
        s_DeviceType = s_Graphics->GetRenderer();
        //s_CurrentAPI = CreateNativeBufferAPI(s_DeviceType);
    }

    // Let the implementation process the device related events
    //if (s_CurrentAPI)
    //{
    //    s_CurrentAPI->ProcessDeviceEvent(eventType, s_UnityInterfaces);
    //}

    // Cleanup graphics API implementation upon shutdown
    if (eventType == kUnityGfxDeviceEventShutdown)
    {
        //delete s_CurrentAPI;
        //s_CurrentAPI = NULL;
        s_DeviceType = kUnityGfxRendererNull;
    }
}



// --------------------------------------------------------------------------
// OnRenderEvent
// This will be called for GL.IssuePluginEvent script calls; eventID will
// be the integer passed to IssuePluginEvent. In this example, we just ignore
// that value.

static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
{
    // Unknown / unsupported graphics device type? Do nothing
    //if (s_CurrentAPI == NULL)
    //    return;
    //s_CurrentAPI->OnRenderEvent(eventID);
}




//-----------------------------------------------------------------------------
// HL2Comm Interface
//-----------------------------------------------------------------------------

// from https://stackoverflow.com/questions/43732825/use-debug-log-from-c

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API 
RegisterLoggingCallback(LoggingFuncCallBack cb) {
    SetupCallbackLogSink(cb);
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
RegisterRawZSubscriber(const char* name, const char* keyexpr, ZenohSubscriptionCallBack cb) {
    return MQ_SetupZenohRawSubscription(name, keyexpr, cb);
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
ZSendMessage(const char* keyexpr, uint8_t * buffer, std::size_t buffer_len, z_encoding_prefix_t encoding, bool block) {
    return MQ_SendMessage(keyexpr, buffer, buffer_len, encoding, block);
}



// OK
void InitializeStreams(const char* _topic_prefix, const char* zcfg, uint32_t enable=0xFFFFFFFF)
{
    if (g_zenoh_context != nullptr) {
        // plugin is already initialized
        // we could check the diff between the enabled streams and adapt for it ??
        return;
    }

    MFStartup(MF_VERSION);

    Locator_Initialize();

    if (enable & HL2SS_ENABLE_RM) { ResearchMode_Initialize(); }
    if (enable & HL2SS_ENABLE_PV) { PersonalVideo_Initialize(); }
    if (enable & HL2SS_ENABLE_SI) { SpatialInput_Initialize(); }
    if (enable & HL2SS_ENABLE_SM) { SpatialMapping_Initialize(); }
    if (enable & HL2SS_ENABLE_SU) { SceneUnderstanding_Initialize(); }
    if (enable & HL2SS_ENABLE_VI) { VoiceInput_Initialize(); }

    // testing zenoh in uwp app
    z_owned_config_t config = z_config_default();
    if (strlen(zcfg) > 0) {
        // configure from zcfg
        config = zc_config_from_str(zcfg);
    }
    if (!z_check(config)) {
        // error
        return;
    }

    auto context = std::make_shared<HC_Context>();

    /*
     * Topic prefix consists of the following components
     * - namespace: tcn (Tum Camp Narvis)
     * - site: loc (default for unrouted networks)
     * - category: [hl2|svc|...] The category of the entry
     * - entity_id: the entity id, unique to the namespace/site parent
     *   (to avoid issues it is better to have globally unique ids.)
     * - comm_type: [str|cfg|rpc] either streaming, configuration or remote-procedure calls
     *
     */
    std::string default_topic_prefix{"tcn/loc/dev/"};

    std::string topic_prefix(_topic_prefix);
    if (topic_prefix.empty()) {
        std::string host_name;
        std::vector<wchar_t> buf;
        GetLocalHostname(buf);
        host_name.resize(buf.size(), '\0');
        std::copy(buf.begin(), buf.end(), host_name.begin());
        topic_prefix = default_topic_prefix + host_name;
    }

    context->topic_prefix = topic_prefix;
    SPDLOG_INFO("Using Topic Prefix: {0}", context->topic_prefix);

    g_zenoh_context->session = z_open(z_move(config));
    if (!z_check(g_zenoh_context->session)) {
        // error
        return;
    }

    g_zenoh_context->streams_enabled = enable;
    g_zenoh_context->streams_started = 0x00;
    g_zenoh_context->valid = true;


    //if (enable & HL2SS_ENABLE_RM) { RM_Initialize(); }
    //if (enable & HL2SS_ENABLE_PV) { PV_Initialize(); }
    //if (enable & HL2SS_ENABLE_MC) { MC_Initialize(); }
    //if (enable & HL2SS_ENABLE_SI) { SI_Initialize(); }
    //if (enable & HL2SS_ENABLE_RC) { RC_Initialize(); }
    //if (enable & HL2SS_ENABLE_SM) { SM_Initialize(); }
    //if (enable & HL2SS_ENABLE_SU) { SU_Initialize(); }
    //if (enable & HL2SS_ENABLE_VI) { VI_Initialize(); }
    //if (enable & HL2SS_ENABLE_MQ) { MQ_Initialize(); }
    //if (enable & HL2SS_ENABLE_EET) { EET_Initialize(); }


    StartManager(g_zenoh_context);

    MQ_Initialize(g_zenoh_context);

}

void TeardownStreams() {
    if (g_zenoh_context != nullptr) {
        MQ_Quit();
        MQ_Cleanup();
        StopManager(g_zenoh_context);
        g_zenoh_context.reset();
    }
}


// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
InitializeStreamsOnUI(const char* cid, const char* zcfg, uint32_t enable)
{
    call_deferred(InitializeStreams, cid, zcfg, enable);
}


// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
TeardownStreamsOnUI()
{
    call_deferred(TeardownStreams);
}


extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartRMOnUI(
    bool enable_location,
    bool enable_left_front, bool enable_left_left,
    bool enable_right_front, bool enable_right_right,
    H26xFormat vlc_format,
    bool enable_depth_ahat, bool enable_depth_long_throw,
    H26xFormat depth_format,
    bool enable_imu_accel, bool enable_imu_gyro, bool enable_imu_mag)
{
    if (!g_zenoh_context) {
        return false;
    }

    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_RM) { 
        call_deferred(
            StartRM, g_zenoh_context, enable_location,
            enable_left_front, enable_left_left,
            enable_right_front, enable_right_right,
            vlc_format,
            enable_depth_ahat, enable_depth_long_throw,
            depth_format,
            enable_imu_accel, enable_imu_gyro, enable_imu_mag);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopRMOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_RM) {
        call_deferred(StopRM, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartMCOnUI(AACFormat aac_format)
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_MC) {
        call_deferred(StartMC, g_zenoh_context, aac_format);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopMCOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_MC) {
        call_deferred(StopMC, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartPVOnUI(bool enable_location, H26xFormat pv_format)
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_PV) {
        call_deferred(StartPV, g_zenoh_context, enable_location, pv_format);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopPVOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_PV) {
        call_deferred(StopPV, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartSIOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SI) {
        call_deferred(StartSI, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopSIOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SI) {
        call_deferred(StopSI, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartRCOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_RC) {
        call_deferred(StartRC, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopRCOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_RC) {
        call_deferred(StopRC, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartSMOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SM) {
        call_deferred(StartSM, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopSMOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SM) {
        call_deferred(StopSM, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartSUOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SU) {
        call_deferred(StartSU, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopSUOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_SU) {
        call_deferred(StopSU, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartVIOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_VI) {
        call_deferred(StartVI, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopVIOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_VI) {
        call_deferred(StopVI, g_zenoh_context);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StartEETOnUI(uint8_t eye_fps)
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_EET) {
        call_deferred(StartEET, g_zenoh_context, eye_fps);
        return true;
    }
    return false;
}

extern "C" bool UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
StopEETOnUI()
{
    if (!g_zenoh_context) { return false; }
    if (g_zenoh_context->streams_enabled & HL2SS_ENABLE_EET) {
        call_deferred(StopEET, g_zenoh_context);
        return true;
    }
    return false;
}


// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
DebugMessage(char const* str)
{
    SPDLOG_INFO("{0}", str);
}

// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
GetLocalIPv4Address(wchar_t *buffer, int size)
{
    std::vector<wchar_t> address;
    GetLocalIPv4Address(address);
    wcscpy_s(buffer, size / sizeof(wchar_t), address.data());
}

// OK
extern "C" int UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
OverrideWorldCoordinateSystem(void* scs_ptr)
{
    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem scs = nullptr;
    if (scs_ptr)
    {
    winrt::copy_from_abi(scs, scs_ptr);
    scs = Locator_SanitizeSpatialCoordinateSystem(scs);
    if (!scs) { return false; }
    }
    Locator_OverrideWorldCoordinateSystem(scs);
    return true;
}
