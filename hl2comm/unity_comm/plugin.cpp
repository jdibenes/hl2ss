
#include <mfapi.h>
#include <thread>
#include <future>
#include <tuple>
#include <utility>
#include "configuration.h"
#include "ipc.h"
#include "plugin.h"

#ifdef UNITY_WIN
#include <Windows.h>
#endif

#include "../hl2comm/log.h"

#include <chrono>

#include "../hl2comm/hl2ss_network.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"





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
    std::future<bool> result = std::async(std::launch::async, [&capture, &event_done]() -> bool {
        capture();
        SetEvent(event_done);
        return true;
        });
    
    WaitForSingleObject(event_done, INFINITE);
    CloseHandle(event_done);
    return result.get();
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



// OK
void InitializeStreams(const char* _topic_prefix, const char* zcfg, uint32_t enable=0x00000000)
{
    if (g_zenoh_context != nullptr) {
        // plugin is already initialized
        // we could check the diff between the enabled streams and adapt for it ??
        return;
    }

    if (enable) {
        SPDLOG_WARN("The currently loaded version of the plugin does not support HL2 streams.");
    }

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
     * - category: [hl2|svc|srg|...] The category of the entry
     * - entity_id: the entity id, unique to the namespace/site parent
     *   (to avoid issues it is better to have globally unique ids.)
     * - comm_type: [str|cfg|rpc] either streaming, configuration or remote-procedure calls
     *
     */
    std::string default_topic_prefix{"tcn/loc/dev/"};

    std::string topic_prefix(_topic_prefix);
    if (topic_prefix.empty()) {
        std::string host_name{"device00"};
#ifdef UNITY_WIN
        char buffer[256];
        if (gethostname(buffer, sizeof(buffer) == 0)) {
            host_name = std::string(buffer);
        }
        else {
            SPDLOG_ERROR("Cannot get Hostname - use device00 as default.");
        }
#endif
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

}

// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
InitializeStreamsOnUI(const char* cid, const char* zcfg, uint32_t enable)
{
    call_deferred(InitializeStreams, cid, zcfg, enable);
}


// OK
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
DebugMessage(char const* str)
{
    SPDLOG_INFO("{0}", str);
}
