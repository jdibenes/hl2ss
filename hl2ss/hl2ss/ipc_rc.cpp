
#include "server.h"
#include "ports.h"
#include "log.h"
#include "holographic_space.h"
#include "personal_video.h"
#include "timestamps.h"
#include "nfo.h"


#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#include "pcpd_msgs/rpc/Hololens2RemoteControl.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_event_client = NULL; // CloseHandle

static HC_Context_Ptr  g_zenoh_context{};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
//static void RC_TransferError()
//{
//    SetEvent(g_event_client);
//}

// OK

struct RC_GetApplicationVersion {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        uint16_t data[4];
        GetApplicationVersion(data);
        response.data({ data[0], data[1], data[2], data[3] });
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_GetUTCOffset {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::UInt64Reply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        UINT64 offset;
        offset = GetQPCToUTCOffset(request.value());
        response.value(offset);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetHSMarkerState {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        HolographicSpace_EnableMarker(request.value() != 0);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_GetPVSubsystemStatus {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::BoolReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        bool status = PersonalVideo_Status();
        response.value(status);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVFocus {
    using RequestT = pcpd_msgs::rpc::HL2RCRequest_SetPVFocus;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetFocus(
            request.focus_mode(),
            request.autofocus_range(),
            request.distance(),
            request.value(),
            request.disable_driver_fallback()
        );
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVVideoTemporalDenoising {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetVideoTemporalDenoising(request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVWhiteBalancePreset {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetWhiteBalance_Preset(request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVWhiteBalanceValue {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetWhiteBalance_Value(request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVExposure {
    using RequestT = pcpd_msgs::rpc::HL2RCRequest_SetPVExposure;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetExposure(request.mode(), request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVExposurePriorityVideo {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetExposurePriorityVideo(request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVIsoSpeed {
    using RequestT = pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetIsoSpeed(request.setauto(), request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct RC_SetPVBacklightCompensation {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetBacklightCompensation(request.value() != 0);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


// OK
struct RC_SetPVSceneMode {
    using RequestT = pcpd_msgs::rpc::UInt32Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        PersonalVideo_SetSceneMode(request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};




/*
* ArgumentHelper
*/

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2RCRequest_SetPVFocus> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVFocus& request) {
        try {
            request.focus_mode(args_extract<uint32_t, 3>(args, "focus_mode"));
            request.autofocus_range(args_extract<uint32_t, 2>(args, "autofocus_range"));
            request.distance(args_extract<uint32_t, 0>(args, "distance"));
            request.value(args_extract<uint32_t, 1000>(args, "value"));
            request.disable_driver_fallback(args_extract<uint32_t, 1>(args, "disable_driver_fallback"));
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_INFO("RC: invalid argument: {0}", e.what());
            return false;
        }
        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2RCRequest_SetPVExposure> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVExposure& request) {
        try {
            request.mode(args_extract<uint32_t, 0>(args, "mode"));
            request.value(args_extract<uint32_t, 10000>(args, "value"));
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_INFO("RC: invalid argument: {0}", e.what());
            return false;
        }
        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed& request) {
        try {
            request.setauto(args_extract<uint32_t, 1>(args, "setauto"));
            request.value(args_extract<uint32_t, 100>(args, "value"));
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_INFO("RC: invalid argument: {0}", e.what());
            return false;
        }

        return true;
    }
};




void RC_QueryHandler(const z_query_t* query, void* context) {
    z_owned_str_t keystr = z_keyexpr_to_string(z_query_keyexpr(query));
    z_bytes_t pred = z_query_parameters(query);
    z_value_t payload_value = z_query_value(query);
    
    std::map<std::string, std::string> arguments;

    // parse query parameters
    UriQueryListA* queryList = NULL;
    int itemCount = 0;
    const int res = uriDissectQueryMallocA(&queryList, &itemCount,
        reinterpret_cast<const char*>(pred.start), reinterpret_cast<const char*>(pred.start + pred.len));
    if (res == URI_SUCCESS) {
        UriQueryListA* current = queryList;
        while (current != nullptr) {
            if (current->key != nullptr && current->value != nullptr) {
                arguments.insert(std::pair(current->key, current->value));
                SPDLOG_INFO("Received argument: %s -> {0}", current->key, current->value);
            }
            current = current->next;
        }
    }
    uriFreeQueryListA(queryList);

    eprosima::fastcdr::FastBuffer result_buffer{};
    std::size_t result_bytes{0};
    bool call_success{ false };

    if (arguments.find("cmd") != arguments.end()) {
        auto cmd = arguments.extract("cmd");

        if (cmd.mapped() == "GetApplicationVersion") {
            call_success = forward_rpc_call(RC_GetApplicationVersion{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetUTCOffset") {
            call_success = forward_rpc_call(RC_GetUTCOffset{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetHSMarkerState") {
            call_success = forward_rpc_call(RC_SetHSMarkerState{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetPVSubsystemStatus") {
            call_success = forward_rpc_call(RC_GetPVSubsystemStatus{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVFocus") {
            call_success = forward_rpc_call(RC_SetPVFocus{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVVideoTemporalDenoising") {
            call_success = forward_rpc_call(RC_SetPVVideoTemporalDenoising{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVWhiteBalancePreset") {
            call_success = forward_rpc_call(RC_SetPVWhiteBalancePreset{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVWhiteBalanceValue") {
            call_success = forward_rpc_call(RC_SetPVWhiteBalanceValue{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVExposure") {
            call_success = forward_rpc_call(RC_SetPVExposure{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVExposurePriorityVideo") {
            call_success = forward_rpc_call(RC_SetPVExposurePriorityVideo{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVIsoSpeed") {
            call_success = forward_rpc_call(RC_SetPVIsoSpeed{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVBacklightCompensation") {
            call_success = forward_rpc_call(RC_SetPVBacklightCompensation{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVSceneMode") {
            call_success = forward_rpc_call(RC_SetPVSceneMode{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else {
            call_success = false;
        }
    }

    if (!call_success) {
        std::string pred_str((const char*)pred.start, pred.len);
        if (payload_value.payload.len > 0) {
            SPDLOG_WARN("[Queryable ] Received unhandled Query '{0}?{1}' with payload length '{2}'",
                z_loan(keystr), pred_str, (int)payload_value.payload.len);
        }
        else {
            SPDLOG_WARN(">> [Queryable ] Received unhandled Query '{0}?{1}'", z_loan(keystr), pred_str);
        }
    }

    z_query_reply_options_t options = z_query_reply_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
    z_query_reply(query, z_keyexpr((const char*)context), (const uint8_t*)result_buffer.getBuffer(), result_bytes, &options);
    z_drop(z_move(keystr));
}



// OK
static DWORD WINAPI RC_EntryPoint(void *param)
{

    if (!g_zenoh_context || !g_zenoh_context->valid) {
        SPDLOG_INFO("RC: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    std::string keyexpr_str = g_zenoh_context->topic_prefix + "/rpc/ctrl";
    SPDLOG_INFO("RC: endpoint: {0}", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        SPDLOG_INFO("RC: {0} is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = RC_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(z_loan(g_zenoh_context->session), keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        SPDLOG_INFO("RC: Unable to create queryable.");
        return 1;
    }

    do
    {
        // heartbeat occassionally ??   
    }
    while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT && !g_zenoh_context->should_exit);

    z_undeclare_queryable(z_move(qable));
    SPDLOG_INFO("RC: Closed");

    return 0;
}

// OK
void RC_Initialize(HC_Context_Ptr& context)
{
    g_zenoh_context = context;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, RC_EntryPoint, NULL, 0, NULL);
}

// OK
void RC_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void RC_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_client = NULL;
    g_event_quit = NULL;

    g_zenoh_context.reset();
}
