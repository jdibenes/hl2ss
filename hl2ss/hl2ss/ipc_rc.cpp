
#include "server.h"
#include "ports.h"
#include "log.h"
#include "holographic_space.h"
#include "personal_video.h"
#include "timestamps.h"
#include "nfo.h"
#include <functional>

#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/rpc/Types.h"
#include "pcpd_msgs/rpc/Hololens2RemoteControl.h"


struct RC_Context {
    std::string client_id;
    z_session_t session;
    bool valid{ false };
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_event_client = NULL; // CloseHandle

static RC_Context* g_zenoh_context = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
//static void RC_TransferError()
//{
//    SetEvent(g_event_client);
//}

// OK
static void RC_MSG_GetApplicationVersion(
    const pcpd_msgs::rpc::NullRequest& /*request*/,
    pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion& response)
{
    uint16_t data[4];
    GetApplicationVersion(data);
    response.data({ data [0], data[1], data[2], data[3] });
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_GetUTCOffset(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::UInt64Reply& response)
{
    UINT64 offset;
    offset = GetQPCToUTCOffset(request.value());
    response.value(offset);
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetHSMarkerState(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    HolographicSpace_EnableMarker(request.value() != 0);
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_GetPVSubsystemStatus(
    const pcpd_msgs::rpc::NullRequest& /*request*/,
    pcpd_msgs::rpc::BoolReply& response)
{
    bool status = PersonalVideo_Status();
    response.value(status);
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVFocus(
    const pcpd_msgs::rpc::HL2RCRequest_SetPVFocus& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetFocus(
        request.focus_mode(), 
        request.autofocus_range(), 
        request.distance(), 
        request.value(), 
        request.disable_driver_fallback()
    );
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVVideoTemporalDenoising(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetVideoTemporalDenoising(request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVWhiteBalancePreset(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetWhiteBalance_Preset(request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVWhiteBalanceValue(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetWhiteBalance_Value(request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVExposure(
    const pcpd_msgs::rpc::HL2RCRequest_SetPVExposure& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetExposure(request.mode(), request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVExposurePriorityVideo(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetExposurePriorityVideo(request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVIsoSpeed(
    const pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetIsoSpeed(request.setauto(), request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVBacklightCompensation(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetBacklightCompensation(request.value() != 0);
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}

// OK
static void RC_MSG_SetPVSceneMode(
    const pcpd_msgs::rpc::UInt32Request& request,
    pcpd_msgs::rpc::NullReply& response)
{
    PersonalVideo_SetSceneMode(request.value());
    response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
}


/*
* ArgumentHelper
*/

template<typename RequestType>
struct ArgumentHelper {
    bool parse(const std::map<std::string, std::string>& args, RequestType& value) {
        ShowMessage("RC: ArgumentHelper used for unregistered type..");
        return false;
    }
};

template<>
struct ArgumentHelper<pcpd_msgs::rpc::NullRequest> {
    bool parse(const std::map<std::string, std::string>& /*args*/, pcpd_msgs::rpc::NullRequest& /*request*/) {
        return true;
    }
};

template<>
struct ArgumentHelper<pcpd_msgs::rpc::UInt32Request> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::UInt32Request& request) {
        if (args.find("value") == args.end()) {
            ShowMessage("RC: missing argument: value");
            return false;
        }
        try {
            auto value = std::stoi(args.at("value"));
            // bounds check ??
            request.value(static_cast<uint32_t>(value));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("RC: invalid argument: %s", e.what());
            return false;
        }
        return true;
    }
};

template<>
struct ArgumentHelper<pcpd_msgs::rpc::HL2RCRequest_SetPVFocus> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVFocus& request) {

        if (args.find("focus_mode") == args.end() || 
            args.find("autofocus_range") == args.end() ||
            args.find("distance") == args.end() ||
            args.find("value") == args.end() ||
            args.find("disable_driver_fallback") == args.end()) {
            ShowMessage("RC: missing argument: focus_mode|autofocus_range|distance|value,disable_driver_fallback");
            return false;
        }
        try {
            request.focus_mode(static_cast<uint32_t>(std::stoi(args.at("focus_mode"))));
            request.autofocus_range(static_cast<uint32_t>(std::stoi(args.at("autofocus_range"))));
            request.distance(static_cast<uint32_t>(std::stoi(args.at("distance"))));
            request.value(static_cast<uint32_t>(std::stoi(args.at("value"))));
            request.disable_driver_fallback(static_cast<uint32_t>(std::stoi(args.at("disable_driver_fallback"))));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("RC: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};

template<>
struct ArgumentHelper<pcpd_msgs::rpc::HL2RCRequest_SetPVExposure> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVExposure& request) {

        if (args.find("mode") == args.end() ||
            args.find("value") == args.end()) {
            ShowMessage("RC: missing argument: mode|value");
            return false;
        }
        try {
            request.mode(static_cast<uint32_t>(std::stoi(args.at("mode"))));
            request.value(static_cast<uint32_t>(std::stoi(args.at("value"))));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("RC: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};

template<>
struct ArgumentHelper<pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed& request) {

        if (args.find("setauto") == args.end() ||
            args.find("value") == args.end()) {
            ShowMessage("RC: missing argument: setauto|value");
            return false;
        }
        try {
            request.setauto(static_cast<uint32_t>(std::stoi(args.at("setauto"))));
            request.value(static_cast<uint32_t>(std::stoi(args.at("value"))));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("RC: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};




/*
* Call Helper
*/

template<typename RequestType, typename ResponseType>
struct CallHelper {

    bool call(std::function<void(const RequestType&, ResponseType&)> func, z_value_t& request_payload, const std::map<std::string, std::string>& args,
        eprosima::fastcdr::FastBuffer& result_buffer, std::size_t& result_bytes) {

        RequestType request{};
        ResponseType response{};

        // parse arguments for request type
        bool args_valid{ false };
        if (request_payload.payload.len > 0) {

            // deserialize payload
            eprosima::fastcdr::FastBuffer request_buffer(
                const_cast<char*>(reinterpret_cast<const char*>(request_payload.payload.start)), 
                request_payload.payload.len);

            eprosima::fastcdr::Cdr request_buffer_cdr(request_buffer);
            try {
                request.deserialize(request_buffer_cdr);
                args_valid = true;
            }
            catch (const eprosima::fastcdr::exception::Exception& e) {
                ShowMessage("RC: error deserializing request payload: %s", e.what());
                args_valid = false;
            }
        }
        else {
            ArgumentHelper<RequestType> ah{};
            args_valid = ah.parse(args, request);
        }

        if (args_valid) {
            func(request, response);
        }
        else {
            response.status(pcpd_msgs::rpc::RPC_STATUS_ERROR);
        }

        eprosima::fastcdr::Cdr buffer_cdr(result_buffer);
        bool success = response.status() == pcpd_msgs::rpc::RPC_STATUS_SUCCESS;
        response.serialize(buffer_cdr);
        result_bytes = buffer_cdr.getSerializedDataLength();

        return success;
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
                ShowMessage("Received argument: %s -> %s", current->key, current->value);
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
            CallHelper<
                pcpd_msgs::rpc::NullRequest, 
                pcpd_msgs::rpc::HL2RCResponse_GetApplicationVersion> ch{};
            call_success = ch.call(RC_MSG_GetApplicationVersion, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetUTCOffset") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::UInt64Reply> ch{};
            call_success = ch.call(RC_MSG_GetUTCOffset, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetHSMarkerState") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetHSMarkerState, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "GetPVSubsystemStatus") {
            CallHelper<
                pcpd_msgs::rpc::NullRequest,
                pcpd_msgs::rpc::BoolReply> ch{};
            call_success = ch.call(RC_MSG_GetPVSubsystemStatus, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVFocus") {
            CallHelper<
                pcpd_msgs::rpc::HL2RCRequest_SetPVFocus,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVFocus, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVVideoTemporalDenoising") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVVideoTemporalDenoising, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVWhiteBalancePreset") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVWhiteBalancePreset, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVWhiteBalanceValue") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVWhiteBalanceValue, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVExposure") {
            CallHelper<
                pcpd_msgs::rpc::HL2RCRequest_SetPVExposure,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVExposure, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVExposurePriorityVideo") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVExposurePriorityVideo, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVIsoSpeed") {
            CallHelper<
                pcpd_msgs::rpc::HL2RCRequest_SetPVIsoSpeed,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVIsoSpeed, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVBacklightCompensation") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVBacklightCompensation, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "SetPVSceneMode") {
            CallHelper<
                pcpd_msgs::rpc::UInt32Request,
                pcpd_msgs::rpc::NullReply> ch{};
            call_success = ch.call(RC_MSG_SetPVSceneMode, payload_value, arguments, result_buffer, result_bytes);
        }
        else {
            call_success = false;
        }
    }

    if (!call_success) {
        if (payload_value.payload.len > 0) {
            ShowMessage(">> [Queryable ] Received unhandled Query '%s?%.*s' with value '%.*s'\n", z_loan(keystr), (int)pred.len,
                pred.start, (int)payload_value.payload.len, payload_value.payload.start);
        }
        else {
            ShowMessage(">> [Queryable ] Received unhandled Query '%s?%.*s'\n", z_loan(keystr), (int)pred.len, pred.start);
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

    if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
        ShowMessage("RC: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    std::string keyexpr_str = "hl2/rpc/rc/" + g_zenoh_context->client_id;
    ShowMessage("RC: endpoint: %s", keyexpr_str.c_str());

    // does it need to be global?
    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        ShowMessage("RC: %s is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = RC_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(g_zenoh_context->session, keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        ShowMessage("RC: Unable to create queryable.");
        return 1;
    }

    do
    {
        // heartbeat occassionally ??   
    }
    while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT);

    z_undeclare_queryable(z_move(qable));
    ShowMessage("RC: Closed");

    return 0;
}

// OK
void RC_Initialize(const char* client_id, z_session_t session)
{
    g_zenoh_context = new RC_Context(); // release
    g_zenoh_context->client_id = std::string(client_id);
    g_zenoh_context->session = session;
    g_zenoh_context->valid = true;

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

    free(g_zenoh_context);
    g_zenoh_context = NULL;
}
