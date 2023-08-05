
#include "server.h"
#include "ports.h"
#include "voice_input.h"
#include "log.h"

#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#include "pcpd_msgs/rpc/Hololens2VoiceInput.h"


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

HANDLE g_thread = NULL;
HANDLE g_event_quit = NULL;
HANDLE g_event_client = NULL;

static HC_Context_Ptr g_zenoh_context;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------


// OK
struct VI_CreateRecognizer {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        VoiceInput_CreateRecognizer();
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct VI_RegisterCommands {
    using RequestT = pcpd_msgs::rpc::HL2VIRequest_RegisterCommands;
    using ResponseT = pcpd_msgs::rpc::UInt8Reply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {

        uint8_t clear = request.clear();
        std::vector<winrt::hstring> commands;
        uint8_t result;

        for (auto& v : request.commands())
        {
            // convert to wchar
            size_t size = v.size() + 1;
            wchar_t* cmd = new wchar_t[size]; // release

            size_t outSize;
            mbstowcs_s(&outSize, cmd, size, v.c_str(), size - 1);

            commands.push_back(winrt::hstring(cmd, static_cast<winrt::hstring::size_type>(size)));

            free(cmd);
        }

        result = VoiceInput_RegisterCommands(commands, clear != 0);
        response.value(result);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct VI_Start {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        VoiceInput_Start();
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct VI_Pop {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::HL2VIResponse_VoiceInput_Pop;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {

        uint32_t count;
        VoiceInput_Result result;

        count = (uint32_t)VoiceInput_GetCount();

        std::vector<pcpd_msgs::rpc::HL2VIVoiceInput_Result> virs(count);

        for (uint32_t i = 0; i < count; ++i)
        {
            auto& t = virs.at(i);
            result = VoiceInput_Pop();
            t.index(result.Index);
            t.confidence(result.Confidence);
            t.phrase_duration(result.PhraseDuration);
            t.phrase_start_time(result.PhraseStartTime);
            t.raw_confidence(result.RawConfidence);
        }
        response.results(std::move(virs));
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


// OK
struct VI_Clear {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        VoiceInput_Clear();
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct VI_Stop {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        VoiceInput_Stop();
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


std::vector < std::string > split_string(std::string str, char separator) {
    std::vector < std::string > strings;
    int startIndex = 0, endIndex = 0;
    for (int i = 0; i <= str.size(); i++) {

        // If we reached the end of the word or the end of the input.
        if (str[i] == separator || i == str.size()) {
            endIndex = i;
            std::string temp;
            temp.append(str, startIndex, endIndex - startIndex);
            strings.push_back(temp);
            startIndex = endIndex + 1;
        }
    }
    return strings;
}

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2VIRequest_RegisterCommands> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2VIRequest_RegisterCommands& request) {
        try {
            request.clear(args_extract<bool, false>(args, "clear"));

            if (args.find("commands") != args.end()) {
                request.commands(split_string(args.at("commands"), '|'));
            }
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_INFO("RC: invalid argument: {0}", e.what());
            return false;
        }

        return true;
    }
};


void VI_QueryHandler(const z_query_t* query, void* context) {
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
                SPDLOG_INFO("Received argument: {0} -> {1}", current->key, current->value);
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

        if (cmd.mapped() == "CreateRecognizer") {
            call_success = forward_rpc_call(VI_CreateRecognizer{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        if (cmd.mapped() == "RegisterCommands") {
            call_success = forward_rpc_call(VI_RegisterCommands{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "Start") {
            call_success = forward_rpc_call(VI_Start{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "Pop") {
            call_success = forward_rpc_call(VI_Pop{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "Clear") {
            call_success = forward_rpc_call(VI_Clear{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "Stop") {
            call_success = forward_rpc_call(VI_Stop{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else {
            call_success = false;
        }
        if (!call_success) {
            SPDLOG_ERROR("VI: Execution of command: {0} failed.", cmd.mapped());
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
static DWORD WINAPI VI_EntryPoint(void *param)
{

    if (!g_zenoh_context || !g_zenoh_context->valid) {
        SPDLOG_INFO("VI: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    std::string keyexpr_str = g_zenoh_context->topic_prefix + "/rpc/vi";
    SPDLOG_INFO("VI: endpoint: {0}", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        SPDLOG_INFO("VI: {0} is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = VI_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(z_loan(g_zenoh_context->session), keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        SPDLOG_INFO("VI: Unable to create queryable.");
        return 1;
    }

    do
    {
        // heartbeat occassionally ??   
    } while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT && !g_zenoh_context->should_exit);

    if (VoiceInput_IsRunning()) { 
        VoiceInput_Stop();
        VoiceInput_Clear();
    }

    z_undeclare_queryable(z_move(qable));
    SPDLOG_INFO("VI: Closed");

    return 0;
}

// OK
void VI_Initialize(HC_Context_Ptr& context)
{
    g_zenoh_context = context;

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, VI_EntryPoint, NULL, 0, NULL);
}

// OK
void VI_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void VI_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_zenoh_context.reset();
}
