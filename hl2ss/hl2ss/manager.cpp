
#include "manager.h"

#include "log.h"
#include "timestamps.h"
#include "nfo.h"
#include <sstream>
#include <spdlog/spdlog.h>
#include "spdlog/sinks/base_sink.h"
#include "spdlog/details/log_msg_buffer.h"
#include "spdlog/details/null_mutex.h"


#define URI_STATIC_BUILD
#include "uriparser/Uri.h"

#include "hl2ss_network.h"

#include "pcpd_msgs/rpc/Hololens2Manager.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_event_client = NULL; // CloseHandle

static HC_Context* g_zenoh_context = NULL;


//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
namespace spdlog {
    namespace sinks {


        template<typename Mutex>
        class zenoh_logging_sink final : public base_sink<Mutex>
        {

        public:
            zenoh_logging_sink() {
                if (g_zenoh_context == nullptr) {
                    // invalid context
                    return;
                }
                _client_id = g_zenoh_context->client_id.c_str();
                std::string keyexpr = "hl2/logs/" + g_zenoh_context->client_id;
                OutputDebugStringA(fmt::format("MGR: publish logs at: {0}\n", keyexpr).c_str());

                z_publisher_options_t publisher_options = z_publisher_options_default();
                publisher_options.priority = Z_PRIORITY_REAL_TIME;

                _publisher = z_declare_publisher(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr.c_str()), &publisher_options);

                if (!z_check(_publisher)) {
                    OutputDebugStringA("MGR: Error creating publisher\n");
                }
                _is_active = true;
            }

            virtual ~zenoh_logging_sink() {
                z_undeclare_publisher(z_move(_publisher));
            }

        protected:
            void sink_it_(const details::log_msg& msg) override
            {
                if (!_is_active) {
                    return;
                }
                memory_buf_t formatted;
                base_sink<Mutex>::formatter_->format(msg, formatted);
                std::string msg_str = fmt::to_string(formatted);
                _messages.push_back(std::move(msg_str));
            }
            void flush_() override {
                // nothing to do here
                z_publisher_put_options_t options = z_publisher_put_options_default();
                options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);

                pcpd_msgs::rpc::Hololens2LogMessage value{};
                {
                    auto timestamp = GetCurrentQPCTimestamp();
                    using namespace std::chrono;
                    auto ts_ = nanoseconds(timestamp * 100);
                    auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
                    auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

                    value.header().stamp().sec(ts_sec);
                    value.header().stamp().nanosec(ts_nsec);

                    value.header().frame_id(_client_id);
                }
                value.messages(_messages);

                eprosima::fastcdr::FastBuffer buffer{};
                eprosima::fastcdr::Cdr buffer_cdr(buffer);

                buffer_cdr.reset();
                value.serialize(buffer_cdr);

                // send message to zenoh
                if (z_publisher_put(z_loan(_publisher), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options)) {
                    OutputDebugStringA("MGR: Error publishing log messages.\n");
                }
                else {
                    _messages.clear();
                }
            }

        private:
            std::vector<std::string> _messages;
            z_owned_publisher_t _publisher;
            const char* _client_id;
            bool _is_active{ false };
        };

        using zenoh_logging_sink_mt = zenoh_logging_sink<std::mutex>;
        using zenoh_logging_sink_st = zenoh_logging_sink<details::null_mutex>;





    } // namespace sinks
}



pcpd_msgs::msg::Hololens2H26xProfile extract_h26x_profile(const std::map<std::string, std::string>& args, const std::string key, 
    pcpd_msgs::msg::Hololens2H26xProfile default = pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Main) {
    if (args.find(key) == args.end()) {
        return default;
    }
    auto& fmt = args.at(key);
    if (fmt == "None") {
       return pcpd_msgs::msg::H26xProfile_None;
    }
    else if (fmt == "H264Base") {
        return pcpd_msgs::msg::H264Profile_Base;
    }
    else if (fmt == "H264Base") {
        return pcpd_msgs::msg::H264Profile_Main;
    }
    else if (fmt == "H264High") {
        return pcpd_msgs::msg::H264Profile_High;
    }
    else if (fmt == "H265Main") {
        return pcpd_msgs::msg::H265Profile_Main;
    }
    ShowMessage("Invalid argument for h26x_profile: %s", fmt.c_str());
    return default;
}

pcpd_msgs::msg::Hololens2AACProfile extract_aac_profile(const std::map<std::string, std::string>& args, const std::string key,
    pcpd_msgs::msg::Hololens2AACProfile default = pcpd_msgs::msg::Hololens2AACProfile::AACProfile_24000) {
    if (args.find(key) == args.end()) {
        return default;
    }
    auto& fmt = args.at(key);
    if (fmt == "None") {
        return pcpd_msgs::msg::AACProfile_None;
    }
    else if (fmt == "AAC12") {
        return pcpd_msgs::msg::AACProfile_12000;
    }
    else if (fmt == "AAC16") {
        return pcpd_msgs::msg::AACProfile_16000;
    }
    else if (fmt == "AAC20") {
        return pcpd_msgs::msg::AACProfile_20000;
    }
    else if (fmt == "AAC24") {
        return pcpd_msgs::msg::AACProfile_24000;
    }
    ShowMessage("Invalid aac_profile: %s", fmt.c_str());
    return default;
}


// OK
struct MGR_StartRM {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartRM;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {

        H26xFormat vlc_fmt;
        vlc_fmt.width = request.vlc_format().width();
        vlc_fmt.height = request.vlc_format().height();
        switch (request.vlc_format().profile()) {
        case pcpd_msgs::msg::Hololens2H26xProfile::H26xProfile_None:
            vlc_fmt.profile = H26xProfile_None;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Base:
            vlc_fmt.profile = H264Profile_Base;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Main:
            vlc_fmt.profile = H264Profile_Main;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_High:
            vlc_fmt.profile = H264Profile_High;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H265Profile_Main:
            vlc_fmt.profile = H265Profile_Main;
            break;
        }
        vlc_fmt.bitrate = request.vlc_format().bitrate();

        H26xFormat depth_fmt;
        depth_fmt.width = request.depth_format().width();
        depth_fmt.height = request.depth_format().height();
        switch (request.depth_format().profile()) {
        case pcpd_msgs::msg::Hololens2H26xProfile::H26xProfile_None:
            depth_fmt.profile = H26xProfile_None;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Base:
            depth_fmt.profile = H264Profile_Base;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Main:
            depth_fmt.profile = H264Profile_Main;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_High:
            depth_fmt.profile = H264Profile_High;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H265Profile_Main:
            depth_fmt.profile = H265Profile_Main;
            break;
        }
        depth_fmt.bitrate = request.depth_format().bitrate();

        StartRM(g_zenoh_context, 
            request.enable_location(), 
            request.enable_left_front(), request.enable_left_left(), 
            request.enable_right_front(), request.enable_right_right(), 
            vlc_fmt, 
            request.enable_depth_ahat(), request.enable_depth_long_throw(), 
            depth_fmt, 
            request.enable_imu_accel(), request.enable_imu_gyro(), request.enable_imu_mag());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopRM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopRM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartMC {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartMC;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        AACFormat fmt;
        fmt.channels = request.aac_format().channels();
        fmt.samplerate = request.aac_format().sample_rate();
        switch (request.aac_format().profile()) {
        case pcpd_msgs::msg::Hololens2AACProfile::AACProfile_None:
            fmt.profile = AACProfile_None;
            break;
        case pcpd_msgs::msg::Hololens2AACProfile::AACProfile_12000:
            fmt.profile = AACProfile_12000;
            break;
        case pcpd_msgs::msg::Hololens2AACProfile::AACProfile_16000:
            fmt.profile = AACProfile_16000;
            break;
        case pcpd_msgs::msg::Hololens2AACProfile::AACProfile_20000:
            fmt.profile = AACProfile_20000;
            break;
        case pcpd_msgs::msg::Hololens2AACProfile::AACProfile_24000:
            fmt.profile = AACProfile_24000;
            break;
        }
        StartMC(g_zenoh_context, fmt);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopMC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopMC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartPV {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartPV;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        H26xFormat pv_fmt;
        pv_fmt.width = request.pv_format().width();
        pv_fmt.height = request.pv_format().height();
        switch (request.pv_format().profile()) {
        case pcpd_msgs::msg::Hololens2H26xProfile::H26xProfile_None:
            pv_fmt.profile = H26xProfile_None;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Base:
            pv_fmt.profile = H264Profile_Base;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_Main:
            pv_fmt.profile = H264Profile_Main;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H264Profile_High:
            pv_fmt.profile = H264Profile_High;
            break;
        case pcpd_msgs::msg::Hololens2H26xProfile::H265Profile_Main:
            pv_fmt.profile = H265Profile_Main;
            break;
        }
        pv_fmt.bitrate = request.pv_format().bitrate();
        StartPV(g_zenoh_context, request.enable_location(), pv_fmt);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopPV {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopPV(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};


// OK
struct MGR_StartSI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StartSI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopSI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopSI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartRC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StartRC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopRC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopRC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartSM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StartSM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopSM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopSM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartSU {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StartSU(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopSU {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopSU(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartVI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StartVI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopVI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopVI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StartEET {
    using RequestT = pcpd_msgs::rpc::UInt8Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        StartEET(g_zenoh_context, request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

// OK
struct MGR_StopEET {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        StopEET(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        return true;
    }
};

/*
* ArgumentHelper
*/

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2MGRRequest_StartRM> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2MGRRequest_StartRM& request) {

        try {
            request.enable_location(args_extract<bool, true>(args, "location"));

            request.enable_left_front(args_extract<bool, false>(args, "vlc_lf"));
            request.enable_left_left(args_extract<bool, false>(args, "vlc_ll"));
            request.enable_right_front(args_extract<bool, false>(args, "vlc_rf"));
            request.enable_right_right(args_extract<bool, false>(args, "vlc_rr"));

            request.enable_depth_ahat(args_extract<bool, false>(args, "depth_ahat"));
            request.enable_depth_long_throw(args_extract<bool, true>(args, "depth_lt"));

            request.enable_imu_accel(args_extract<bool, false>(args, "imu_accel"));
            request.enable_imu_gyro(args_extract<bool, false>(args, "imu_gyro"));
            request.enable_imu_mag(args_extract<bool, false>(args, "imu_mag"));

            auto& vf = request.vlc_format();
            vf.width(args_extract<uint16_t, 640>(args, "vlc_width"));
            vf.height(args_extract<uint16_t, 480>(args, "vlc_height"));
            vf.frame_rate(args_extract<uint8_t, 30>(args, "vlc_fps"));
            vf.profile(extract_h26x_profile(args, "vlc_profile"));
            vf.bitrate(args_extract<uint32_t, static_cast<int>((640 * 480 * 30 * 12.) * (4. / 420.))>(args, "vlc_bitrate"));

            auto& df = request.depth_format();
            df.width(args_extract<uint16_t, 512>(args, "ahat_width"));
            df.height(args_extract<uint16_t, 512>(args, "ahat_height"));
            df.frame_rate(args_extract<uint8_t, 45>(args, "ahat_fps"));
            df.profile(extract_h26x_profile(args, "ahat_profile"));
            df.bitrate(args_extract<uint32_t, 20000 * 8>(args, "ahat_bitrate")); // unsure about sensible default for bitrate here

        }
        catch (const std::invalid_argument& e) {
            ShowMessage("MGR: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2MGRRequest_StartPV> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2MGRRequest_StartPV& request) {
        try {
            request.enable_location(args_extract<bool, true>(args, "location"));
            request.pv_format().width(args_extract<uint16_t, 640>(args, "width"));
            request.pv_format().height(args_extract<uint16_t, 360>(args, "height"));
            request.pv_format().frame_rate(args_extract<uint8_t, 15>(args, "fps"));
            request.pv_format().profile(extract_h26x_profile(args, "profile", pcpd_msgs::msg::H264Profile_High));
            request.pv_format().bitrate(args_extract<uint32_t, 40000 * 8>(args, "bitrate"));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("MGR: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::HL2MGRRequest_StartMC> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::HL2MGRRequest_StartMC& request) {

        try {
            request.aac_format().channels(args_extract<uint32_t, 2>(args, "channels"));
            request.aac_format().sample_rate(args_extract<uint32_t, 48000>(args, "sample_rate"));
            request.aac_format().profile(extract_aac_profile(args, "profile"));
        }
        catch (const std::invalid_argument& e) {
            ShowMessage("MGR: invalid argument: %s", e.what());
            return false;
        }

        return true;
    }
};




void MGR_QueryHandler(const z_query_t* query, void* context) {
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

        if (cmd.mapped() == "StartRM") {
            call_success = forward_rpc_call(MGR_StartRM{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopRM") {
            call_success = forward_rpc_call(MGR_StopRM{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartMC") {
            call_success = forward_rpc_call(MGR_StartMC{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopMC") {
            call_success = forward_rpc_call(MGR_StopMC{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartPV") {
            call_success = forward_rpc_call(MGR_StartPV{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopPV") {
            call_success = forward_rpc_call(MGR_StopPV{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartSI") {
            call_success = forward_rpc_call(MGR_StartSI{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopSI") {
            call_success = forward_rpc_call(MGR_StopSI{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartRC") {
            call_success = forward_rpc_call(MGR_StartRC{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopRC") {
            call_success = forward_rpc_call(MGR_StopRC{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartSM") {
            call_success = forward_rpc_call(MGR_StartSM{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopSM") {
            call_success = forward_rpc_call(MGR_StopSM{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartSU") {
            call_success = forward_rpc_call(MGR_StartSU{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopSU") {
            call_success = forward_rpc_call(MGR_StopSU{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartVI") {
            call_success = forward_rpc_call(MGR_StartVI{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopVI") {
            call_success = forward_rpc_call(MGR_StopVI{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StartEET") {
            call_success = forward_rpc_call(MGR_StartEET{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
        }
        else if (cmd.mapped() == "StopEET") {
            call_success = forward_rpc_call(MGR_StopEET{}, nullptr, payload_value, arguments, result_buffer, result_bytes);
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
static DWORD WINAPI MGR_EntryPoint(void* param)
{
    ShowMessage("MGR: EntryPoint Main Thread");

    if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
        ShowMessage("MGR: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    // register logger
    auto cbs = std::make_shared< spdlog::sinks::zenoh_logging_sink_mt>();
    std::vector<spdlog::sink_ptr>& sinks = spdlog::details::registry::instance().get_default_raw()->sinks();
    sinks.push_back(cbs);

    std::string keyexpr_str = "hl2/rpc/mgr/" + g_zenoh_context->client_id;
    ShowMessage("MGR: endpoint: %s", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        ShowMessage("MGR: %s is not a valid key expression", keyexpr_str.c_str());
        return 1;
    }
    const char* expr = keyexpr_str.c_str();

    // zclosure macro does not work with c++17
    z_owned_closure_query_t callback{};
    callback.call = MGR_QueryHandler;
    callback.context = const_cast<void*>(static_cast<const void*>(expr));
    callback.drop = nullptr;

    z_owned_queryable_t qable = z_declare_queryable(z_loan(g_zenoh_context->session), keyexpr, z_move(callback), NULL);
    if (!z_check(qable)) {
        ShowMessage("MGR: Unable to create queryable.");
        return 1;
    }

    uint64_t heart_beat_counter{ 0 };

    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer);
    std::string keyexpr1 = "hl2/presence/" + g_zenoh_context->client_id;

    {
        pcpd_msgs::rpc::Hololens2Presence value{};

        {
            auto timestamp = GetCurrentQPCTimestamp();
            using namespace std::chrono;
            auto ts_ = nanoseconds(timestamp * 100);
            auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
            auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

            value.header().stamp().sec(ts_sec);
            value.header().stamp().nanosec(ts_nsec);

            value.header().frame_id(g_zenoh_context->client_id);
        }
        value.heart_beat_counter(heart_beat_counter);

        buffer_cdr.reset();
        value.serialize(buffer_cdr);

        // put message to zenoh
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            ShowMessage("MGR: Error putting presence");
        }
        else {
            ShowMessage("MGR: put presence");
        }
    }


    // heartbeat every 5 seconds
    uint32_t wait_counter{ 0 };
    do
    {
        Sleep(1000 / 100);
        if (wait_counter % 50 == 0) {
            pcpd_msgs::rpc::Hololens2Presence value{};

            {
                auto timestamp = GetCurrentQPCTimestamp();
                using namespace std::chrono;
                auto ts_ = nanoseconds(timestamp * 100);
                auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
                auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

                value.header().stamp().sec(ts_sec);
                value.header().stamp().nanosec(ts_nsec);

                value.header().frame_id(g_zenoh_context->client_id);
            }
            value.heart_beat_counter(heart_beat_counter);

            buffer_cdr.reset();
            value.serialize(buffer_cdr);

            // put message to zenoh
            z_put_options_t options1 = z_put_options_default();
            options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
            int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
            if (res > 0) {
                ShowMessage("MGR: Error putting presence");
            }

        }
    } while (WaitForSingleObject(g_event_quit, 100) == WAIT_TIMEOUT);

    z_delete_options_t options = z_delete_options_default();
    int res = z_delete(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), &options);
    if (res < 0) {
        ShowMessage("MGR: delete presence failed");
    }

    z_undeclare_queryable(z_move(qable));
    ShowMessage("MGR: Closed");

    return 0;
}



void MGR_Initialize() {

    assert(g_zenoh_context != nullptr);
    assert(g_zenoh_context->valid);

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, MGR_EntryPoint, NULL, 0, NULL);
}

void MGR_Quit() {
    SetEvent(g_event_quit);
}

void MGR_Cleanup() {
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_client = NULL;
    g_event_quit = NULL;

    // who is responsible for cleaning up .. the app or the manager??
    //free(g_zenoh_context);
    //g_zenoh_context = NULL;
}




bool StartManager(HC_Context* context) {
    ShowMessage("StartManager...");
    if (context == nullptr) {
        return false;
    }
    if (g_zenoh_context != nullptr) {
        // tried to start with another context .. ignore
        return false;
    }
    g_zenoh_context = context;

    MGR_Initialize();

    return true;
}

bool StopManager(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }

    MGR_Quit();
    MGR_Cleanup();

    return true;
}




bool StartRM(HC_Context* context,
    bool enable_location,
    bool enable_left_front, bool enable_left_left,
    bool enable_right_front, bool enable_right_right,
    H26xFormat vlc_format,
    bool enable_depth_ahat, bool enable_depth_long_throw,
    H26xFormat depth_format,
    bool enable_imu_accel, bool enable_imu_gyro, bool enable_imu_mag) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RM)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_RM) { return false; }
    context->streams_started |= HL2SS_ENABLE_RM;

    RMStreamConfig rm_config{};
    rm_config.enable_location = enable_location;
    rm_config.enable_left_front = enable_left_front;
    rm_config.enable_left_left = enable_left_left;
    rm_config.enable_right_front = enable_right_front;
    rm_config.enable_right_right = enable_right_right;
    rm_config.vlc_format = vlc_format;
    rm_config.enable_depth_ahat = enable_depth_ahat;
    rm_config.enable_depth_long_throw = enable_depth_long_throw;
    rm_config.depth_format = depth_format;
    rm_config.enable_imu_accel = enable_imu_accel;
    rm_config.enable_imu_gyro = enable_imu_gyro;
    rm_config.enable_imu_mag = enable_imu_mag;

    RM_Initialize(context->client_id.c_str(), z_loan(context->session), rm_config);

    return true;
}

bool StopRM(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RM)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_RM)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_RM;
    RM_Quit();
    RM_Cleanup();
    return true;
}

bool StartPV(HC_Context* context,
    bool enable_location,
    H26xFormat pv_format) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_PV)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_PV) { return false; }
    context->streams_started |= HL2SS_ENABLE_PV;
    PV_Initialize(context->client_id.c_str(), z_loan(context->session), enable_location, pv_format);
    return true;
}

bool StopPV(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_PV)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_PV)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_PV;
    PV_Quit();
    PV_Cleanup();
    return true;
}

bool StartMC(HC_Context* context, AACFormat aac_format) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_MC)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_MC) { return false; }
    context->streams_started |= HL2SS_ENABLE_MC;
    MC_Initialize(context->client_id.c_str(), z_loan(context->session), aac_format);
    return true;
}

bool StopMC(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_MC)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_MC)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_MC;
    MC_Quit();
    MC_Cleanup();
    return true;
}

bool StartSI(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SI)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_SI) { return false; }
    context->streams_started |= HL2SS_ENABLE_SI;
    SI_Initialize(context->client_id.c_str(), z_loan(context->session));
    return true;
}

bool StopSI(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SI)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_SI)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_SI;
    SI_Quit();
    SI_Cleanup();
    return true;
}

bool StartRC(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RC)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_RC) { return false; }
    context->streams_started |= HL2SS_ENABLE_RC;
    RC_Initialize(context->client_id.c_str(), z_loan(context->session));
    return true;
}

bool StopRC(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RC)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_RC)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_RC;
    RC_Quit();
    RC_Cleanup();
    return true;
}

bool StartSM(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SM)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_SM) { return false; }
    context->streams_started |= HL2SS_ENABLE_SM;
    SM_Initialize(context->client_id.c_str(), z_loan(context->session));
    return true;
}

bool StopSM(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SM)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_SM)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_SM;
    SM_Quit();
    SM_Cleanup();
    return true;
}

bool StartSU(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SU)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_SU) { return false; }
    context->streams_started |= HL2SS_ENABLE_SU;
    SU_Initialize(context->client_id.c_str(), z_loan(context->session));
    return true;
}

bool StopSU(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SU)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_SU)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_SU;
    SU_Quit();
    SU_Cleanup();
    return true;
}

bool StartVI(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_VI)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_VI) { return false; }
    context->streams_started |= HL2SS_ENABLE_VI;
    VI_Initialize(context->client_id.c_str(), z_loan(context->session));
    return true;
}

bool StopVI(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_VI)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_VI)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_VI;
    VI_Quit();
    VI_Cleanup();
    return true;
}

bool StartEET(HC_Context* context, uint8_t eye_fps) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_EET)) { return false; }
    if (context->streams_started & HL2SS_ENABLE_EET) { return false; }
    context->streams_started |= HL2SS_ENABLE_EET;
    EET_Initialize(context->client_id.c_str(), z_loan(context->session), eye_fps);
    return true;
}

bool StopEET(HC_Context* context) {
    if (context == nullptr) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_EET)) { return false; }
    if (!(context->streams_started & HL2SS_ENABLE_EET)) { return false; }
    context->streams_started &= ~HL2SS_ENABLE_EET;
    EET_Quit();
    EET_Cleanup();
    return true;
}