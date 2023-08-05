
#include "manager.h"

#include "log.h"
#include "timestamps.h"
#include "nfo.h"
#include <sstream>
#include <spdlog/spdlog.h>
#include "spdlog/sinks/base_sink.h"
#include "spdlog/details/log_msg_buffer.h"
#include "spdlog/details/null_mutex.h"
#include "spdlog/fmt/ranges.h"

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

static HC_Context_Ptr g_zenoh_context{};


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
                if (!g_zenoh_context) {
                    // invalid context
                    return;
                }
                _topic_prefix = g_zenoh_context->topic_prefix.c_str();
                std::string keyexpr = g_zenoh_context->topic_prefix + "/str/logs";
                OutputDebugStringA(fmt::format("MGR: publish logs at: {0}\\n", keyexpr).c_str());

                z_publisher_options_t publisher_options = z_publisher_options_default();
                publisher_options.priority = Z_PRIORITY_DATA;

                _publisher = z_declare_publisher(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr.c_str()), &publisher_options);

                if (!z_check(_publisher)) {
                    OutputDebugStringA("MGR: Error creating publisher\n");
                    return;
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
                _items.emplace_back(
                    std::chrono::time_point_cast<std::chrono::nanoseconds>(msg.time).time_since_epoch(),
                    msg.level,
                    msg_str
                );
            }
            void flush_() override {
                if (_items.empty()) {
                    return;
                }
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

                    value.header().frame_id(_topic_prefix);
                }
                std::vector< pcpd_msgs::rpc::Hololens2LogItem> items(_items.size());
                for (int i = 0; i < _items.size();++i) {
                    auto& [ts_, ll, msg] = _items.at(i);
                    auto& v = items.at(i);
                    {
                        using namespace std::chrono;
                        auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
                        auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

                        v.timestamp().sec(ts_sec);
                        v.timestamp().nanosec(ts_nsec);
                    }
                    switch (ll) {
                    case spdlog::level::level_enum::critical:
                    case spdlog::level::level_enum::err:
                        v.severity(pcpd_msgs::rpc::HL2_LOG_ERROR);
                        break;
                    case spdlog::level::level_enum::warn:
                        v.severity(pcpd_msgs::rpc::HL2_LOG_WARNING);
                        break;
                    case spdlog::level::level_enum::info:
                        v.severity(pcpd_msgs::rpc::HL2_LOG_INFO);
                        break;
                    case spdlog::level::level_enum::debug:
                        v.severity(pcpd_msgs::rpc::HL2_LOG_DEBUG);
                        break;
                    case spdlog::level::level_enum::trace:
                        v.severity(pcpd_msgs::rpc::HL2_LOG_TRACE);
                        break;
                    }
                    v.message(msg);
                }
                value.items(items);

                eprosima::fastcdr::FastBuffer buffer{};
                eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
                buffer_cdr.reset();
                buffer_cdr.serialize_encapsulation();

                value.serialize(buffer_cdr);

                // send message to zenoh
                if (z_publisher_put(z_loan(_publisher), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options)) {
                    OutputDebugStringA("MGR: Error publishing log messages.\n");
                }
                else {
                    _items.clear();
                }
            }

        private:
            std::vector<std::tuple<std::chrono::nanoseconds, spdlog::level::level_enum, std::string>> _items;
            z_owned_publisher_t _publisher;
            const char* _topic_prefix;
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
    SPDLOG_INFO("Invalid argument for h26x_profile: {0}", fmt.c_str());
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
    SPDLOG_INFO("Invalid aac_profile: {0}", fmt.c_str());
    return default;
}


// OK
struct MGR_StartRM {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartRM;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartRM");
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
        SPDLOG_DEBUG("MGR: StartRM done.");

        return true;
    }
};

// OK
struct MGR_StopRM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopRM");
        StopRM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopRM done.");
        return true;
    }
};

// OK
struct MGR_StartMC {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartMC;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartMC");
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
        SPDLOG_DEBUG("MGR: StartMC done.");
        return true;
    }
};

// OK
struct MGR_StopMC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopMC");
        StopMC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopMC done.");
        return true;
    }
};

// OK
struct MGR_StartPV {
    using RequestT = pcpd_msgs::rpc::HL2MGRRequest_StartPV;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartPV");
        H26xFormat pv_fmt;
        pv_fmt.width = request.pv_format().width();
        pv_fmt.height = request.pv_format().height();
        pv_fmt.framerate = request.pv_format().frame_rate();
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
        SPDLOG_DEBUG("MGR: StartPV done.");
        return true;
    }
};

// OK
struct MGR_StopPV {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopPV");
        StopPV(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopPV done.");
        return true;
    }
};


// OK
struct MGR_StartSI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartSI");
        StartSI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartSI done.");
        return true;
    }
};

// OK
struct MGR_StopSI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopSI");
        StopSI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopSI done.");
        return true;
    }
};

// OK
struct MGR_StartRC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartRC");
        StartRC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartRC done.");
        return true;
    }
};

// OK
struct MGR_StopRC {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopRC");
        StopRC(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopRC done.");
        return true;
    }
};

// OK
struct MGR_StartSM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartSM");
        StartSM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartSM done.");
        return true;
    }
};

// OK
struct MGR_StopSM {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopSM");
        StopSM(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopSM done.");
        return true;
    }
};

// OK
struct MGR_StartSU {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartSU");
        StartSU(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartSU done.");
        return true;
    }
};

// OK
struct MGR_StopSU {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopSU");
        StopSU(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopSU done.");
        return true;
    }
};

// OK
struct MGR_StartVI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartVI");
        StartVI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartVI done.");
        return true;
    }
};

// OK
struct MGR_StopVI {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopVI");
        StopVI(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopVI done.");
        return true;
    }
};

// OK
struct MGR_StartEET {
    using RequestT = pcpd_msgs::rpc::UInt8Request;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& request, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StartEET");
        StartEET(g_zenoh_context, request.value());
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StartEET done.");
        return true;
    }
};

// OK
struct MGR_StopEET {
    using RequestT = pcpd_msgs::rpc::NullRequest;
    using ResponseT = pcpd_msgs::rpc::NullReply;

    bool call(const RequestT& /*request*/, ResponseT& response, void* /*context*/) {
        SPDLOG_DEBUG("MGR: StopEET");
        StopEET(g_zenoh_context);
        response.status(pcpd_msgs::rpc::RPC_STATUS_SUCCESS);
        SPDLOG_DEBUG("MGR: StopEET done.");
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
            request.enable_depth_long_throw(args_extract<bool, false>(args, "depth_lt"));

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
            df.bitrate(args_extract<uint32_t, 8 * 1024 * 1024>(args, "ahat_bitrate"));

        }
        catch (const std::invalid_argument& e) {
            SPDLOG_INFO("MGR: invalid argument: {0}", e.what());
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
            SPDLOG_INFO("MGR: invalid argument: {0}", e.what());
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
            SPDLOG_INFO("MGR: invalid argument: {0}", e.what());
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
                SPDLOG_DEBUG("Received argument: {0} -> {1}", current->key, current->value);
            }
            current = current->next;
        }
    }
    uriFreeQueryListA(queryList);

    SPDLOG_INFO("MGR: called with: {0}", arguments);

    eprosima::fastcdr::FastBuffer result_buffer{};
    std::size_t result_bytes{0};
    bool call_success{ false };

    if (arguments.find("cmd") != arguments.end()) {
        auto cmd = arguments.extract("cmd");
        try {

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
        catch (const std::exception& e) {
            SPDLOG_ERROR("MGR: Error executing command: {0}", e.what());
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
    SPDLOG_INFO("MGR: EntryPoint Main Thread");

    if (!g_zenoh_context || !g_zenoh_context->valid) {
        OutputDebugStringA("MGR: Invalid Zenoh Context");
        return 1;
    }

    (void)param;

    // register zenoh logger
    auto cbs = std::make_shared< spdlog::sinks::zenoh_logging_sink_mt>();
    std::vector<spdlog::sink_ptr>& sinks = spdlog::details::registry::instance().get_default_raw()->sinks();
    cbs->set_pattern("%H:%M:%S.%e [%L] %v (%s:%#)");
    sinks.push_back(cbs);

    std::string keyexpr_str = g_zenoh_context->topic_prefix + "/rpc/mgr";
    SPDLOG_INFO("MGR: endpoint: {0}", keyexpr_str.c_str());

    z_keyexpr_t keyexpr = z_keyexpr(keyexpr_str.c_str());
    if (!z_check(keyexpr)) {
        SPDLOG_ERROR("MGR: {0} is not a valid key expression", keyexpr_str.c_str());
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
        SPDLOG_ERROR("MGR: Unable to create queryable.");
        return 1;
    }

    uint64_t heart_beat_counter{ 0 };

    eprosima::fastcdr::FastBuffer buffer{};
    eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
    std::string keyexpr1 = g_zenoh_context->topic_prefix + "/cfg/presence";

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

            value.header().frame_id(g_zenoh_context->topic_prefix);
        }
        value.heart_beat_counter(heart_beat_counter);

        buffer_cdr.reset();
        buffer_cdr.serialize_encapsulation();
        value.serialize(buffer_cdr);

        // put message to zenoh
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_ERROR("MGR: Error putting presence");
        }
        else {
            SPDLOG_INFO("MGR: put presence: {}", keyexpr1);
        }
    }


    // heartbeat every 5 seconds
    do
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

            value.header().frame_id(g_zenoh_context->topic_prefix);
        }
        value.heart_beat_counter(heart_beat_counter);

        buffer_cdr.reset();
        buffer_cdr.serialize_encapsulation();
        value.serialize(buffer_cdr);

        // put message to zenoh
        z_put_options_t options1 = z_put_options_default();
        options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
        int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
        if (res > 0) {
            SPDLOG_ERROR("MGR: Error putting presence");
        }
        else {
            SPDLOG_DEBUG("MGR: Heartbeat {0}", heart_beat_counter);
        }

        ++heart_beat_counter;
    } while (WaitForSingleObject(g_event_quit, 10*1000) == WAIT_TIMEOUT);

    z_delete_options_t options = z_delete_options_default();
    int res = z_delete(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), &options);
    if (res < 0) {
        SPDLOG_ERROR("MGR: delete presence failed");
    }

    z_undeclare_queryable(z_move(qable));
    SPDLOG_INFO("MGR: Closed");

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

    g_zenoh_context.reset();
}




bool StartManager(HC_Context_Ptr& context) {
    SPDLOG_INFO("StartManager...");
    if (!context) {
        SPDLOG_ERROR("StartManager: Invalid context");
        return false;
    }
    g_zenoh_context = context;

    MGR_Initialize();

    return true;
}

bool StopManager(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartManager: Invalid context");
        return false;
    }

    MGR_Quit();
    MGR_Cleanup();

    return true;
}




bool StartRM(HC_Context_Ptr& context,
    bool enable_location,
    bool enable_left_front, bool enable_left_left,
    bool enable_right_front, bool enable_right_right,
    H26xFormat vlc_format,
    bool enable_depth_ahat, bool enable_depth_long_throw,
    H26xFormat depth_format,
    bool enable_imu_accel, bool enable_imu_gyro, bool enable_imu_mag) {
    if (!context) {
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RM)) { 
        SPDLOG_WARN("StartRM: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_RM) { 
        SPDLOG_WARN("StartRM: ist already started");
        return false;
    }
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

    RM_Initialize(context, rm_config);

    return true;
}

bool StopRM(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopRM: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RM)) { 
        SPDLOG_WARN("StopRM: not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_RM)) { 
        SPDLOG_WARN("StopRM: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_RM;
    RM_Quit();
    RM_Cleanup();
    return true;
}

bool StartPV(HC_Context_Ptr& context,
    bool enable_location,
    H26xFormat pv_format) {
    if (!context) {
        SPDLOG_ERROR("StartPV: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_PV)) { 
        SPDLOG_WARN("StartPV: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_PV) { 
        SPDLOG_WARN("StartPV: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_PV;
    PV_Initialize(context, enable_location, pv_format);
    return true;
}

bool StopPV(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopPV: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_PV)) { 
        SPDLOG_WARN("StopPV: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_PV)) { 
        SPDLOG_WARN("StopPV: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_PV;
    PV_Quit();
    PV_Cleanup();
    return true;
}

bool StartMC(HC_Context_Ptr& context, AACFormat aac_format) {
    if (!context) {
        SPDLOG_ERROR("StartMC: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_MC)) { 
        SPDLOG_WARN("StartMC: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_MC) { 
        SPDLOG_WARN("StartMC: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_MC;
    MC_Initialize(context, aac_format);
    return true;
}

bool StopMC(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopMC: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_MC)) { 
        SPDLOG_WARN("StopMC: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_MC)) { 
        SPDLOG_WARN("StopMC: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_MC;
    MC_Quit();
    MC_Cleanup();
    return true;
}

bool StartSI(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartSI: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SI)) { 
        SPDLOG_WARN("StartSI: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_SI) { 
        SPDLOG_WARN("StartSI: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_SI;
    SI_Initialize(context);
    return true;
}

bool StopSI(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopSI: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SI)) { 
        SPDLOG_WARN("StopSI: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_SI)) { 
        SPDLOG_WARN("StopSI: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_SI;
    SI_Quit();
    SI_Cleanup();
    return true;
}

bool StartRC(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartRC: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RC)) { 
        SPDLOG_WARN("StartRC: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_RC) { 
        SPDLOG_WARN("StartRC: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_RC;
    RC_Initialize(context);
    return true;
}

bool StopRC(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopRC: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_RC)) { 
        SPDLOG_WARN("StopRC: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_RC)) { 
        SPDLOG_WARN("StopRC: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_RC;
    RC_Quit();
    RC_Cleanup();
    return true;
}

bool StartSM(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartSM: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SM)) { 
        SPDLOG_WARN("StartSM: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_SM) { 
        SPDLOG_WARN("StartSM: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_SM;
    SM_Initialize(context);
    return true;
}

bool StopSM(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopSM: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SM)) { 
        SPDLOG_WARN("StopSM: is not enabled");    
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_SM)) { 
        SPDLOG_WARN("StopSM: is not running");        
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_SM;
    SM_Quit();
    SM_Cleanup();
    return true;
}

bool StartSU(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartSU: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SU)) { 
        SPDLOG_WARN("StartSU: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_SU) { 
        SPDLOG_WARN("StartSU: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_SU;
    SU_Initialize(context);
    return true;
}

bool StopSU(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopSU: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_SU)) { 
        SPDLOG_WARN("StopSU: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_SU)) { 
        SPDLOG_WARN("StopSU: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_SU;
    SU_Quit();
    SU_Cleanup();
    return true;
}

bool StartVI(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StartVI: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_VI)) { 
        SPDLOG_WARN("StartVI: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_VI) { 
        SPDLOG_WARN("StartVI: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_VI;
    VI_Initialize(context);
    return true;
}

bool StopVI(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopVI: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_VI)) { 
        SPDLOG_WARN("StopVI: is not enbled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_VI)) { 
        SPDLOG_WARN("StopVI: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_VI;
    VI_Quit();
    VI_Cleanup();
    return true;
}

bool StartEET(HC_Context_Ptr& context, uint8_t eye_fps) {
    if (!context) {
        SPDLOG_ERROR("StartEET: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_EET)) { 
        SPDLOG_WARN("StartEET: is not enabled");
        return false;
    }
    if (context->streams_started & HL2SS_ENABLE_EET) { 
        SPDLOG_WARN("StartEET: is already running");
        return false;
    }
    context->streams_started |= HL2SS_ENABLE_EET;
    EET_Initialize(context, eye_fps);
    return true;
}

bool StopEET(HC_Context_Ptr& context) {
    if (!context) {
        SPDLOG_ERROR("StopEET: Invalid context");
        return false;
    }
    if (!(context->streams_enabled & HL2SS_ENABLE_EET)) { 
        SPDLOG_WARN("StopEET: is not enabled");
        return false;
    }
    if (!(context->streams_started & HL2SS_ENABLE_EET)) { 
        SPDLOG_WARN("StopEET: is not running");
        return false;
    }
    context->streams_started &= ~HL2SS_ENABLE_EET;
    EET_Quit();
    EET_Cleanup();
    return true;
}