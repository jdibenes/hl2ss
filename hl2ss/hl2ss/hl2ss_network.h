#pragma once

#include <functional>
#include <sstream>
#include <memory>

#include "log.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "zenoh.h"

#include "pcpd_msgs/rpc/Types.h"


#define HL2SS_ENABLE_RM    1
#define HL2SS_ENABLE_PV    2
#define HL2SS_ENABLE_MC    4
#define HL2SS_ENABLE_SI    8
#define HL2SS_ENABLE_RC   16
#define HL2SS_ENABLE_SM   32
#define HL2SS_ENABLE_SU   64
#define HL2SS_ENABLE_VI  128
#define HL2SS_ENABLE_MQ  256
#define HL2SS_ENABLE_EET 512

struct HC_Context {
    std::string topic_prefix;
    z_owned_session_t session;
    uint32_t streams_enabled;
    uint32_t streams_started;
    bool valid{ false };
    bool should_exit{ false };
};

typedef std::shared_ptr<HC_Context> HC_Context_Ptr;

/*
* Call Helper
*/

template<typename T>
bool forward_rpc_call(T handler, void* context, z_value_t& request_payload, const std::map<std::string, std::string>& args,
    eprosima::fastcdr::FastBuffer& result_buffer, std::size_t& result_bytes) {

    T::RequestT request{};
    T::ResponseT response{};

    // parse arguments for request type
    bool args_valid{ false };
    bool call_success{ false };
    if (request_payload.payload.len > 0) {

        // deserialize payload
        eprosima::fastcdr::FastBuffer request_buffer(
            const_cast<char*>(reinterpret_cast<const char*>(request_payload.payload.start)),
            request_payload.payload.len);

        eprosima::fastcdr::Cdr request_buffer_cdr(request_buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        try {
            request.deserialize(request_buffer_cdr);
            args_valid = true;
        }
        catch (const eprosima::fastcdr::exception::Exception& e) {
            SPDLOG_INFO("RC: error deserializing request payload: {0}", e.what());
            args_valid = false;
        }
    }
    else {
        try {
            RpcRequestArgs<T::RequestT> ah{};
            args_valid = ah.parse(args, request);
        } catch(const std::exception& e) {
            SPDLOG_INFO("RC: error parsing parameters: {0}", e.what());
            args_valid = false;
        }
    }

    if (args_valid) {
        call_success = handler.call(request, response, context);
    }
    else {
        response.status(pcpd_msgs::rpc::RPC_STATUS_ERROR);
    }

    eprosima::fastcdr::Cdr buffer_cdr(result_buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
    call_success &= response.status() == pcpd_msgs::rpc::RPC_STATUS_SUCCESS;
    response.serialize(buffer_cdr);
    result_bytes = buffer_cdr.getSerializedDataLength();

    return call_success;
}


/*
* ArgumentHelper
*/

template<typename T, T Default>
T args_extract(const std::map<std::string, std::string>& args, const std::string key) {
    if (args.find(key) == args.end()) {
        return Default;
    }
    T v;
    if constexpr (std::is_same_v<T, bool>) {
        std::istringstream(args.at(key)) >> std::boolalpha >> v;
    }
    else {
        std::istringstream(args.at(key)) >> v;
    }
    return v;
}


template<typename RequestType>
struct RpcRequestArgs {
    bool parse(const std::map<std::string, std::string>& /*args*/, RequestType& /*value*/) {
        SPDLOG_WARN("RC: ArgumentHelper used for unregistered type..");
        return false;
    }
};

// concrete implementations

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::NullRequest> {
    bool parse(const std::map<std::string, std::string>& /*args*/, pcpd_msgs::rpc::NullRequest& /*request*/) {
        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::UInt32Request> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::UInt32Request& request) {
        if (args.find("value") == args.end()) {
            SPDLOG_INFO("RC: missing argument: value");
            return false;
        }
        try {
            auto value = std::stoi(args.at("value"));
            // bounds check ??
            request.value(static_cast<uint32_t>(value));
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_ERROR("RC: invalid argument: {0}", e.what());
            return false;
        }
        return true;
    }
};

template<>
struct RpcRequestArgs<pcpd_msgs::rpc::UInt8Request> {
    bool parse(const std::map<std::string, std::string>& args, pcpd_msgs::rpc::UInt8Request& request) {
        if (args.find("value") == args.end()) {
            SPDLOG_INFO("RC: missing argument: value");
            return false;
        }
        try {
            auto value = std::stoi(args.at("value"));
            if (value > 255 || value < 0) {
                SPDLOG_ERROR("RC: Invalid eye_fps: {0}", value);
                return false;
            }
            else {
                request.value(static_cast<uint8_t>(value));
            }
        }
        catch (const std::invalid_argument& e) {
            SPDLOG_ERROR("RC: invalid argument: {0}", e.what());
            return false;
        }
        return true;
    }
};



