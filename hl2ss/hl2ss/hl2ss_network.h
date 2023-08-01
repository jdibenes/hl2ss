#pragma once

#include <functional>
#include <sstream>

#include "log.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "zenoh.h"

#include "pcpd_msgs/rpc/Types.h"



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
        try {
            RpcRequestArgs<T::RequestT> ah{};
            args_valid = ah.parse(args, request);
        } catch(const std::exception& e) {
            ShowMessage("RC: error parsing parameters: %s", e.what());
            args_valid = false;
        }
    }

    if (args_valid) {
        call_success = handler.call(request, response, context);
    }
    else {
        response.status(pcpd_msgs::rpc::RPC_STATUS_ERROR);
    }

    eprosima::fastcdr::Cdr buffer_cdr(result_buffer);
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
        ShowMessage("RC: ArgumentHelper used for unregistered type..");
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


