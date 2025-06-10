#pragma once

#include <string>
#include <iostream>
#include <ros/ros.h> // Include ROS for parameter access
#include <covins/covins_base/typedefs_base.hpp> // For COUTERROR

namespace covins {

namespace covins_params { // Nested namespace

// These functions fetch parameters from the ROS parameter server.
// They are defined as `static inline` so their implementations are
// available directly in any translation unit that includes this header,
// eliminating the need for a separate `config_comm.cpp` for these definitions.

static inline std::string GetCommProtocolType() {
    std::string type;
    if (!ros::param::get("~communication_protocol_type", type)) {
        COUTWARN << "Parameter 'communication_protocol_type' not found, defaulting to 'tcp'" << std::endl;
        type = "tcp";
    }
    return type;
}

static inline std::string GetCommSerializationFormat() {
    std::string format;
    if (!ros::param::get("~communication_serialization_format", format)) {
        COUTWARN << "Parameter 'communication_serialization_format' not found, defaulting to 'protobuf'" << std::endl;
        format = "protobuf";
    }
    return format;
}

static inline std::string GetServerIP() {
    std::string ip;
    if (!ros::param::get("~server_ip", ip)) {
        COUTWARN << "Parameter 'server_ip' not found, defaulting to '127.0.0.1'" << std::endl;
        ip = "127.0.0.1";
    }
    return ip;
}

static inline int GetPort() { // Returns int, as expected by frontend_wrapper.cpp
    int port;
    if (!ros::param::get("~port", port)) {
        COUTWARN << "Parameter 'port' not found, defaulting to 8080" << std::endl;
        port = 8080;
    }
    return port;
}

static inline void ShowParamsComm() {
    COUTINFO << "Communication Parameters:" << std::endl;
    COUTINFO << "  Protocol Type: " << GetCommProtocolType() << std::endl;
    COUTINFO << "  Serialization Format: " << GetCommSerializationFormat() << std::endl;
    COUTINFO << "  Server IP: " << GetServerIP() << std::endl;
    COUTINFO << "  Port: " << GetPort() << std::endl;
}

} // namespace covins_params

} // namespace covins
