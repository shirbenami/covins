//
// Created by user1 on 08/06/25.
//
#include <covins/comm_abstraction/CommunicatorFactory.hpp>
#include <iostream> // For error logging (can be replaced with a proper logging framework)

// Include concrete communicator implementations (these files will be created later)
// For now, we'll assume their existence.
#include <covins/comm_protocols/Ros1Communicator.hpp> // Example: for ROS1
#include <covins/comm_protocols/Ros2Communicator.hpp> // Example: for ROS2
#include <covins/comm_protocols/TcpSocketCommunicator.hpp> // Example: for TCP sockets

// Include concrete serializer/deserializer implementations (these files will be created later)
#include <covins/comm_serialization/ProtobufSerializer.hpp> // Example: for Protocol Buffers
#include <covins/comm_serialization/CerealSerializer.hpp>   // Example: for Cereal

namespace covins {

std::unique_ptr<ICommunicator> CommunicatorFactory::createCommunicator(
    const std::string& protocol_type,
    const std::string& serialization_format, // This might be used by the communicator internally
    const std::string& address,
    int port) {

    // First, create the appropriate serializer/deserializer for the communicator
    // Note: The Communicator itself might not directly use the serializer/deserializer
    // interfaces for its internal ROS/TCP communication, but rather rely on
    // IMessage's serialize/deserialize methods with an ISerializer/IDeserializer.
    // However, some communicators might need to know the serialization_format.

    if (protocol_type == "ros1") {
        // In a real ROS1 implementation, you might pass the serialization_format
        // to the Ros1Communicator constructor if it needs to prepare its internal
        // ROS message types based on the expected serialization (e.g., for custom ROS messages).
        // For standard ROS messages, the ROS layer handles its own serialization/deserialization.
        // For simplicity, we'll assume it handles serialization internally based on IMessage's methods.
        return std::make_unique<Ros1Communicator>(address, port);
    } else if (protocol_type == "ros2") {
        // Similar to ROS1, a Ros2Communicator would manage DDS/rclcpp communication.
        // TODO: temp replace
        return std::make_unique<Ros1Communicator>(address, port);
    } else if (protocol_type == "tcp") {
        // The TcpSocketCommunicator will definitely need to know the serialization format
        // to correctly frame and unfame messages, and to pass the correct (de)serializer
        // to the IMessage objects.
        return std::make_unique<TcpSocketCommunicator>(serialization_format, address, port);
    } else {
        std::cerr << "ERROR: Unknown communication protocol type: " << protocol_type << std::endl;
        return nullptr;
    }
}

std::unique_ptr<ISerializer> CommunicatorFactory::createSerializer(
    const std::string& serialization_format) {

    if (serialization_format == "protobuf") {
        return std::make_unique<ProtobufSerializer>();
    } else if (serialization_format == "cereal") {
        return std::make_unique<CerealSerializer>();
    } else {
        std::cerr << "ERROR: Unknown serialization format: " << serialization_format << std::endl;
        return nullptr;
    }
}

std::unique_ptr<IDeserializer> CommunicatorFactory::createDeserializer(
    const std::string& serialization_format) {

    if (serialization_format == "protobuf") {
        return std::make_unique<ProtobufSerializer>(); // ProtobufSerializer will typically implement IDeserializer too
    } else if (serialization_format == "cereal") {
        return std::make_unique<CerealSerializer>();   // CerealSerializer will typically implement IDeserializer too
    } else {
        std::cerr << "ERROR: Unknown deserialization format: " << serialization_format << std::endl;
        return nullptr;
    }
}

} // namespace covins
