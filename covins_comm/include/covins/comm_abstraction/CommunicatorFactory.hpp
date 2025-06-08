//
// Created by user1 on 08/06/25.
//

#ifndef COMMUNICATORFACTORY_HPP
#define COMMUNICATORFACTORY_HPP

#pragma once

#include <string>
#include <memory> // For std::unique_ptr

// Forward declarations for the interfaces this factory will create
namespace covins {
    class ICommunicator;
    class ISerializer;
    class IDeserializer;
    class IMessage; // Potentially needed if creating specific message types via factory
}

namespace covins {

/**
 * @brief Factory class for creating instances of ICommunicator, ISerializer, and IDeserializer.
 *
 * This class encapsulates the creation logic for protocol-specific communication and
 * serialization objects. It uses configuration parameters to dynamically determine
 * which concrete implementation to instantiate, adhering to the Factory Method pattern.
 */
class CommunicatorFactory {
public:
    /**
     * @brief Creates a concrete instance of ICommunicator based on the specified protocol type.
     * @param protocol_type The string identifier for the desired communication protocol (e.g., "ros1", "ros2", "tcp").
     * @param serialization_format The string identifier for the desired serialization format (e.g., "protobuf", "cereal").
     * @param address The address or identifier for the communication endpoint.
     * @param port The port number for the communication endpoint.
     * @return A unique_ptr to the created ICommunicator instance, or nullptr if the type is unknown.
     */
    static std::unique_ptr<ICommunicator> createCommunicator(
        const std::string& protocol_type,
        const std::string& serialization_format,
        const std::string& address,
        int port
    );

    /**
     * @brief Creates a concrete instance of ISerializer based on the specified format.
     * @param serialization_format The string identifier for the desired serialization format.
     * @return A unique_ptr to the created ISerializer instance, or nullptr if the format is unknown.
     */
    static std::unique_ptr<ISerializer> createSerializer(
        const std::string& serialization_format
    );

    /**
     * @brief Creates a concrete instance of IDeserializer based on the specified format.
     * @param serialization_format The string identifier for the desired serialization format.
     * @return A unique_ptr to the created IDeserializer instance, or nullptr if the format is unknown.
     */
    static std::unique_ptr<IDeserializer> createDeserializer(
        const std::string& serialization_format
    );

    // Potentially add a method for creating specific IMessage types if needed in the future,
    // though typically messages are instantiated directly by the sending module.
    // static std::unique_ptr<IMessage> createMessage(const std::string& message_type);
};

} // namespace covins

#endif //COMMUNICATORFACTORY_HPP
