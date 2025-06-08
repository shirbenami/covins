//
// Created by user1 on 08/06/25.
//

#ifndef ICOMMUNICATOR_HPP
#define ICOMMUNICATOR_HPP

#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <functional> // For std::function

// Forward declaration of IMessage to avoid circular dependencies
// The ICommunicator interface interacts with IMessage, but doesn't need
// its full definition in the header.
namespace covins {
    class IMessage;
}

namespace covins {

/**
 * @brief Abstract interface for communication channels.
 *
 * This interface defines the contract for sending and receiving data,
 * abstracting away the underlying protocol-specific details (e.g., ROS1, ROS2, TCP).
 * Any concrete communication implementation (e.g., Ros1Communicator, TcpSocketCommunicator)
 * must inherit from and implement this interface.
 */
class ICommunicator {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~ICommunicator() = default;

    /**
     * @brief Connects to the communication endpoint (e.g., server, ROS master).
     * @param address The address or identifier of the communication endpoint.
     * @param port The port number, if applicable.
     * @return True if connection was successful, false otherwise.
     */
    virtual bool connect(const std::string& address, int port) = 0;

    /**
     * @brief Disconnects from the communication endpoint.
     * @return True if disconnection was successful, false otherwise.
     */
    virtual bool disconnect() = 0;

    /**
     * @brief Sends an IMessage object through the communication channel.
     *
     * This method expects a concrete IMessage object, which will internally
     * use an ISerializer to convert its data into a byte stream
     * suitable for transmission by the concrete communicator.
     *
     * @param message The IMessage object to send. Must be a concrete implementation.
     * @return True if the message was successfully sent, false otherwise.
     */
    virtual bool send(const IMessage& message) = 0;

    /**
     * @brief Receives an IMessage object from the communication channel (blocking or polling).
     *
     * Note: For real-time systems, it's generally preferred to use registerReceiveCallback
     * for asynchronous message handling to avoid blocking the main thread.
     *
     * @return A unique_ptr to the received IMessage object, or nullptr if no message
     * was received or an error occurred. The caller takes ownership.
     */
    virtual std::unique_ptr<IMessage> receive() = 0;

    /**
     * @brief Checks if the communication channel is currently connected.
     * @return True if connected, false otherwise.
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief Registers a callback function to be invoked when a new message is received asynchronously.
     *
     * This is the preferred method for handling incoming messages in a non-blocking way.
     * The concrete communicator implementation will call this registered function
     * after it has received and deserialized an incoming message into an IMessage object.
     *
     * @param callback A function object (e.g., lambda, std::bind) that accepts a
     * std::unique_ptr<IMessage> as an argument.
     */
    virtual void registerReceiveCallback(std::function<void(std::unique_ptr<IMessage>)> callback) = 0;
};

} // namespace covins
#endif //ICOMMUNICATOR_HPP
