//
// Created by user1 on 08/06/25.
//

#ifndef TCPSOCKETCOMMUNICATOR_HPP
#define TCPSOCKETCOMMUNICATOR_HPP

#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <functional> // For std::function
#include <thread>   // For std::thread
#include <mutex>    // For std::mutex
#include <queue>    // For internal message buffering

// POSIX socket includes
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h> // For non-blocking sockets

// Core abstraction layer interfaces
#include <covins/comm_abstraction/ICommunicator.hpp>
#include <covins/comm_abstraction/IMessage.hpp>
#include <covins/comm_abstraction/ISerializer.hpp> // For serializing/deserializing IMessage objects

namespace covins {

/**
 * @brief Concrete implementation of ICommunicator for TCP Socket communication (Client role).
 *
 * This class establishes and manages a TCP connection, sends serialized IMessage objects,
 * and receives incoming messages, dispatching them via a registered callback.
 * It uses a length-prefixing strategy for reliable message framing over the TCP stream.
 */
class TcpSocketCommunicator : public ICommunicator {
public:
    /**
     * @brief Constructor for TcpSocketCommunicator.
     * @param serialization_format The format to use for serializing/deserializing messages (e.g., "protobuf", "cereal").
     * @param address The IP address or hostname of the server to connect to.
     * @param port The port number of the server.
     */
    TcpSocketCommunicator(const std::string& serialization_format,
                          const std::string& address,
                          int port);

    /**
     * @brief Destructor. Closes the socket and stops communication threads.
     */
    virtual ~TcpSocketCommunicator();

    /**
     * @brief Connects to the specified TCP server.
     * @param address The IP address or hostname of the server.
     * @param port The port number of the server.
     * @return True if connection was successful, false otherwise.
     */
    bool connect(const std::string& address, int port) override;

    /**
     * @brief Disconnects from the TCP server.
     * @return True if disconnection was successful, false otherwise.
     */
    bool disconnect() override;

    /**
     * @brief Sends an IMessage object through the TCP connection.
     * The message is serialized, length-prefixed, and then sent.
     * @param message The IMessage object to send.
     * @return True if the message was successfully sent, false otherwise.
     */
    bool send(const IMessage& message) override;

    /**
     * @brief Receives an IMessage object from the TCP connection (blocking or polling).
     * In this callback-driven design, this method primarily services internal buffers
     * or indicates that messages are handled asynchronously.
     * @return A unique_ptr to the received IMessage object, or nullptr.
     */
    std::unique_ptr<IMessage> receive() override;

    /**
     * @brief Checks if the TCP socket is currently connected.
     * @return True if connected, false otherwise.
     */
    bool isConnected() const override;

    /**
     * @brief Registers a callback function for asynchronous message reception.
     * This callback will be invoked by the internal receiving thread when a full
     * IMessage is received and deserialized.
     * @param callback The function to call with received IMessage objects.
     */
    void registerReceiveCallback(std::function<void(std::unique_ptr<IMessage>)> callback) override;

private:
    // --- Connection and Socket Members ---
    int sock_fd_;                 // Socket file descriptor
    std::string server_address_;  // Server IP address or hostname
    int server_port_;             // Server port
    std::atomic<bool> connected_; // Connection status (atomic for thread safety)

    // --- Threading and Buffering for Receive ---
    std::unique_ptr<std::thread> recv_thread_; // Thread for continuous receiving
    std::atomic<bool> stop_recv_thread_;       // Flag to stop the receive thread

    std::function<void(std::unique_ptr<IMessage>)> receive_callback_; // Registered callback

    // Internal receive buffer for partial messages
    std::vector<uint8_t> recv_buffer_data_;
    size_t current_msg_size_; // Expected size of the current message being received
    bool expecting_message_size_; // True if we are waiting for the 4-byte size prefix

    // --- Serialization/Deserialization ---
    std::string serialization_format_; // Stores the chosen serialization format
    std::unique_ptr<ISerializer> serializer_;   // Used for sending
    std::unique_ptr<IDeserializer> deserializer_;// Used for receiving


    // --- Private Helper Methods ---
    void receiveThreadLoop(); // The function executed by recv_thread_
    bool sendBytes(const std::vector<uint8_t>& data); // Helper to send raw bytes
    int receiveBytes(std::vector<uint8_t>& buffer, size_t num_bytes); // Helper to receive raw bytes
    void processReceivedData(const std::vector<uint8_t>& new_data); // Processes incoming data stream
    std::unique_ptr<IMessage> deserializeMessage(const std::vector<uint8_t>& data); // Helper to deserialize
};

} // namespace covins

#endif //TCPSOCKETCOMMUNICATOR_HPP
