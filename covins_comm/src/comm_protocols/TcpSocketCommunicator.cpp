#include <covins/comm_protocols/TcpSocketCommunicator.hpp>
#include <iostream>
#include <arpa/inet.h> // For inet_pton, inet_ntop
#include <sys/socket.h> // For socket, connect, send, recv, setsockopt, SOL_SOCKET, SO_RCVBUF, SO_SNDBUF
#include <unistd.h> // For close
#include <netdb.h> // For gethostbyname, struct hostent
#include <stdexcept> // For std::runtime_error
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::milliseconds
#include <algorithm> // For std::min

// Include concrete message types (REQUIRED for deserialization switch)
#include <covins/comm_messages/MsgImage.hpp>
#include <covins/comm_messages/MsgOdometry.hpp>
#include <covins/comm_messages/MsgKeyframe.hpp>
#include <covins/comm_messages/MsgLandmark.hpp>

// Include CommunicatorFactory for creating serializers/deserializers
#include <covins/comm_abstraction/CommunicatorFactory.hpp>


namespace covins {

// Define header constants for TCP framing
const uint32_t MESSAGE_SIZE_HEADER_BYTES = 4; // Size of the header indicating message length
const uint32_t MESSAGE_TYPE_HEADER_BYTES = 1; // Size of the header indicating message type length
const uint32_t MESSAGE_TYPE_MAX_LENGTH = 32;  // Max length of the string message type

TcpSocketCommunicator::TcpSocketCommunicator(const std::string& serialization_format,
                                             const std::string& address,
                                             int port)
    : socket_fd_(-1),
      serialization_format_(serialization_format),
      address_(address),
      port_(port),
      connected_(false)
{
    // Create serializer and deserializer based on the format
    serializer_ = CommunicatorFactory::createSerializer(serialization_format_);
    deserializer_ = CommunicatorFactory::createDeserializer(serialization_format_);

    if (!serializer_ || !deserializer_) {
        throw std::runtime_error("Failed to create serializer or deserializer for format: " + serialization_format_);
    }
}

TcpSocketCommunicator::~TcpSocketCommunicator() {
    disconnect(); // Ensure socket is closed on destruction
}

bool TcpSocketCommunicator::connect(const std::string& address, int port) {
    if (connected_) {
        std::cerr << "TcpSocketCommunicator: Already connected." << std::endl;
        return true;
    }

    address_ = address;
    port_ = port;

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == -1) {
        std::cerr << "TcpSocketCommunicator: Failed to create socket." << std::endl;
        return false;
    }

    // Set buffer sizes for performance (optional, but good practice for large data)
    int buffer_size = 1024 * 1024 * 10; // 10 MB
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
        std::cerr << "TcpSocketCommunicator: Failed to set receive buffer size." << std::endl;
    }
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size)) < 0) {
        std::cerr << "TcpSocketCommunicator: Failed to set send buffer size." << std::endl;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_); // Convert port to network byte order

    // Convert address string to binary form
    // Handles both IP addresses and hostnames
    struct hostent* host_entry;
    struct in_addr** addr_list;

    if (inet_pton(AF_INET, address_.c_str(), &server_addr.sin_addr) <= 0) {
        // Not a valid IP address, try resolving as hostname
        host_entry = gethostbyname(address_.c_str());
        if (host_entry == nullptr) {
            std::cerr << "TcpSocketCommunicator: Failed to resolve hostname: " << address_ << std::endl;
            close(socket_fd_);
            return false;
        }
        addr_list = (struct in_addr **)host_entry->h_addr_list;
        server_addr.sin_addr = *addr_list[0]; // Use the first resolved address
    }

    if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "TcpSocketCommunicator: Failed to connect to " << address_ << ":" << port_ << std::endl;
        close(socket_fd_);
        return false;
    }

    connected_ = true;
    std::cout << "TcpSocketCommunicator: Successfully connected to " << address_ << ":" << port_ << std::endl;

    // Start a receive thread if a callback is registered
    if (receive_callback_) {
        // Ensure only one receive thread is active
        if (receive_thread_.joinable()) {
            receive_thread_.join(); // Join existing thread if any
        }
        receive_thread_ = std::thread(&TcpSocketCommunicator::receiveLoop, this);
    }

    return true;
}

bool TcpSocketCommunicator::disconnect() {
    if (!connected_) {
        std::cout << "TcpSocketCommunicator: Not connected." << std::endl;
        return true;
    }

    connected_ = false; // Signal receiveLoop to stop

    if (receive_thread_.joinable()) {
        receive_thread_.join(); // Wait for receive thread to finish
    }

    if (socket_fd_ != -1) {
        close(socket_fd_); // Close the socket
        socket_fd_ = -1;
        std::cout << "TcpSocketCommunicator: Disconnected." << std::endl;
        return true;
    }
    return false;
}

bool TcpSocketCommunicator::send(const IMessage& message) {
    if (!connected_) {
        std::cerr << "TcpSocketCommunicator: Not connected. Cannot send message." << std::endl;
        return false;
    }

    // Reset serializer for new message
    serializer_->reset();
    message.serialize(*serializer_);
    std::vector<uint8_t> serialized_data = serializer_->getSerializedData();

    if (serialized_data.empty()) {
        std::cerr << "TcpSocketCommunicator: Failed to serialize message of type " << message.getType() << std::endl;
        return false;
    }

    // Get message type as a string
    std::string message_type_str = message.getType();

    // Ensure message type string length does not exceed max allowed
    if (message_type_str.length() > MESSAGE_TYPE_MAX_LENGTH) {
        std::cerr << "TcpSocketCommunicator: Message type string too long. Max allowed: " << MESSAGE_TYPE_MAX_LENGTH << " bytes." << std::endl;
        return false;
    }

    // --- Prepare header for message type length ---
    uint32_t type_len_network = htonl(static_cast<uint32_t>(message_type_str.length()));
    std::vector<uint8_t> type_len_header(MESSAGE_TYPE_HEADER_BYTES);
    std::memcpy(type_len_header.data(), &type_len_network, MESSAGE_TYPE_HEADER_BYTES);

    // --- Prepare header for total message size (type_len + type_str + data_len + data) ---
    uint32_t total_message_size = MESSAGE_TYPE_HEADER_BYTES + message_type_str.length() + MESSAGE_SIZE_HEADER_BYTES + serialized_data.size();
    uint32_t total_size_network = htonl(total_message_size);
    std::vector<uint8_t> size_header(MESSAGE_SIZE_HEADER_BYTES);
    std::memcpy(size_header.data(), &total_size_network, MESSAGE_SIZE_HEADER_BYTES);

    // Combine all parts: (total_size_header | type_len_header | type_str | data_size_header | serialized_data)
    std::vector<uint8_t> full_packet;
    full_packet.insert(full_packet.end(), size_header.begin(), size_header.end());
    full_packet.insert(full_packet.end(), type_len_header.begin(), type_len_header.end());
    full_packet.insert(full_packet.end(), message_type_str.begin(), message_type_str.end());
    full_packet.insert(full_packet.end(), serialized_data.begin(), serialized_data.end()); // Actual data follows type string


    ssize_t bytes_sent = ::send(socket_fd_, full_packet.data(), full_packet.size(), 0);
    if (bytes_sent == -1) {
        std::cerr << "TcpSocketCommunicator: Failed to send message of type " << message.getType() << ". Error: " << strerror(errno) << std::endl;
        return false;
    }
    if (static_cast<size_t>(bytes_sent) != full_packet.size()) {
        std::cerr << "TcpSocketCommunicator: Sent incomplete message for type " << message.getType() << ". Expected " << full_packet.size() << ", sent " << bytes_sent << std::endl;
        return false;
    }

    // std::cout << "TcpSocketCommunicator: Sent " << bytes_sent << " bytes for message type: " << message.getType() << std::endl;
    return true;
}

std::unique_ptr<IMessage> TcpSocketCommunicator::receive() {
    // This synchronous receive is generally discouraged for continuous streams.
    // Use the callback mechanism for better performance in real-time systems.
    if (!connected_) {
        std::cerr << "TcpSocketCommunicator: Not connected. Cannot receive message." << std::endl;
        return nullptr;
    }

    // Implement a blocking receive, potentially with timeout.
    // For simplicity, this example will just call internal receive logic.
    // In a production system, this would likely involve polling or a dedicated thread.
    // This method is primarily here to fulfill the ICommunicator interface contract.

    std::cerr << "TcpSocketCommunicator: 'receive()' called. Consider using 'registerReceiveCallback' for asynchronous handling." << std::endl;
    // In a real implementation, you'd likely have a blocking read here or
    // a mechanism to fetch a message from an internal queue populated by the receiveLoop.
    // For now, we'll return nullptr, expecting users to rely on the callback.
    return nullptr;
}

void TcpSocketCommunicator::receiveLoop() {
    std::vector<uint8_t> buffer;
    buffer.reserve(1024 * 1024 * 5); // Reserve 5MB for incoming data

    while (connected_) {
        // Step 1: Read the total message size header (4 bytes)
        uint32_t total_message_size;
        if (!readExactly(socket_fd_, reinterpret_cast<uint8_t*>(&total_message_size), MESSAGE_SIZE_HEADER_BYTES)) {
            if (connected_) { // Only log error if still connected (not graceful disconnect)
                std::cerr << "TcpSocketCommunicator: Failed to read total message size header or disconnected." << std::endl;
            }
            break;
        }
        total_message_size = ntohl(total_message_size); // Convert from network byte order

        if (total_message_size == 0) {
            std::cerr << "TcpSocketCommunicator: Received zero-length message, skipping." << std::endl;
            continue;
        }

        // Adjust for headers already read if total_message_size includes them
        // Our send logic includes MESSAGE_SIZE_HEADER_BYTES + MESSAGE_TYPE_HEADER_BYTES
        uint32_t remaining_size = total_message_size - MESSAGE_SIZE_HEADER_BYTES;

        // Step 2: Read the message type length header (1 byte)
        uint32_t message_type_len;
        if (!readExactly(socket_fd_, reinterpret_cast<uint8_t*>(&message_type_len), MESSAGE_TYPE_HEADER_BYTES)) {
            if (connected_) {
                std::cerr << "TcpSocketCommunicator: Failed to read message type length header or disconnected." << std::endl;
            }
            break;
        }
        message_type_len = ntohl(message_type_len); // Convert from network byte order

        if (message_type_len == 0 || message_type_len > MESSAGE_TYPE_MAX_LENGTH) {
            std::cerr << "TcpSocketCommunicator: Invalid message type length (" << message_type_len << "), skipping message." << std::endl;
            // Need to discard the rest of the message to stay in sync
            std::vector<uint8_t> discard_buffer(remaining_size - MESSAGE_TYPE_HEADER_BYTES);
            readExactly(socket_fd_, discard_buffer.data(), discard_buffer.size());
            continue;
        }
        remaining_size -= MESSAGE_TYPE_HEADER_BYTES;


        // Step 3: Read the message type string
        std::vector<uint8_t> message_type_bytes(message_type_len);
        if (!readExactly(socket_fd_, message_type_bytes.data(), message_type_len)) {
            if (connected_) {
                std::cerr << "TcpSocketCommunicator: Failed to read message type string or disconnected." << std::endl;
            }
            break;
        }
        std::string message_type(message_type_bytes.begin(), message_type_bytes.end());
        remaining_size -= message_type_len;


        // Step 4: Read the actual serialized data (remaining_size)
        // Check for buffer capacity and resize if necessary
        if (buffer.capacity() < remaining_size) {
            buffer.reserve(remaining_size); // Ensure enough capacity
        }
        buffer.resize(remaining_size); // Resize to exact incoming data size

        if (!readExactly(socket_fd_, buffer.data(), remaining_size)) {
            if (connected_) {
                std::cerr << "TcpSocketCommunicator: Failed to read message data or disconnected." << std::endl;
            }
            break;
        }

        // Deserialize and dispatch message
        std::unique_ptr<IMessage> received_msg = deserializeMessage(message_type, buffer);

        if (received_msg && receive_callback_) {
            receive_callback_(std::move(received_msg)); // Use std::move to transfer ownership
        }
    }
    std::cout << "TcpSocketCommunicator: Receive loop stopped." << std::endl;
}


// Helper function to read exactly 'bytes_to_read' from the socket
bool TcpSocketCommunicator::readExactly(int socket_fd, uint8_t* buffer, size_t bytes_to_read) {
    size_t total_bytes_read = 0;
    while (total_bytes_read < bytes_to_read) {
        ssize_t bytes_read = ::recv(socket_fd, buffer + total_bytes_read, bytes_to_read - total_bytes_read, 0);
        if (bytes_read == 0) { // Connection closed by peer
            std::cerr << "TcpSocketCommunicator: Peer closed connection during readExactly." << std::endl;
            connected_ = false; // Signal disconnection
            return false;
        }
        if (bytes_read == -1) { // Error
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Non-blocking socket would return here. With blocking, it waits.
                // This branch for completeness if socket turns non-blocking.
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small delay
                continue;
            }
            std::cerr << "TcpSocketCommunicator: Socket error during readExactly: " << strerror(errno) << std::endl;
            connected_ = false;
            return false;
        }
        total_bytes_read += bytes_read;
    }
    return true;
}


std::unique_ptr<IMessage> TcpSocketCommunicator::deserializeMessage(const std::string& message_type, const std::vector<uint8_t>& data) {
    deserializer_->setData(data); // Set the data for the deserializer

    std::unique_ptr<IMessage> msg = nullptr;

    // Use if-else if chain to handle different message types.
    // This is a common pattern when a factory method cannot be used due to abstract return type.
    if (message_type == "Keyframe") {
        msg = std::make_unique<MsgKeyframe>();
    } else if (message_type == "Image") {
        msg = std::make_unique<MsgImage>();
    } else if (message_type == "Odometry") {
        msg = std::make_unique<MsgOdometry>();
    } else if (message_type == "Landmark") {
        msg = std::make_unique<MsgLandmark>();
    } else {
        std::cerr << "TcpSocketCommunicator: Received unknown message type: " << message_type << std::endl;
        return nullptr;
    }

    if (msg) {
        msg->deserialize(*deserializer_);
    }
    return msg;
}

bool TcpSocketCommunicator::isConnected() const {
    return connected_;
}

void TcpSocketCommunicator::registerReceiveCallback(std::function<void(std::unique_ptr<IMessage>)> callback) {
    receive_callback_ = std::move(callback);
    // If already connected, start the receive thread now.
    if (connected_ && !receive_thread_.joinable()) {
        receive_thread_ = std::thread(&TcpSocketCommunicator::receiveLoop, this);
    }
}

} // namespace covins
