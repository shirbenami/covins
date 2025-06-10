#include <covins/comm_messages/MsgOdometry.hpp>
#include <iostream> // For error/debug output

namespace covins {

    // Constructor definition removed as it's defined in the header.
    // MsgOdometry::MsgOdometry()
    //     : timestamp(0.0), transform(Eigen::Matrix4d::Identity())
    // {
    //     // Default initializes transform to Identity
    // }

    std::unique_ptr<IMessage> MsgOdometry::clone() const {
        auto cloned_msg = std::make_unique<MsgOdometry>();
        cloned_msg->timestamp = this->timestamp;
        cloned_msg->transform = this->transform; // Eigen matrices support direct assignment/copy
        return cloned_msg;
    }

    void MsgOdometry::serialize(ISerializer& serializer) const {
        // IMPORTANT: Serialize the message type first for deserialization
        serializer.write("message_type", getType());

        serializer.write("timestamp", timestamp);
        serializer.write("transform", transform); // Uses ISerializer's write(const std::string& key, const Eigen::Matrix4d& transform)
    }

    void MsgOdometry::deserialize(IDeserializer& deserializer) {
        // IMPORTANT: Deserialize the message type first
        std::string message_type_read = deserializer.readString("message_type");
        if (message_type_read != getType()) {
            std::cerr << "MsgOdometry: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
            // Handle error, maybe throw exception or set invalid state
        }

        timestamp = deserializer.readDouble("timestamp");
        transform = deserializer.readTransform("transform"); // Uses IDeserializer's readTransform
    }

} // namespace covins
