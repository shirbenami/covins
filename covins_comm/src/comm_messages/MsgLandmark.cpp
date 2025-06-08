#include <covins/comm_messages/MsgLandmark.hpp>
#include <iostream> // For error/debug output

namespace covins {

MsgLandmark::MsgLandmark()
    : id({-1, -1}), timestamp(0.0), is_new(true),
      position(TypeDefs::Vector3Type::Zero())
{
    descriptor = cv::Mat(); // Default initialize empty cv::Mat
}

std::unique_ptr<IMessage> MsgLandmark::clone() const {
    auto cloned_msg = std::make_unique<MsgLandmark>();

    cloned_msg->id = this->id;
    cloned_msg->timestamp = this->timestamp;
    cloned_msg->is_new = this->is_new;
    cloned_msg->position = this->position;
    cloned_msg->descriptor = this->descriptor.clone(); // Deep copy for cv::Mat

    return cloned_msg;
}

void MsgLandmark::serialize(ISerializer& serializer) const {
    // IMPORTANT: Serialize the message type first for deserialization
    serializer.write("message_type", getType());

    // Basic Landmark info
    serializer.write("landmark_id_first", id.first);
    serializer.write("landmark_id_second", id.second);
    serializer.write("timestamp", timestamp);
    serializer.write("is_new", is_new);

    // 3D position
    serializer.write("position", position);

    // Optional descriptor
    if (!descriptor.empty()) {
        serializer.write("descriptor_rows", descriptor.rows);
        serializer.write("descriptor_cols", descriptor.cols);
        serializer.write("descriptor_type", descriptor.type()); // e.g., CV_8U, CV_32F

        size_t data_size = descriptor.total() * descriptor.elemSize();
        std::vector<uint8_t> data_vec(data_size);
        std::memcpy(data_vec.data(), descriptor.data, data_size);
        serializer.write("descriptor_data", data_vec);
    } else {
        serializer.write("descriptor_rows", 0); // Indicate empty descriptor
        serializer.write("descriptor_cols", 0);
        serializer.write("descriptor_type", 0);
        serializer.write("descriptor_data", std::vector<uint8_t>{});
    }
}

void MsgLandmark::deserialize(IDeserializer& deserializer) {
    // IMPORTANT: Deserialize the message type first
    std::string message_type_read = deserializer.readString("message_type");
    // You might want to assert or check if message_type_read matches "Landmark" here
    if (message_type_read != getType()) {
        std::cerr << "MsgLandmark: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error
    }

    // Basic Landmark info
    id.first = deserializer.readInt("landmark_id_first");
    id.second = deserializer.readInt("landmark_id_second");
    timestamp = deserializer.readDouble("timestamp");
    is_new = deserializer.readBool("is_new");

    // 3D position
    position = deserializer.readVector3d("position");

    // Optional descriptor
    int desc_rows = deserializer.readInt("descriptor_rows");
    int desc_cols = deserializer.readInt("descriptor_cols");
    int desc_type = deserializer.readInt("descriptor_type");
    std::vector<uint8_t> descriptor_data = deserializer.readBinary("descriptor_data");

    if (desc_rows > 0 && desc_cols > 0 && !descriptor_data.empty()) {
        // Reconstruct cv::Mat from raw data
        descriptor = cv::Mat(desc_rows, desc_cols, desc_type, descriptor_data.data()).clone();
    } else {
        descriptor = cv::Mat(); // Empty matrix
    }
}

} // namespace covins
