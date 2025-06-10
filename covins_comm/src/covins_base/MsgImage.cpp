#include <covins/comm_messages/MsgImage.hpp>
#include <iostream> // For error/debug output
#include <cstring>  // For std::memcpy
#include <sstream> // For std::stringstream

// Cereal headers for serialization
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

// No need to define cv::Mat serialization here if it's already in cereal_types_base.hpp

namespace covins {

MsgImage::MsgImage()
    : timestamp(0.0), image(cv::Mat()) // Initialize with empty Mat
{
}

void MsgImage::setImage(const cv::Mat& img) {
    image = img.clone(); // Deep copy
}

const cv::Mat& MsgImage::getImage() const {
    return image;
}

std::unique_ptr<IMessage> MsgImage::clone() const {
    auto cloned_msg = std::make_unique<MsgImage>();
    cloned_msg->timestamp = this->timestamp;
    cloned_msg->image = this->image.clone(); // Deep copy for cv::Mat
    return cloned_msg;
}

void MsgImage::serialize(ISerializer& serializer) const {
    // IMPORTANT: Serialize the message type first for deserialization
    serializer.write("message_type", getType());

    // Use a temporary archive to serialize the whole object with Cereal's
    // generated serialize method (which uses CEREAL_NVP).
    // Then get the binary data and write it via the ISerializer's write method.
    std::stringstream ss;
    cereal::BinaryOutputArchive archive(ss);
    // Directly serialize members within this Cereal archive.
    // No CEREAL_NVP required here, as we are handling it manually.
    archive(timestamp);
    archive(image); // Uses the custom cv::Mat serialization from cereal_types_base.hpp

    serializer.write("data", std::vector<uint8_t>(ss.str().begin(), ss.str().end()));
}

void MsgImage::deserialize(IDeserializer& deserializer) {
    // IMPORTANT: Deserialize the message type first
    std::string message_type_read = deserializer.readString("message_type");
    if (message_type_read != getType()) {
        std::cerr << "MsgImage: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error
        return;
    }

    std::vector<uint8_t> data = deserializer.readBinary("data");
    std::string s_data(data.begin(), data.end());
    std::stringstream ss(s_data);
    cereal::BinaryInputArchive archive(ss);
    // Deserialize members from this Cereal archive.
    archive(timestamp);
    archive(image); // Uses the custom cv::Mat serialization from cereal_types_base.hpp
}

} // namespace covins
//
// Created by user1 on 10/06/25.
//
