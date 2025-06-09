//
// Created by user1 on 09/06/25.
//
#include <covins/comm_messages/PreintegrationData.hpp>
#include <iostream> // For error/debug output

namespace covins {

PreintegrationData::PreintegrationData()
    : delta_pose_(Eigen::Matrix4d::Identity()),
      delta_velocity_(Eigen::Vector3d::Zero()),
      delta_bias_acc_(Eigen::Vector3d::Zero()),
      delta_bias_gyro_(Eigen::Vector3d::Zero()),
      delta_time_(0.0)
{
    // All members are initialized to identity/zero in the initializer list.
}

std::unique_ptr<IMessage> PreintegrationData::clone() const {
    auto cloned_msg = std::make_unique<PreintegrationData>();
    cloned_msg->delta_pose_ = this->delta_pose_;
    cloned_msg->delta_velocity_ = this->delta_velocity_;
    cloned_msg->delta_bias_acc_ = this->delta_bias_acc_;
    cloned_msg->delta_bias_gyro_ = this->delta_bias_gyro_;
    cloned_msg->delta_time_ = this->delta_time_;
    return cloned_msg;
}

void PreintegrationData::serialize(ISerializer& serializer) const {
    // IMPORTANT: Serialize the message type first for deserialization
    serializer.write("message_type", getType());

    serializer.write("delta_pose", delta_pose_);
    serializer.write("delta_velocity", delta_velocity_);
    serializer.write("delta_bias_acc", delta_bias_acc_);
    serializer.write("delta_bias_gyro", delta_bias_gyro_);
    serializer.write("delta_time", delta_time_);
}

void PreintegrationData::deserialize(IDeserializer& deserializer) {
    // IMPORTANT: Deserialize the message type first
    std::string message_type_read = deserializer.readString("message_type");
    if (message_type_read != getType()) {
        std::cerr << "PreintegrationData: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error, maybe throw exception or set invalid state
    }

    delta_pose_ = deserializer.readTransform("delta_pose");
    delta_velocity_ = deserializer.readVector3d("delta_velocity");
    delta_bias_acc_ = deserializer.readVector3d("delta_bias_acc");
    delta_bias_gyro_ = deserializer.readVector3d("delta_bias_gyro");
    delta_time_ = deserializer.readDouble("delta_time");
}

} // namespace covins
