#include <covins/comm_messages/PreintegrationData.hpp> // Ensure .hpp extension
#include <iostream> // For error/debug output

namespace covins {

// Constructor definition removed as it's defined in the header.
// PreintegrationData::PreintegrationData()
//     : delta_time_(0.0),
//       delta_vel_(TypeDefs::Vector3Type::Zero()),
//       delta_ang_(TypeDefs::Vector3Type::Zero()),
//       covariance_(TypeDefs::Matrix9Type::Identity()), // Assuming 9x9 matrix for covariance
//       gyr_bias_(TypeDefs::Vector3Type::Zero()),
//       acc_bias_(TypeDefs::Vector3Type::Zero())
// {
// }


std::unique_ptr<IMessage> PreintegrationData::clone() const {
    auto cloned_msg = std::make_unique<PreintegrationData>();
    cloned_msg->delta_time_ = this->delta_time_;
    cloned_msg->delta_vel_ = this->delta_vel_;
    cloned_msg->delta_ang_ = this->delta_ang_;
    cloned_msg->covariance_ = this->covariance_;
    cloned_msg->gyr_bias_ = this->gyr_bias_;
    cloned_msg->acc_bias_ = this->acc_bias_;
    return cloned_msg;
}

void PreintegrationData::serialize(ISerializer& serializer) const {
    serializer.write("message_type", getType());
    serializer.write("delta_time", delta_time_);
    serializer.write("delta_vel", delta_vel_);
    serializer.write("delta_ang", delta_ang_);
    serializer.write("covariance", covariance_);
    serializer.write("gyr_bias", gyr_bias_);
    serializer.write("acc_bias", acc_bias_);
}

void PreintegrationData::deserialize(IDeserializer& deserializer) {
    std::string message_type_read = deserializer.readString("message_type");
    if (message_type_read != getType()) {
        std::cerr << "PreintegrationData: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error
    }
    delta_time_ = deserializer.readDouble("delta_time");
    delta_vel_ = deserializer.readVector3d("delta_vel");
    delta_ang_ = deserializer.readVector3d("delta_ang");
    covariance_ = deserializer.readMatrix9d("covariance"); // Assuming IDeserializer has readMatrix9d
    gyr_bias_ = deserializer.readVector3d("gyr_bias");
    acc_bias_ = deserializer.readVector3d("acc_bias");
}

} // namespace covins
