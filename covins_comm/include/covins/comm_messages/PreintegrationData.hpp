#pragma once

#include <string>
#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <covins/comm_abstraction/IMessage.hpp>
#include <covins/comm_serialization/ISerializer.hpp> // For ISerializer and IDeserializer
#include <covins/covins_base/typedefs_base.hpp> // For TypeDefs

namespace covins {

/**
 * @brief Concrete implementation of IMessage for PreintegrationData.
 * This message typically carries preintegrated IMU measurements.
 */
class PreintegrationData : public IMessage {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Public data members related to IMU preintegration
    double delta_time_;
    TypeDefs::Vector3Type delta_vel_;
    TypeDefs::Vector3Type delta_ang_;
    TypeDefs::Matrix9Type covariance_; // Covariance matrix for preintegration
    TypeDefs::Vector3Type gyr_bias_;   // Gyroscope bias
    TypeDefs::Vector3Type acc_bias_;   // Accelerometer bias

public:
    /**
     * @brief Default constructor. Initializes base class with message type.
     * !!! CRITICAL FIX: Call base class constructor with string argument !!!
     */
    PreintegrationData() : IMessage("PreintegrationData")
    {
        delta_time_ = 0.0;
        delta_vel_.setZero();
        delta_ang_.setZero();
        covariance_.setIdentity(); // Initialize to identity or zeros as appropriate
        gyr_bias_.setZero();
        acc_bias_.setZero();
    }

    /**
     * @brief Destructor.
     */
    virtual ~PreintegrationData() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "PreintegrationData"
     */
    std::string getType() const override { return message_type_; }

    /**
     * @brief Creates a deep copy of this PreintegrationData object.
     * @return A unique_ptr to the new PreintegrationData instance.
     */
    std::unique_ptr<IMessage> clone() const override;

    /**
     * @brief Serializes the PreintegrationData using the provided ISerializer.
     * @param serializer The serializer instance to use.
     */
    void serialize(ISerializer& serializer) const override;

    /**
     * @brief Deserializes data into this PreintegrationData object using the provided IDeserializer.
     * @param deserializer The deserializer instance to use.
     */
    void deserialize(IDeserializer& deserializer) override;
};

} // namespace covins
