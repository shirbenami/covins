//
// Created by user1 on 09/06/25.
//

#ifndef PREINTEGRATIONDATA_HPP
#define PREINTEGRATIONDATA_HPP
#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr

// Eigen includes for matrices and vectors
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Abstract message interface and serialization interfaces
#include <covins/comm_abstraction/IMessage.hpp>
#include <covins/comm_abstraction/ISerializer.hpp> // IDeserializer is in the same header

namespace covins {

/**
 * @brief Concrete implementation of IMessage for preintegrated IMU data.
 *
 * This message type encapsulates the change in pose, velocity, and biases
 * over a time interval, as typically computed by IMU preintegration.
 * It handles its own serialization and deserialization via the
 * ISerializer/IDeserializer interfaces.
 */
class PreintegrationData : public IMessage {
public:
    // Public data members for preintegrated IMU measurements
    Eigen::Matrix4d delta_pose_;         // Change in pose (rotation and translation)
    Eigen::Vector3d delta_velocity_;     // Change in velocity
    Eigen::Vector3d delta_bias_acc_;     // Change in accelerometer bias
    Eigen::Vector3d delta_bias_gyro_;    // Change in gyroscope bias
    double delta_time_;                  // Total time interval of integration

public:
    /**
     * @brief Default constructor. Initializes all members to identity/zero.
     */
    PreintegrationData();

    /**
     * @brief Destructor.
     */
    virtual ~PreintegrationData() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "PreintegrationData"
     */
    std::string getType() const override { return "PreintegrationData"; }

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

#endif //PREINTEGRATIONDATA_HPP
