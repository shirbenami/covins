#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr

// Eigen includes for transformation matrix
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Abstract message interface and serialization interfaces
#include <covins/comm_abstraction/IMessage.hpp>
#include <covins/comm_serialization/ISerializer.hpp> // IDeserializer is in the same header

namespace covins {

/**
 * @brief Concrete implementation of IMessage for Odometry data.
 *
 * This class encapsulates a 4x4 pose transformation matrix and its associated timestamp.
 * It handles its own serialization and deserialization via the
 * ISerializer/IDeserializer interfaces.
 */
class MsgOdometry : public IMessage {
public:
    // Public data members
    double timestamp;             // Timestamp of the odometry
    Eigen::Matrix4d transform;    // 4x4 pose transformation matrix (e.g., T_world_robot)

public:
    /**
     * @brief Default constructor. Initializes base class with message type.
     */
    MsgOdometry() : IMessage("Odometry") {} // Call base class constructor

    /**
     * @brief Destructor.
     */
    virtual ~MsgOdometry() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "Odometry"
     */
    std::string getType() const override { return message_type_; }

    /**
     * @brief Creates a deep copy of this MsgOdometry object.
     * @return A unique_ptr to the new MsgOdometry instance.
     */
    std::unique_ptr<IMessage> clone() const override;

    /**
     * @brief Serializes the MsgOdometry data using the provided ISerializer.
     * @param serializer The serializer instance to use.
     */
    void serialize(ISerializer& serializer) const override;

    /**
     * @brief Deserializes data into this MsgOdometry object using the provided IDeserializer.
     * @param deserializer The deserializer instance to use.
     */
    void deserialize(IDeserializer& deserializer) override;

    // Utility methods
    void setTransform(const Eigen::Matrix4d& t); // Ensure declared if defined in .cpp
    const Eigen::Matrix4d& getTransform() const; // Ensure declared if defined in .cpp
    void setTimestamp(double ts) { timestamp = ts; }
    double getTimestamp() const { return timestamp; }
};

} // namespace covins
