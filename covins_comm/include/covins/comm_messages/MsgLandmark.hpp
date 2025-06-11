#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <utility>  // For std::pair

// OpenCV includes for descriptor data
#include <opencv2/core/core.hpp>

// Eigen includes for 3D position
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Abstract message interface and serialization interfaces
#include <covins/comm_abstraction/IMessage.hpp> // Ensure .h extension is used
#include <covins/comm_serialization/ISerializer.hpp> // IDeserializer is in the same header, ensure .h

// Covins base types (assuming TypeDefs.h defines things like TypeDefs::Vector3Type)
// Ensure this path is correct based on your 'covins_backend/include' structure
#include <covins/covins_base/typedefs_base.hpp> // Changed to .hpp, assuming your file is named typedefs_base.hpp

namespace covins {

/**
 * @brief Concrete implementation of IMessage for Landmark data.
 *
 * This class encapsulates all necessary information for a 3D landmark in the SLAM system,
 * including its ID, 3D position, an optional descriptor, and metadata like timestamp
 * and whether it's a new landmark or an update. It handles its own serialization
 * and deserialization via the ISerializer/IDeserializer interfaces.
 */
class MsgLandmark : public IMessage {
public:
    // Public data members
    std::pair<int, int> id;         // Landmark ID (first: client_id, second: landmark_local_id)
    double timestamp;               // Timestamp of the landmark (e.g., when it was last updated or created)
    bool is_new;                    // True if this is a newly detected landmark, false if it's an update

    TypeDefs::Vector3Type position; // 3D position of the landmark in world coordinates

    cv::Mat descriptor;             // Optional: descriptor associated with the landmark

public:
    /**
     * @brief Default constructor. Initializes base class with message type.
     */
    MsgLandmark() : IMessage("Landmark") {} // Call base class constructor

    /**
     * @brief Destructor.
     */
    virtual ~MsgLandmark() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "Landmark"
     */
    std::string getType() const override { return message_type_; }

    /**
     * @brief Creates a deep copy of this MsgLandmark object.
     * @return A unique_ptr to the new MsgLandmark instance.
     */
    std::unique_ptr<IMessage> clone() const override;

    /**
     * @brief Serializes the MsgLandmark data using the provided ISerializer.
     * @param serializer The serializer instance to use.
     */
    void serialize(ISerializer& serializer) const override;

    /**
     * @brief Deserializes data into this MsgLandmark object using the provided IDeserializer.
     * @param deserializer The deserializer instance to use.
     */
    void deserialize(IDeserializer& deserializer) override;
};

} // namespace covins
