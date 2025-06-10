#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr

// OpenCV includes for image data
#include <opencv2/core/core.hpp>

// Abstract message interface and serialization interfaces
#include <covins/comm_abstraction/IMessage.hpp>
#include <covins/comm_serialization/ISerializer.hpp> // IDeserializer is in the same header

namespace covins {

/**
 * @brief Concrete implementation of IMessage for Image data.
 *
 * This class encapsulates image data and its associated timestamp.
 * It handles its own serialization and deserialization via the
 * ISerializer/IDeserializer interfaces.
 */
class MsgImage : public IMessage {
public:
    // Public data members
    double timestamp;   // Timestamp of the image
    cv::Mat image;      // The image data

public:
    /**
     * @brief Default constructor. Initializes base class with message type.
     */
    MsgImage() : IMessage("Image") {}

    /**
     * @brief Destructor.
     */
    virtual ~MsgImage() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "Image"
     */
    std::string getType() const override { return message_type_; } // Now uses base class implementation

    /**
     * @brief Creates a deep copy of this MsgImage object.
     * @return A unique_ptr to the new MsgImage instance.
     */
    std::unique_ptr<IMessage> clone() const override;

    /**
     * @brief Serializes the MsgImage data using the provided ISerializer.
     * @param serializer The serializer instance to use.
     */
    void serialize(ISerializer& serializer) const override;

    /**
     * @brief Deserializes data into this MsgImage object using the provided IDeserializer.
     * @param deserializer The deserializer instance to use.
     */
    void deserialize(IDeserializer& deserializer) override;

    // Utility methods
    void setImage(const cv::Mat& img);
    const cv::Mat& getImage() const;
    void setTimestamp(double ts) { timestamp = ts; }
    double getTimestamp() const { return timestamp; }
};

} // namespace covins
