#pragma once

#include <string>
#include <memory> // For std::unique_ptr

// Forward declarations for ISerializer and IDeserializer to avoid circular dependencies.
namespace covins {
    class ISerializer;
    class IDeserializer;
}

namespace covins {

/**
 * @brief Abstract interface for all data types exchanged within the system.
 *
 * This interface serves as the polymorphic base for various message types
 * (e.g., Keyframe, Landmark, Image, Odometry). It ensures that any message
 * can be identified by its type, cloned polymorphically, and handle its
 * own serialization and deserialization processes via the ISerializer/IDeserializer interfaces.
 */
class IMessage {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~IMessage() = default;

    /**
     * @brief Returns a string identifier for the message type.
     *
     * This method is a pure virtual function, forcing derived classes to implement it.
     * Derived classes will typically return the 'message_type_' member.
     *
     * @return A string representing the type of the message (e.g., "Keyframe", "Image", "Odometry").
     */
    virtual std::string getType() const = 0; // Made pure virtual

    /**
     * @brief Creates a polymorphic copy of the current message object.
     * @return A unique_ptr to a newly created, deep copy of the message.
     */
    virtual std::unique_ptr<IMessage> clone() const = 0;

    /**
     * @brief Serializes the message's data using the provided ISerializer.
     * @param serializer The ISerializer instance to use for writing data.
     */
    virtual void serialize(ISerializer& serializer) const = 0;

    /**
     * @brief Deserializes data into the message's members using the provided IDeserializer.
     * @param deserializer The IDeserializer instance to use for reading data.
     */
    virtual void deserialize(IDeserializer& deserializer) = 0;

protected:
    /**
     * @brief Protected constructor to initialize the message type for derived classes.
     * @param type The string identifier for the message type.
     */
    IMessage(const std::string& type) : message_type_(type) {}

    std::string message_type_; // Stores the message type, accessible by derived classes
};

} // namespace covins
