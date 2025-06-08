//
// Created by user1 on 08/06/25.
//

#ifndef IMESSAGE_HPP
#define IMESSAGE_HPP

#pragma once

#include <string>
#include <memory> // For std::unique_ptr

// Forward declarations for ISerializer and IDeserializer to avoid circular dependencies.
// The IMessage interface interacts with these, but doesn't need their full definitions here.
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
     * This method allows the receiver (e.g., FrontendWrapper's genericMessageCallback)
     * to determine the concrete type of the received message for proper casting and processing.
     *
     * @return A string representing the type of the message (e.g., "Keyframe", "Image", "Odometry").
     */
    virtual std::string getType() const = 0;

    /**
     * @brief Creates a polymorphic copy of the current message object.
     *
     * This method is crucial for handling unique_ptr<IMessage> as it allows
     * for creating deep copies of derived message types without knowing their
     * concrete type at compile time.
     *
     * @return A unique_ptr to a newly created, deep copy of the message.
     */
    virtual std::unique_ptr<IMessage> clone() const = 0;

    /**
     * @brief Serializes the message's data using the provided ISerializer.
     *
     * Concrete message implementations will use the ISerializer's write methods
     * to convert their internal data members into a format suitable for transmission.
     *
     * @param serializer The ISerializer instance to use for writing data.
     */
    virtual void serialize(ISerializer& serializer) const = 0;

    /**
     * @brief Deserializes data into the message's members using the provided IDeserializer.
     *
     * Concrete message implementations will use the IDeserializer's read methods
     * to reconstruct their internal data members from a received byte stream.
     *
     * @param deserializer The IDeserializer instance to use for reading data.
     */
    virtual void deserialize(IDeserializer& deserializer) = 0;
};

} // namespace covins

#endif //IMESSAGE_HPP
