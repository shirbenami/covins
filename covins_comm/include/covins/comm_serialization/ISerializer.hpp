#pragma once

#include <string>
#include <vector>
#include <cstdint> // For uint8_t
#include <eigen3/Eigen/Dense> // For Eigen::Matrix4d, Eigen::Vector3d, etc.

namespace covins {

/**
 * @brief Abstract interface for serializing data into a byte stream.
 *
 * Concrete implementations of this interface (e.g., ProtobufSerializer, CerealSerializer)
 * will convert various C++ data types into a format suitable for network transmission.
 */
class ISerializer {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~ISerializer() = default;

    /**
     * @brief Writes a string value to the serializer.
     * @param key A key or identifier for the data (useful for structured serialization like JSON/Protobuf).
     * @param value The string to write.
     */
    virtual void write(const std::string& key, const std::string& value) = 0;

    /**
     * @brief Writes an integer value to the serializer.
     * @param key A key or identifier for the data.
     * @param value The integer to write.
     */
    virtual void write(const std::string& key, int value) = 0;

    /**
     * @brief Writes a double-precision floating-point value to the serializer.
     * @param key A key or identifier for the data.
     * @param value The double to write.
     */
    virtual void write(const std::string& key, double value) = 0; // FIX: Removed duplicate 'void'

    /**
     * @brief Writes a boolean value to the serializer.
     * @param key A key or identifier for the data.
     * @param value The boolean to write.
     */
    virtual void write(const std::string& key, bool value) = 0;

    /**
     * @brief Writes a vector of unsigned 8-bit integers (raw binary data) to the serializer.
     * @param key A key or identifier for the data.
     * @param data The vector of bytes to write.
     */
    virtual void write(const std::string& key, const std::vector<uint8_t>& data) = 0;

    /**
     * @brief Writes an Eigen 4x4 transformation matrix to the serializer.
     * @param key A key or identifier for the data.
     * @param transform The Eigen::Matrix4d to write.
     */
    virtual void write(const std::string& key, const Eigen::Matrix4d& transform) = 0;

    /**
     * @brief Writes an Eigen 3D vector to the serializer.
     * @param key A key or identifier for the data.
     * @param vector The Eigen::Vector3d to write.
     */
    virtual void write(const std::string& key, const Eigen::Vector3d& vector) = 0;

    /**
     * @brief Writes an Eigen 9x9 matrix (e.g., for covariance) to the serializer.
     * @param key A key or identifier for the data.
     * @param matrix The Eigen::Matrix<double, 9, 9> to write.
     */
    virtual void write(const std::string& key, const Eigen::Matrix<double, 9, 9>& matrix) = 0;


    /**
     * @brief Retrieves the final serialized data as a vector of bytes.
     *
     * This method is typically called after all data members of a message
     * have been written using the other 'write' methods.
     *
     * @return A vector of uint8_t containing the serialized data.
     */
    virtual std::vector<uint8_t> getSerializedData() const = 0;

    /**
     * @brief Resets the internal state of the serializer for reuse or new serialization.
     * This is useful for efficiency in high-throughput scenarios.
     */
    virtual void reset() = 0;
};

/**
 * @brief Abstract interface for deserializing data from a byte stream.
 *
 * Concrete implementations of this interface will read byte streams
 * and reconstruct C++ data types.
 */
class IDeserializer {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~IDeserializer() = default;

    /**
     * @brief Sets the raw serialized data from which to read.
     * This method must be called before any 'read' operations.
     * @param data The vector of uint8_t containing the serialized data.
     */
    virtual void setData(const std::vector<uint8_t>& data) = 0;

    /**
     * @brief Reads a string value from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read string.
     */
    virtual std::string readString(const std::string& key) = 0;

    /**
     * @brief Reads an integer value from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read integer.
     */
    virtual int readInt(const std::string& key) = 0;

    /**
     * @brief Reads a double-precision floating-point value from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read double.
     */
    virtual double readDouble(const std::string& key) = 0;

    /**
     * @brief Reads a boolean value from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read boolean.
     */
    virtual bool readBool(const std::string& key) = 0;

    /**
     * @brief Reads a vector of unsigned 8-bit integers (raw binary data) from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read vector of bytes.
     */
    virtual std::vector<uint8_t> readBinary(const std::string& key) = 0;

    /**
     * @brief Reads an Eigen 4x4 transformation matrix from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read Eigen::Matrix4d.
     */
    virtual Eigen::Matrix4d readTransform(const std::string& key) = 0;

    /**
     * @brief Reads an Eigen 3D vector from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read Eigen::Vector3d.
     */
    virtual Eigen::Vector3d readVector3d(const std::string& key) = 0;

    /**
     * @brief Reads an Eigen 9x9 matrix from the deserializer.
     * @param key The key or identifier for the data to read.
     * @return The read Eigen::Matrix<double, 9, 9>.
     */
    virtual Eigen::Matrix<double, 9, 9> readMatrix9d(const std::string& key) = 0;
};

} // namespace covins
