//
// Created by user1 on 08/06/25.
//

#ifndef CEREALSERIALIZER_HPP
#define CEREALSERIALIZER_HPP
#pragma once

#include <string>
#include <vector>
#include <cstdint> // For uint8_t
#include <memory>  // For std::unique_ptr
#include <sstream> // For std::stringstream

// Eigen includes for matrix/vector types
#include <eigen3/Eigen/Dense>

// Cereal library includes
// We'll use binary archives for efficiency
#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>      // Potentially useful if keys are used directly by Cereal
#include <cereal/types/complex.hpp>  // For complex numbers, if needed (not directly used by ISerializer here)

// Custom Cereal serialization for Eigen types (declaration)
// This is typically done in a separate header or directly in the CerealSerializer.cpp
// or a common utility header. For simplicity, we'll declare it here and define in .cpp
// or serialize element-wise if preferred.
// For robust Eigen serialization, consider a dedicated header like:
// #include <cereal/types/eigen.hpp> // if using a cereal-eigen extension
// For now, we'll serialize element-wise within the write/read methods.

// Include the abstract interfaces
#include <covins/comm_abstraction/ISerializer.hpp> // This also defines IDeserializer

namespace covins {

/**
 * @brief Concrete implementation of ISerializer and IDeserializer using the Cereal library.
 *
 * This class provides methods to serialize C++ data types into a binary stream
 * and deserialize them back, fulfilling the contracts defined by ISerializer and IDeserializer.
 * It uses Cereal's binary archive for efficient data packing.
 */
class CerealSerializer : public ISerializer, public IDeserializer {
public:
    /**
     * @brief Constructor. Initializes the internal stringstream and Cereal archives.
     */
    CerealSerializer();

    /**
     * @brief Destructor. Cleans up archive objects.
     */
    virtual ~CerealSerializer();

    // --- ISerializer interface implementation ---

    void write(const std::string& key, const std::string& value) override;
    void write(const std::string& key, int value) override;
    void write(const std::string& key, double value) override;
    void write(const std::string& key, bool value) override;
    void write(const std::string& key, const std::vector<uint8_t>& data) override;
    void write(const std::string& key, const Eigen::Matrix4d& transform) override;
    void write(const std::string& key, const Eigen::Vector3d& vector) override;

    std::vector<uint8_t> getSerializedData() const override;
    void reset() override;

    // --- IDeserializer interface implementation ---

    void setData(const std::vector<uint8_t>& data) override;
    std::string readString(const std::string& key) override;
    int readInt(const std::string& key) override;
    double readDouble(const std::string& key) override;
    bool readBool(const std::string& key) override;
    std::vector<uint8_t> readBinary(const std::string& key) override;
    Eigen::Matrix4d readTransform(const std::string& key) override;
    Eigen::Vector3d readVector3d(const std::string& key) override;

private:
    // Internal stringstream to hold the binary data
    std::stringstream os_; // For output (serialization)
    std::stringstream is_; // For input (deserialization)

    // Cereal archives
    // Use unique_ptr to manage their lifetime, as they are not default constructible
    // and need to be recreated when streams are reset.
    std::unique_ptr<cereal::BinaryOutputArchive> oarchive_;
    std::unique_ptr<cereal::BinaryInputArchive> iarchive_;

    // To handle the "keys" from ISerializer/IDeserializer. Cereal doesn't
    // inherently use keys for binary archives in the same way JSON/XML do.
    // For direct serialization, we just serialize values sequentially.
    // If keys are crucial for distinguishing data within a stream,
    // a map<string, T> could be used, or a custom approach with Cereal.
    // For this implementation, we'll assume sequential serialization for simplicity
    // and rely on the consuming IMessage to know the order of data.
    // However, if the key is essential for context, we can serialize the key along with the value.
    // Let's adapt the implementation to serialize the key along with the value,
    // which makes the binary stream more robust but slightly larger.
    // For this, we'll use a map internally for writes, and read by key.
    // This is more complex but matches the ISerializer/IDeserializer interface better.
    // Simpler approach for binary: just serialize values in a known order.
    // Given the ISerializer interface requires a 'key', let's stick to using a map structure for now.
    // A map within the stream might not be the most efficient for pure binary,
    // but it respects the API.

    // A map to hold serialized data by key before flushing to stream, or for reads
    // This approach might be less performant for large binary blobs.
    // A more idiomatic Cereal approach would be to define custom save/load functions
    // for each IMessage, passing the archive directly, rather than through
    // generic write/read methods with keys.
    // For now, let's process the 'key' argument in the actual implementation for demonstration.
    // A direct binary archive doesn't map keys to values implicitly.
    // So, we'll write/read values sequentially, and the 'key' argument in ISerializer/IDeserializer
    // will serve as a hint for the user of CerealSerializer on *what* they are writing/reading.
    // This is a common simplification when bridging a key-value interface to a sequential binary one.
};

} // namespace covins

#endif //CEREALSERIALIZER_HPP
