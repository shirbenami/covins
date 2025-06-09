#include <covins/comm_serialization/ProtobufSerializer.hpp> // Ensure .hpp extension
#include <iostream> // For error reporting
#include <stdexcept> // For std::runtime_error

// Include the generated Protobuf header for DataBuffer
#include "data_buffer.pb.h" // Assuming this file is generated and available

namespace covins {

// Helper to ensure data_buffer_pb_ is initialized
void ProtobufSerializer::ensureDataBufferInitialized() {
    if (!data_buffer_pb_) {
        data_buffer_pb_ = std::make_unique<covins::protobuf::DataBuffer>();
    }
}

// Helper to populate deserialized_entries_ map after setData
void ProtobufSerializer::populateDeserializedEntries() {
    deserialized_entries_.clear();
    if (data_buffer_pb_) {
        for (const auto& entry : data_buffer_pb_->entries()) {
            deserialized_entries_[entry.key()] = &entry;
        }
    }
}


ProtobufSerializer::ProtobufSerializer() {
    ensureDataBufferInitialized();
}

ProtobufSerializer::~ProtobufSerializer() {
    // Unique_ptr handles deletion of data_buffer_pb_
}

// --- ISerializer interface implementation ---

void ProtobufSerializer::write(const std::string& key, const std::string& value) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    entry->mutable_value()->set_string_val(value);
}

void ProtobufSerializer::write(const std::string& key, int value) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    entry->mutable_value()->set_int_val(value);
}

void ProtobufSerializer::write(const std::string& key, double value) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    entry->mutable_value()->set_double_val(value);
}

void ProtobufSerializer::write(const std::string& key, bool value) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    entry->mutable_value()->set_bool_val(value);
}

void ProtobufSerializer::write(const std::string& key, const std::vector<uint8_t>& data) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    entry->mutable_value()->set_binary_val(data.data(), data.size());
}

void ProtobufSerializer::write(const std::string& key, const Eigen::Matrix4d& transform) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    auto* matrix_val = entry->mutable_value()->mutable_matrix4d_val();
    matrix_val->Reserve(16); // Reserve space for 4x4 matrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix_val->Add(transform(i, j));
        }
    }
}

void ProtobufSerializer::write(const std::string& key, const Eigen::Vector3d& vector) {
    ensureDataBufferInitialized();
    auto* entry = data_buffer_pb_->add_entries();
    entry->set_key(key);
    auto* vector_val = entry->mutable_value()->mutable_vector3d_val();
    vector_val->Reserve(3); // Reserve space for 3D vector
    for (int i = 0; i < 3; ++i) {
        vector_val->Add(vector(i));
    }
}

std::vector<uint8_t> ProtobufSerializer::getSerializedData() const {
    if (!data_buffer_pb_) {
        std::cerr << "ERROR: ProtobufSerializer: Data buffer not initialized for serialization." << std::endl;
        return {};
    }
    std::string serialized_string;
    if (!data_buffer_pb_->SerializeToString(&serialized_string)) {
        std::cerr << "ERROR: ProtobufSerializer: Failed to serialize DataBuffer to string." << std::endl;
        return {};
    }
    return std::vector<uint8_t>(serialized_string.begin(), serialized_string.end());
}

void ProtobufSerializer::reset() {
    data_buffer_pb_->Clear(); // Clear the Protobuf message
    deserialized_entries_.clear(); // Clear the lookup map
}

// --- IDeserializer interface implementation ---

void ProtobufSerializer::setData(const std::vector<uint8_t>& data) {
    ensureDataBufferInitialized();
    std::string serialized_string(reinterpret_cast<const char*>(data.data()), data.size());
    if (!data_buffer_pb_->ParseFromString(serialized_string)) {
        std::cerr << "ERROR: ProtobufSerializer: Failed to parse DataBuffer from string." << std::endl;
        data_buffer_pb_->Clear(); // Clear any partially parsed data
    }
    populateDeserializedEntries(); // Populate the lookup map after parsing
}

const covins::protobuf::DataBuffer_Entry* getEntry(const std::string& key,
                                                     const std::map<std::string, const covins::protobuf::DataBuffer_Entry*>& entries) {
    auto it = entries.find(key);
    if (it == entries.end()) {
        std::cerr << "ERROR: ProtobufSerializer: Key '" << key << "' not found during deserialization." << std::endl;
        // Optionally throw an exception here for unrecoverable errors
        // throw std::runtime_error("Key not found: " + key);
        return nullptr;
    }
    return it->second;
}

std::string ProtobufSerializer::readString(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    // In proto3, for oneof fields, check which field is set using oneof_case()
    if (entry && entry->value().type_value_case() == covins::protobuf::Value::kStringVal) {
        return entry->value().string_val();
    }
    std::cerr << "ERROR: ProtobufSerializer: Expected string value for key '" << key << "' but found different type or no entry." << std::endl;
    return "";
}

int ProtobufSerializer::readInt(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    if (entry && entry->value().type_value_case() == covins::protobuf::Value::kIntVal) {
        return entry->value().int_val();
    }
    std::cerr << "ERROR: ProtobufSerializer: Expected int value for key '" << key << "' but found different type or no entry." << std::endl;
    return 0;
}

double ProtobufSerializer::readDouble(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    if (entry && entry->value().type_value_case() == covins::protobuf::Value::kDoubleVal) {
        return entry->value().double_val();
    }
    std::cerr << "ERROR: ProtobufSerializer: Expected double value for key '" << key << "' but found different type or no entry." << std::endl;
    return 0.0;
}

bool ProtobufSerializer::readBool(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    if (entry && entry->value().type_value_case() == covins::protobuf::Value::kBoolVal) {
        return entry->value().bool_val();
    }
    std::cerr << "ERROR: ProtobufSerializer: Expected bool value for key '" << key << "' but found different type or no entry." << std::endl;
    return false;
}

std::vector<uint8_t> ProtobufSerializer::readBinary(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    if (entry && entry->value().type_value_case() == covins::protobuf::Value::kBinaryVal) {
        const std::string& binary_str = entry->value().binary_val();
        return std::vector<uint8_t>(binary_str.begin(), binary_str.end());
    }
    std::cerr << "ERROR: ProtobufSerializer: Expected binary value for key '" << key << "' but found different type or no entry." << std::endl;
    return {};
}

Eigen::Matrix4d ProtobufSerializer::readTransform(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    // Removed type_value_case check, directly check size for repeated field
    if (entry && entry->value().matrix4d_val_size() == 16) {
        int k = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform(i, j) = entry->value().matrix4d_val(k++);
            }
        }
    } else {
        // If entry is null, or size is not 16, it's an error for a Matrix4d
        std::cerr << "ERROR: ProtobufSerializer: Expected matrix4d value for key '" << key << "' but found different size or no entry." << std::endl;
    }
    return transform;
}

Eigen::Vector3d ProtobufSerializer::readVector3d(const std::string& key) {
    const auto* entry = getEntry(key, deserialized_entries_);
    Eigen::Vector3d vector = Eigen::Vector3d::Zero();
    // Removed type_value_case check, directly check size for repeated field
    if (entry && entry->value().vector3d_val_size() == 3) {
        for (int i = 0; i < 3; ++i) {
            vector(i) = entry->value().vector3d_val(i);
        }
    } else {
        // If entry is null, or size is not 3, it's an error for a Vector3d
        std::cerr << "ERROR: ProtobufSerializer: Expected vector3d value for key '" << key << "' but found different size or no entry." << std::endl;
    }
    return vector;
}

} // namespace covins
