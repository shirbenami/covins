//
// Created by user1 on 08/06/25.
//
#include <covins/comm_serialization/CerealSerializer.hpp>
#include <iostream> // For error reporting

// Cereal extensions for Eigen (if available and preferred)
// If you have a cereal-eigen library, uncomment this:
// #include <cereal/archives/portable_binary.hpp> // Often used with cereal-eigen
// #include <cereal/types/eigen.hpp> // Assumes you have this library installed

namespace covins {

// Helper function to serialize Eigen Matrix/Vector element-wise
template<class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void save(Archive & archive, const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
{
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            archive(m(i, j));
        }
    }
}

// Helper function to deserialize Eigen Matrix/Vector element-wise
template<class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void load(Archive & archive, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
{
    m.resize(Rows, Cols); // Ensure correct size for dynamic matrices, though fixed size here
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            archive(m(i, j));
        }
    }
}


CerealSerializer::CerealSerializer()
    : os_(), is_() {
    // Initialize output archive. Input archive will be initialized in setData.
    oarchive_ = std::make_unique<cereal::BinaryOutputArchive>(os_);
}

CerealSerializer::~CerealSerializer() {
    // Archives are managed by unique_ptr, so they will be correctly destroyed.
}

// --- ISerializer interface implementation ---

// Note on 'key': For a pure binary Cereal archive, the 'key' parameter
// is often ignored, and data is serialized sequentially. The IMessage
// implementation is responsible for knowing the order in which to write/read data.
// If actual key-value mapping is needed in binary, a more complex structure (e.g.,
// std::map<std::string, T>) would need to be serialized, which might impact performance
// for large datasets. For simplicity and typical binary efficiency, we proceed sequentially.

void CerealSerializer::write(const std::string& key, const std::string& value) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    (*oarchive_)(value);
}

void CerealSerializer::write(const std::string& key, int value) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    (*oarchive_)(value);
}

void CerealSerializer::write(const std::string& key, double value) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    (*oarchive_)(value);
}

void CerealSerializer::write(const std::string& key, bool value) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    (*oarchive_)(value);
}

void CerealSerializer::write(const std::string& key, const std::vector<uint8_t>& data) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    (*oarchive_)(data);
}

void CerealSerializer::write(const std::string& key, const Eigen::Matrix4d& transform) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    // Serialize Eigen Matrix element-wise
    save(*oarchive_, transform);
}

void CerealSerializer::write(const std::string& key, const Eigen::Vector3d& vector) {
    if (!oarchive_) { std::cerr << "CerealSerializer: Output archive not initialized." << std::endl; return; }
    // Serialize Eigen Vector element-wise
    save(*oarchive_, vector);
}

std::vector<uint8_t> CerealSerializer::getSerializedData() const {
    // Access the underlying stringstream's buffer
    const std::string& str = os_.str();
    return std::vector<uint8_t>(str.begin(), str.end());
}

void CerealSerializer::reset() {
    os_.str("");        // Clear the stringstream content
    os_.clear();        // Clear any error flags
    oarchive_.reset();  // Destroy and recreate output archive
    oarchive_ = std::make_unique<cereal::BinaryOutputArchive>(os_);

    is_.str("");        // Clear the input stringstream content
    is_.clear();        // Clear any error flags
    iarchive_.reset();  // Destroy input archive
}

// --- IDeserializer interface implementation ---

void CerealSerializer::setData(const std::vector<uint8_t>& data) {
    is_.str(""); // Clear previous data
    is_.clear(); // Clear any error flags
    is_.write(reinterpret_cast<const char*>(data.data()), data.size()); // Write new data

    iarchive_.reset(); // Destroy and recreate input archive
    iarchive_ = std::make_unique<cereal::BinaryInputArchive>(is_);
}

// Note on 'key': Similar to write, for binary Cereal, keys are usually ignored
// and data is deserialized sequentially. The IMessage implementation must
// read in the same order it wrote.

std::string CerealSerializer::readString(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return ""; }
    std::string value;
    (*iarchive_)(value);
    return value;
}

int CerealSerializer::readInt(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return 0; }
    int value;
    (*iarchive_)(value);
    return value;
}

double CerealSerializer::readDouble(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return 0.0; }
    double value;
    (*iarchive_)(value);
    return value;
}

bool CerealSerializer::readBool(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return false; }
    bool value;
    (*iarchive_)(value);
    return value;
}

std::vector<uint8_t> CerealSerializer::readBinary(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return {}; }
    std::vector<uint8_t> data;
    (*iarchive_)(data);
    return data;
}

Eigen::Matrix4d CerealSerializer::readTransform(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return Eigen::Matrix4d::Identity(); }
    Eigen::Matrix4d transform;
    load(*iarchive_, transform);
    return transform;
}

Eigen::Vector3d CerealSerializer::readVector3d(const std::string& key) {
    if (!iarchive_) { std::cerr << "CerealSerializer: Input archive not initialized." << std::endl; return Eigen::Vector3d::Zero(); }
    Eigen::Vector3d vector;
    load(*iarchive_, vector);
    return vector;
}

} // namespace covins
