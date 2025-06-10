//
// Created by user1 on 10/06/25.
//

#ifndef CEREAL_TYPES_BASE_HPP
#define CEREAL_TYPES_BASE_HPP
#pragma once

// Standard Cereal type includes
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp> // For std::pair

// Eigen includes
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Cereal archive type (must match what's used in Serializer.cpp)
#include <cereal/archives/binary.hpp>

// Custom Cereal serialization for Eigen types (element-wise)
// This is necessary if you don't use the dedicated <cereal/types/eigen.hpp> extension.
// It will serialize/deserialize Eigen matrices and vectors element by element.
namespace cereal {

    // Helper for fixed-size Eigen matrices/vectors
    template<class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    void save(Archive & archive, const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
    {
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                archive(m(i, j));
            }
        }
    }

    template<class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    void load(Archive & archive, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
    {
        // For fixed-size, it's implicitly correct. For dynamic, resize before loading.
        // Assuming the matrix/vector is already correctly sized or will be resized based on context.
        // For fixed-size matrices, `m.resize(Rows, Cols);` is not strictly needed but harmless.
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                archive(m(i, j));
            }
        }
    }

    // Specialization for Eigen::DynamicVectorType (Eigen::Matrix<double, Eigen::Dynamic, 1>)
    template<class Archive, typename Scalar, int Options, int MaxRows, int MaxCols>
    void save(Archive & archive, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options, MaxRows, MaxCols>& v)
    {
        archive(v.rows()); // Save number of rows for dynamic vector
        for (int i = 0; i < v.rows(); ++i) {
            archive(v(i));
        }
    }

    template<class Archive, typename Scalar, int Options, int MaxRows, int MaxCols>
    void load(Archive & archive, Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options, MaxRows, MaxCols>& v)
    {
        int rows;
        archive(rows); // Load number of rows
        v.resize(rows);
        for (int i = 0; i < rows; ++i) {
            archive(v(i));
        }
    }

    // Specialization for `Eigen::Matrix<float, 2, 1>` (TypeDefs::KeypointType)
    template<class Archive>
    void serialize(Archive & archive, Eigen::Matrix<float, 2, 1>& kp)
    {
        archive(kp(0), kp(1));
    }

    // Specialization for `Eigen::Matrix<float, 4, 1>` (TypeDefs::AorsType)
    template<class Archive>
    void serialize(Archive & archive, Eigen::Matrix<float, 4, 1>& aors)
    {
        archive(aors(0), aors(1), aors(2), aors(3));
    }

    // Specialization for `std::pair<long unsigned int, size_t>` (TypeDefs::IdType)
    template<class Archive>
    void serialize(Archive & archive, std::pair<long unsigned int, size_t>& id_pair)
    {
        archive(id_pair.first, id_pair.second);
    }

    // Specialization for `std::vector<uint32_t>` (TypeDefs::MsgTypeVector)
    template<class Archive>
    void serialize(Archive & archive, std::vector<uint32_t>& vec)
    {
        archive(vec); // Cereal has built-in support for std::vector
    }

    // Specialization for enum types (eCamModel, eDistortionModel)
    template<class Archive>
    void serialize(Archive & archive, covins::eCamModel& model) {
        archive(reinterpret_cast<int&>(model));
    }
    template<class Archive>
    void serialize(Archive & archive, covins::eDistortionModel& model) {
        archive(reinterpret_cast<int&>(model));
    }

    // Custom serialization for cv::Mat
    template<class Archive>
    void save(Archive & archive, const cv::Mat& m)
    {
        if (m.empty()) {
            archive(0, 0, 0); // rows, cols, type to indicate empty
            return;
        }
        archive(m.rows, m.cols, m.type());
        const int data_size = m.rows * m.cols * m.elemSize();
        archive(cereal::binary_data(m.data, data_size));
    }

    template<class Archive>
    void load(Archive & archive, cv::Mat& m)
    {
        int rows, cols, type;
        archive(rows, cols, type);

        if (rows == 0 || cols == 0 || type == 0) {
            m = cv::Mat(); // Assign empty matrix
            return;
        }

        m.create(rows, cols, type);
        const int data_size = rows * cols * m.elemSize();
        archive(cereal::binary_data(m.data, data_size));
    }

} // namespace cereal

#endif //CEREAL_TYPES_BASE_HPP
