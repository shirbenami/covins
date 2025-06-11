/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp> // For cv::Mat

#include <mutex>
#include <thread>
#include <tuple>
#include <map>
#include <vector>
#include <set>
#include <list>
#include <deque>
#include <string>
#include <atomic>
#include <iostream> // For std::cerr and std::cout
#include <utility> // For std::pair, std::make_pair

namespace covins {

class TypeDefs {
public:
    using precision_t = double;

    // Eigen types
    using Vector2Type = Eigen::Matrix<precision_t, 2, 1>;
    using Vector3Type = Eigen::Matrix<precision_t, 3, 1>;
    using Vector4Type = Eigen::Matrix<precision_t, 4, 1>;
    using Vector6Type = Eigen::Matrix<precision_t, 6, 1>;
    using VectorXType = Eigen::Matrix<precision_t, Eigen::Dynamic, 1>;
    using Matrix2Type = Eigen::Matrix<precision_t, 2, 2>;
    using Matrix3Type = Eigen::Matrix<precision_t, 3, 3>;
    using Matrix4Type = Eigen::Matrix<precision_t, 4, 4>;
    using Matrix6Type = Eigen::Matrix<precision_t, 6, 6>;
    using MatrixXType = Eigen::Matrix<precision_t, Eigen::Dynamic, Eigen::Dynamic>;

    // FIX: Added Matrix9Type definition
    using Matrix9Type = Eigen::Matrix<precision_t, 9, 9>;

    // Add DynamicVectorType explicitly for clarity and usage
    using DynamicVectorType = Eigen::Matrix<precision_t, Eigen::Dynamic, 1>;


    using TransformType = Eigen::Matrix<precision_t, 4, 4>; // For SE3 transformations
    using QuaternionType = Eigen::Quaternion<precision_t>;

    // ORB-SLAM3 related types for compatibility
    using KeypointType = Eigen::Matrix<float, 2, 1>; // For (u,v) pixel coordinates
    using AorsType = Eigen::Matrix<float, 4, 1>;     // Angle, Octave, Response, Size

    // Thread management
    using ThreadPtr = std::unique_ptr<std::thread>;

    // Generic types for inter-client communication
    using MsgInfoType = std::vector<uint32_t>; // Generic message information vector
    using ClientIdType = size_t;               // Type for client IDs
    using IdType = std::pair<long unsigned int, ClientIdType>; // For unique IDs across clients (e.g., KeyFrame ID, Client ID)

    // Custom Error/Info Output Macros (Simplified to avoid conflicts)
#ifndef NDEBUG // Only show detailed error in debug builds
#define COUTERROR std::cerr << "\033[1;31m[ERROR]\033[0m " << __func__ << ":" << __LINE__ << ": "
#else
#define COUTERROR std::cerr << "\033[1;31m[ERROR]\033[0m "
#endif

#define COUTWARN std::cerr << "\033[1;33m[WARN]\033[0m "
#define COUTINFO std::cout << "\033[1;34m[INFO]\033[0m "

}; // class TypeDefs

} // namespace covins
