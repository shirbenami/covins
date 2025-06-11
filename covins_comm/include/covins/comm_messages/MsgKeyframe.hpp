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

#include <vector>
#include <map>
#include <string>
#include <opencv2/core/core.hpp> // For cv::Mat
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// COVINS Base Types
#include <covins/covins_base/typedefs_base.hpp>
#include <covins/comm_abstraction/IMessage.hpp> // Inherit from IMessage

namespace covins {

// Define camera model enum
enum class eCamModel {
    PINHOLE = 0,
    FISHEYE = 1,
    OMNI = 2
};

// Define distortion model enum
enum class eDistortionModel {
    NONE = 0,
    RADTAN = 1,
    EQUIDISTANT = 2
};

// Structure for IMU and Camera calibration parameters,
// embedded directly into MsgKeyframe for convenience.
struct VI_Calibration {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure Eigen types are correctly aligned

    // Sensor transform from IMU (S) to Camera (C) frame
    TypeDefs::TransformType T_SC;

    // Camera calibration parameters
    eCamModel cam_model;          // Camera model type
    eDistortionModel dist_model;  // Distortion model type
    TypeDefs::DynamicVectorType dist_coeffs; // Distortion coefficients (e.g., k1, k2, p1, p2, k3)

    double img_width;
    double img_height;

    double fx; // Focal length x
    double fy; // Focal length y
    double cx; // Principal point x
    double cy; // Principal point y

    // IMU calibration parameters
    double gyr_noise_density;      // Gyroscope noise density
    double gyr_random_walk;        // Gyroscope random walk
    double acc_noise_density;      // Accelerometer noise density
    double acc_random_walk;        // Accelerometer random walk
    double gyr_bias_noise_density; // Gyroscope bias noise density
    double acc_bias_noise_density; // Accelerometer bias noise density
    double imu_freq;               // IMU frequency
    double gravity;                // Gravity magnitude
    TypeDefs::Vector3Type gravity_vec; // Gravity vector
    double time_offset_imu_cam;    // Time offset between IMU and Camera
    double min_imu_preint_time;    // Minimum IMU preintegration time
    double max_imu_preint_time;    // Maximum IMU preintegration time


    // Default constructor
    VI_Calibration() :
        T_SC(TypeDefs::TransformType::Identity()),
        cam_model(eCamModel::PINHOLE),
        dist_model(eDistortionModel::NONE),
        dist_coeffs(TypeDefs::DynamicVectorType::Zero(4)), // Default to 4 coeffs for radtan
        img_width(0), img_height(0),
        fx(0), fy(0), cx(0), cy(0),
        gyr_noise_density(0), gyr_random_walk(0),
        acc_noise_density(0), acc_random_walk(0),
        gyr_bias_noise_density(0), acc_bias_noise_density(0),
        imu_freq(0), gravity(0), gravity_vec(TypeDefs::Vector3Type::Zero()),
        time_offset_imu_cam(0), min_imu_preint_time(0), max_imu_preint_time(0)
    {}

    // Parameterized constructor
    VI_Calibration(
        const TypeDefs::TransformType& T_SC_,
        eCamModel cam_model_, eDistortionModel dist_model_,
        const TypeDefs::DynamicVectorType& dist_coeffs_,
        double img_width_, double img_height_,
        double fx_, double fy_, double cx_, double cy_,
        double gyr_noise_density_, double gyr_random_walk_,
        double acc_noise_density_, double acc_random_walk_,
        double gyr_bias_noise_density_, double acc_bias_noise_density_,
        double imu_freq_, double gravity_, const TypeDefs::Vector3Type& gravity_vec_,
        double time_offset_imu_cam_, double min_imu_preint_time_, double max_imu_preint_time_
    ) :
        T_SC(T_SC_),
        cam_model(cam_model_), dist_model(dist_model_),
        dist_coeffs(dist_coeffs_),
        img_width(img_width_), img_height(img_height_),
        fx(fx_), fy(fy_), cx(cx_), cy(cy_),
        gyr_noise_density(gyr_noise_density_), gyr_random_walk(gyr_random_walk_),
        acc_noise_density(acc_noise_density_), acc_random_walk(acc_random_walk_),
        gyr_bias_noise_density(gyr_bias_noise_density_), acc_bias_noise_density(acc_bias_noise_density_),
        imu_freq(imu_freq_), gravity(gravity_), gravity_vec(gravity_vec_),
        time_offset_imu_cam(time_offset_imu_cam_), min_imu_preint_time(min_imu_preint_time_), max_imu_preint_time_(max_imu_preint_time_)
    {}
};


class MsgKeyframe : public IMessage {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure Eigen types are correctly aligned

    /**
     * @brief Default constructor. Initializes base class with message type.
     */
    MsgKeyframe() : IMessage("Keyframe") {}

    // Keyframe ID (first: KF ID, second: Client ID)
    TypeDefs::IdType id;

    // Is this an update message for an existing keyframe?
    bool is_update_msg;

    // Timestamp
    double timestamp;

    // Pose of the keyframe in world coordinates (Twb)
    TypeDefs::TransformType T_w_b;

    // Previous pose for odometry delta calculation (Twb_prev)
    TypeDefs::TransformType T_w_b_prev;

    // Transform from sensor to camera frame (T_s_c) - should be in VI_Calibration generally
    TypeDefs::TransformType T_s_c; // Redundant if in calibration, but kept for compatibility if original ORB-SLAM3 uses it outside calibration.

    // Frame (image) data
    cv::Mat img; // Raw image data

    // Linear acceleration and angular velocity at this keyframe (from IMU)
    TypeDefs::Vector3Type lin_acc;
    TypeDefs::Vector3Type ang_vel;

    // Relative pose from reference frame (sref) to current sensor frame (s)
    TypeDefs::TransformType T_sref_s;

    // Unique IDs for previous/next keyframes in the graph
    TypeDefs::IdType id_predecessor;
    TypeDefs::IdType id_successor;
    TypeDefs::IdType id_reference; // Reference KeyFrame for this KF's pose in BA

    // Place Recognition Features
    std::vector<TypeDefs::AorsType, Eigen::aligned_allocator<TypeDefs::AorsType>> keypoints_aors;
    std::vector<TypeDefs::KeypointType, Eigen::aligned_allocator<TypeDefs::KeypointType>> keypoints_distorted;
    std::vector<TypeDefs::KeypointType, Eigen::aligned_allocator<TypeDefs::KeypointType>> keypoints_undistorted;
    cv::Mat descriptors;

    // Additional features for pose estimation (if different from PR features)
    std::vector<TypeDefs::AorsType, Eigen::aligned_allocator<TypeDefs::AorsType>> keypoints_aors_add;
    std::vector<TypeDefs::KeypointType, Eigen::aligned_allocator<TypeDefs::KeypointType>> keypoints_distorted_add;
    std::vector<TypeDefs::KeypointType, Eigen::aligned_allocator<TypeDefs::KeypointType>> keypoints_undistorted_add;
    cv::Mat descriptors_add;

    // Image dimensions for undistortion, if needed on receiver side
    double img_dim_x_min;
    double img_dim_y_min;
    double img_dim_x_max;
    double img_dim_y_max;

    // IMU and Camera Calibration parameters for this Keyframe
    VI_Calibration calibration; // Embed the calibration struct directly

    // --- IMessage interface implementation ---
    std::string getType() const override { return message_type_; }
    std::unique_ptr<IMessage> clone() const override;
    void serialize(ISerializer& serializer) const override;
    void deserialize(IDeserializer& deserializer) override;

    // Utility methods for image access
    void setImage(const cv::Mat& img); // FIX: Added declaration
    const cv::Mat& getImage() const;    // FIX: Added declaration
};

} // namespace covins
