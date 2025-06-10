//
// Created by user1 on 10/06/25.
//

#ifndef VI_CALIBRATION_HPP
#define VI_CALIBRATION_HPP
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <covins/covins_base/typedefs_base.hpp> // For TypeDefs

// Cereal includes for serialization of the struct
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp> // For std::pair

// Cereal NVP macro for named-value pairs in serialization
#include <cereal/types/common.hpp>
#include <cereal/access.hpp> // Required for friend class access

namespace covins {

// Define camera models
enum eCamModel {
    PINHOLE = 0,
    FISHEYE = 1
};

// Define distortion models
enum eDistortionModel {
    RADTAN = 0,
    EQUIDISTANT = 1
};

/**
 * @brief Structure to hold Visual-Inertial (VI) Calibration parameters.
 *
 * This struct encapsulates intrinsic and extrinsic camera parameters,
 * IMU parameters, and other relevant calibration data for a VI system.
 */
struct VICalibration {
    // Camera intrinsics and distortion
    Eigen::Matrix<double, 3, 3> K_cam;              // Camera intrinsic matrix
    TypeDefs::DynamicVectorType dist_coeffs;        // Distortion coefficients (dynamic size for k3, k4 etc)
    eCamModel cam_model;                            // Camera model type (PINHOLE, FISHEYE)
    eDistortionModel dist_model;                    // Distortion model type (RADTAN, EQUIDISTANT)
    int img_width;                                  // Image width
    int img_height;                                 // Image height

    // Standard camera parameters
    double fx, fy, cx, cy;

    // IMU to Camera extrinsic transformation (T_sensor_camera)
    TypeDefs::TransformType T_SC;

    // IMU parameters
    double s;         // Scale factor (stereo baseline ratio or similar)
    double inv_s;     // Inverse scale factor
    double freq;      // Frequency
    double delta_t;   // Time delta for integration
    double min_norm_acc;
    double max_norm_acc;
    double min_norm_gyro;
    double max_norm_gyro;

    // IMU noise parameters
    double sigma_g;   // Gyroscope noise density
    double sigma_ga;  // Gyroscope random walk
    double sigma_bg;  // Gyroscope bias random walk
    double sigma_ba;  // Accelerometer bias random walk
    double sigma_gw;  // Gyroscope white noise
    double sigma_aw;  // Accelerometer white noise

    TypeDefs::Vector3Type g;        // Gravity vector
    TypeDefs::Vector3Type w_c_b;    // Angular velocity bias

    bool is_synced; // Whether IMU and camera are synchronized

    // Default constructor
    VICalibration() :
        K_cam(Eigen::Matrix3d::Identity()),
        dist_coeffs(TypeDefs::DynamicVectorType::Zero(4)), // Default to 4 coeffs
        cam_model(PINHOLE), dist_model(RADTAN),
        img_width(0), img_height(0),
        fx(0.0), fy(0.0), cx(0.0), cy(0.0),
        T_SC(TypeDefs::TransformType::Identity()),
        s(0.0), inv_s(0.0), freq(0.0), delta_t(0.0),
        min_norm_acc(0.0), max_norm_acc(0.0),
        min_norm_gyro(0.0), max_norm_gyro(0.0),
        sigma_g(0.0), sigma_ga(0.0), sigma_bg(0.0), sigma_ba(0.0),
        sigma_gw(0.0), sigma_aw(0.0), // Initialize these
        g(TypeDefs::Vector3Type::Zero()), w_c_b(TypeDefs::Vector3Type::Zero()),
        is_synced(false) {}

    // Constructor with parameters (to match frontend_wrapper.cpp's construction)
    // Ensure the order and number of parameters match where this is called.
    VICalibration(
        const TypeDefs::TransformType& T_SC_,
        eCamModel cam_model_, eDistortionModel dist_model_,
        const TypeDefs::DynamicVectorType& dist_coeffs_,
        int img_width_, int img_height_,
        double fx_, double fy_, double cx_, double cy_,
        double s_, double inv_s_, double freq_, double delta_t_,
        double min_norm_acc_, double max_norm_acc_,
        double min_norm_gyro_, double max_norm_gyro_,
        double sigma_g_, double sigma_ga_, double sigma_bg_, double sigma_ba_,
        const TypeDefs::Vector3Type& g_, const TypeDefs::Vector3Type& w_c_b_,
        bool is_synced_, double sigma_gw_, double sigma_aw_) :
        dist_coeffs(dist_coeffs_), cam_model(cam_model_), dist_model(dist_model_),
        img_width(img_width_), img_height(img_height_),
        fx(fx_), fy(fy_), cx(cx_), cy(cy_),
        T_SC(T_SC_), s(s_), inv_s(inv_s_), freq(freq_), delta_t(delta_t_),
        min_norm_acc(min_norm_acc_), max_norm_acc(max_norm_acc_),
        min_norm_gyro(min_norm_gyro_), max_norm_gyro(max_norm_gyro_),
        sigma_g(sigma_g_), sigma_ga(sigma_ga_), sigma_bg(sigma_bg_), sigma_ba(sigma_ba_),
        sigma_gw(sigma_gw_), sigma_aw(sigma_aw_),
        g(g_), w_c_b(w_c_b_), is_synced(is_synced_)
    {
        K_cam << fx, 0, cx,
                 0, fy, cy,
                 0, 0, 1;
    }

    // Cereal serialization for VICalibration struct
    template<class Archive>
    void serialize(Archive & archive)
    {
        archive(CEREAL_NVP(K_cam), CEREAL_NVP(dist_coeffs), CEREAL_NVP(cam_model), CEREAL_NVP(dist_model),
                CEREAL_NVP(img_width), CEREAL_NVP(img_height),
                CEREAL_NVP(fx), CEREAL_NVP(fy), CEREAL_NVP(cx), CEREAL_NVP(cy),
                CEREAL_NVP(T_SC),
                CEREAL_NVP(s), CEREAL_NVP(inv_s), CEREAL_NVP(freq), CEREAL_NVP(delta_t),
                CEREAL_NVP(min_norm_acc), CEREAL_NVP(max_norm_acc),
                CEREAL_NVP(min_norm_gyro), CEREAL_NVP(max_norm_gyro),
                CEREAL_NVP(sigma_g), CEREAL_NVP(sigma_ga), CEREAL_NVP(sigma_bg), CEREAL_NVP(sigma_ba),
                CEREAL_NVP(sigma_gw), CEREAL_NVP(sigma_aw),
                CEREAL_NVP(g), CEREAL_NVP(w_c_b), CEREAL_NVP(is_synced));
    }
};

} // namespace covins

#endif //VI_CALIBRATION_HPP
