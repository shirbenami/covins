#include "ros_conversion.hpp"

namespace covins {

covins_msgs::msg::Keyframe ToRosMsg(const MsgKeyframe& src) {
    covins_msgs::msg::Keyframe dst;
    dst.timestamp = src.timestamp;
    dst.id = src.id.second; // or pack both first/second as needed

    // Calibration (if you have a VICalibration.msg)
    dst.calibration = ToRosCalibration(src.calibration);

    dst.img_dim_x_min = src.img_dim_x_min;
    dst.img_dim_y_min = src.img_dim_y_min;
    dst.img_dim_x_max = src.img_dim_x_max;
    dst.img_dim_y_max = src.img_dim_y_max;

    // Keypoints (convert vector<cv::KeyPoint> to custom msg)
    dst.keypoints_distorted = ToRosKeypoints(src.keypoints_distorted);
    // ...repeat for other keypoints...

    // Descriptors (convert cv::Mat to vector<uint8>)
    dst.descriptors = MatToVector(src.descriptors);

    // Poses (convert Eigen::Matrix to geometry_msgs/Pose or custom msg)
    dst.T_w_s = ToRosTransform(src.T_w_s);

    // Velocity, bias, etc.
    dst.velocity = ToRosVector3(src.velocity);
    dst.bias_gyro = ToRosVector3(src.bias_gyro);
    dst.bias_accel = ToRosVector3(src.bias_accel);

    // Preintegration (if you have a PreintegrationData.msg)
    dst.preintegration = ToRosPreintegration(src.preintegration);

    // Landmarks (if you have a LandmarksMinimalType.msg)
    dst.landmarks = ToRosLandmarks(src.landmarks);

    // etc. for all other fields...
    return dst;
}

covins_msgs::msg::Landmark ToRosMsg(const MsgLandmark& src) {
    covins_msgs::msg::Landmark dst;
    dst.is_update_msg = src.is_update_msg;
    dst.id = src.id.first;
    dst.client_id = src.id.second;
    dst.id_reference = src.id_reference.first;
    dst.ref_client_id = src.id_reference.second;

    dst.pos_ref.x = src.pos_ref.x();
    dst.pos_ref.y = src.pos_ref.y();
    dst.pos_ref.z = src.pos_ref.z();

    // Convert observations map to array of Observation.msg
    for (const auto& obs : src.observations) {
        covins_msgs::msg::Observation obs_msg;
        obs_msg.kf_id = obs.first.first;
        obs_msg.client_id = obs.first.second;
        obs_msg.value = obs.second;
        dst.observations.push_back(obs_msg);
    }
    return dst;
}

covins_msgs::msg::DataBundle ToRosMsg(const data_bundle& src) {
    covins_msgs::msg::DataBundle dst;
    for (const auto& kf : src.keyframes) {
        dst.keyframes.push_back(ToRosMsg(kf));
    }
    for (const auto& lm : src.landmarks) {
        dst.landmarks.push_back(ToRosMsg(lm));
    }
    // Add other fields from data_bundle if needed
    // dst.header.stamp = ...;
    // dst.client_id = src.client_id;
    return dst;
}

} // namespace covins