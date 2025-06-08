#pragma once

#include <string>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <map>      // For various maps if needed in keyframe
#include <utility>  // For std::pair

// OpenCV includes for image data and keypoints
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp> // For cv::KeyPoint
#include <opencv2/imgproc.hpp> // For cv::resize, cv::cvtColor if needed

// Eigen includes for transformation matrices and vectors
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Abstract message interface and serialization interfaces
#include <covins/comm_abstraction/IMessage.hpp> // Ensure .hpp extension
#include <covins/comm_abstraction/ISerializer.hpp> // IDeserializer is in the same header, ensure .hpp

// Covins base types and calibration structs (assuming these are defined elsewhere)
// Ensure this path is correct based on your 'covins_backend/include' structure
#include <covins/covins_base/typedefs_base.hpp> // Changed to .hpp, assuming your file is named typedefs_base.hpp
//#include <covins/covins_base/VICalibration.hpp> // Assuming this is a struct/class for camera calibration, changed to .hpp

namespace covins {

/**
 * @brief Concrete implementation of IMessage for Keyframe data.
 *
 * This class encapsulates all necessary information for a keyframe in the SLAM system,
 * including image, pose, features, and optionally calibration data. It handles its own
 * serialization and deserialization via the ISerializer/IDeserializer interfaces.
 */
class MsgKeyframe : public IMessage {
public:
    // Public data members (for direct access, similar to original structure)
    std::pair<int, int> id;         // Keyframe ID (first: index, second: client_id)
    double timestamp;               // Timestamp of the keyframe
    bool is_update_msg;             // Flag indicating if this is an update message

    // VICalibration calibration; // REMOVED: If this exists elsewhere, we'll re-add it from there.

    TypeDefs::TransformType T_s_c;  // Transform from sensor (IMU) frame to camera frame
    TypeDefs::Vector3Type lin_acc;  // Linear acceleration (e.g., from IMU)
    TypeDefs::Vector3Type ang_vel;  // Angular velocity (e.g., from IMU)

    // Keypoint and descriptor data
    std::vector<TypeDefs::AorsType> keypoints_aors;      // Angle, Octave, Response, Size for PR features
    std::vector<TypeDefs::KeypointType> keypoints_distorted; // Distorted keypoint coordinates for PR features
    std::vector<TypeDefs::KeypointType> keypoints_undistorted; // Undistorted keypoint coordinates for PR features
    cv::Mat descriptors;                               // Descriptors for PR features

    std::vector<TypeDefs::AorsType> keypoints_aors_add; // Angle, Octave, Response, Size for additional features
    std::vector<TypeDefs::KeypointType> keypoints_distorted_add; // Distorted keypoint coordinates for additional features
    std::vector<TypeDefs::KeypointType> keypoints_undistorted_add; // Undistorted keypoint coordinates for additional features
    cv::Mat descriptors_add;                           // Descriptors for additional features

    TypeDefs::TransformType T_sref_s; // Relative transform from reference sensor frame to current sensor frame

    std::pair<int, int> id_predecessor; // ID of the predecessor keyframe
    std::pair<int, int> id_reference;   // ID of the reference keyframe
    std::pair<int, int> id_successor;   // ID of the successor keyframe

    double img_dim_x_min, img_dim_y_min; // Image ROI min dimensions
    double img_dim_x_max, img_dim_y_max; // Image ROI max dimensions

    // Image data (Note: Cv::Mat can be large, consider serializing it as a binary blob)
    cv::Mat image; // The actual image data

public:
    /**
     * @brief Default constructor.
     */
    MsgKeyframe();

    /**
     * @brief Destructor.
     */
    virtual ~MsgKeyframe() = default;

    // --- IMessage interface implementation ---

    /**
     * @brief Returns the type identifier for this message.
     * @return "Keyframe"
     */
    std::string getType() const override { return "Keyframe"; }

    /**
     * @brief Creates a deep copy of this MsgKeyframe object.
     * @return A unique_ptr to the new MsgKeyframe instance.
     */
    std::unique_ptr<IMessage> clone() const override;

    /**
     * @brief Serializes the MsgKeyframe data using the provided ISerializer.
     * @param serializer The serializer instance to use.
     */
    void serialize(ISerializer& serializer) const override;

    /**
     * @brief Deserializes data into this MsgKeyframe object using the provided IDeserializer.
     * @param deserializer The deserializer instance to use.
     */
    void deserialize(IDeserializer& deserializer) override;

    // Utility methods to set image and get image (if not directly accessing public member)
    void setImage(const cv::Mat& img);
    const cv::Mat& getImage() const;
    void setTimestamp(double ts) { timestamp = ts; }
    double getTimestamp() const { return timestamp; }
};

} // namespace covins
