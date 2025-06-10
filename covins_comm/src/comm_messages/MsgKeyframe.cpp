#include <covins/comm_messages/MsgKeyframe.hpp> // Ensure .hpp extension
#include <iostream> // For error/debug output
#include <cstring>  // For std::memcpy

// Include OpenCV for image encoding/decoding and image processing functions
#include <opencv2/imgcodecs.hpp> // For cv::imencode, cv::imdecode
#include <opencv2/imgproc.hpp>   // For cv::resize

// !!! CRITICAL FIX: Include the full definition of ISerializer/IDeserializer !!!
#include <covins/comm_serialization/ISerializer.hpp>

namespace covins {

// Constructor definition removed as it's defined in the header.
// MsgKeyframe::MsgKeyframe()
//     : id({-1, -1}), timestamp(0.0), is_update_msg(false),
//       T_s_c(TypeDefs::TransformType::Identity()),
//       lin_acc(TypeDefs::Vector3Type::Zero()), ang_vel(TypeDefs::Vector3Type::Zero()),
//       T_sref_s(TypeDefs::TransformType::Identity()),
//       id_predecessor({-1, -1}), id_reference({-1, -1}), id_successor({-1, -1}),
//       img_dim_x_min(0.0), img_dim_y_min(0.0), img_dim_x_max(0.0), img_dim_y_max(0.0)
// {
//     descriptors = cv::Mat();
//     descriptors_add = cv::Mat();
//     img = cv::Mat(); // Changed from 'image' to 'img'
// }

std::unique_ptr<IMessage> MsgKeyframe::clone() const {
    // Create a new MsgKeyframe and copy all members
    auto cloned_msg = std::make_unique<MsgKeyframe>();

    cloned_msg->id = this->id;
    cloned_msg->timestamp = this->timestamp;
    cloned_msg->is_update_msg = this->is_update_msg;

    cloned_msg->T_w_b = this->T_w_b; // Added for completeness, was missing in clone
    cloned_msg->T_w_b_prev = this->T_w_b_prev; // Added for completeness

    cloned_msg->T_s_c = this->T_s_c;
    cloned_msg->lin_acc = this->lin_acc;
    cloned_msg->ang_vel = this->ang_vel;

    // Deep copy vectors and cv::Mat
    cloned_msg->keypoints_aors = this->keypoints_aors;
    cloned_msg->keypoints_distorted = this->keypoints_distorted;
    cloned_msg->keypoints_undistorted = this->keypoints_undistorted;
    cloned_msg->descriptors = this->descriptors.clone(); // Deep copy for cv::Mat

    cloned_msg->keypoints_aors_add = this->keypoints_aors_add;
    cloned_msg->keypoints_distorted_add = this->keypoints_distorted_add;
    cloned_msg->keypoints_undistorted_add = this->keypoints_undistorted_add;
    cloned_msg->descriptors_add = this->descriptors_add.clone(); // Deep copy for cv::Mat

    cloned_msg->T_sref_s = this->T_sref_s;

    cloned_msg->id_predecessor = this->id_predecessor;
    cloned_msg->id_reference = this->id_reference;
    cloned_msg->id_successor = this->id_successor;

    cloned_msg->img_dim_x_min = this->img_dim_x_min;
    cloned_msg->img_dim_y_min = this->img_dim_y_min;
    cloned_msg->img_dim_x_max = this->img_dim_x_max;
    cloned_msg->img_dim_y_max = this->img_dim_y_max;
    cloned_msg->img = this->img.clone(); // !!! FIX: Changed from 'image' to 'img' !!!

    cloned_msg->calibration = this->calibration; // Copy the nested struct

    return cloned_msg;
}

void MsgKeyframe::serialize(ISerializer& serializer) const {
    // IMPORTANT: Serialize the message type first for deserialization
    serializer.write("message_type", getType());

    // Basic Keyframe info
    serializer.write("id_first", id.first);
    serializer.write("id_second", id.second);
    serializer.write("timestamp", timestamp);
    serializer.write("is_update_msg", is_update_msg);

    // New: Serialize calibration struct members individually
    serializer.write("calib_T_SC", calibration.T_SC);
    serializer.write("calib_cam_model", static_cast<int>(calibration.cam_model));
    serializer.write("calib_dist_model", static_cast<int>(calibration.dist_model));

    // Serialize Eigen::DynamicVectorType for dist_coeffs
    // Convert to std::vector<double> then to std::vector<uint8_t>
    std::vector<double> dist_coeffs_vec_double(calibration.dist_coeffs.data(), calibration.dist_coeffs.data() + calibration.dist_coeffs.size());
    serializer.write("calib_dist_coeffs", std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(dist_coeffs_vec_double.data()),
                                                                reinterpret_cast<const uint8_t*>(dist_coeffs_vec_double.data()) + dist_coeffs_vec_double.size() * sizeof(double)));

    serializer.write("calib_img_width", calibration.img_width);
    serializer.write("calib_img_height", calibration.img_height);
    serializer.write("calib_fx", calibration.fx);
    serializer.write("calib_fy", calibration.fy);
    serializer.write("calib_cx", calibration.cx);
    serializer.write("calib_cy", calibration.cy);
    serializer.write("calib_gyr_noise_density", calibration.gyr_noise_density);
    serializer.write("calib_gyr_random_walk", calibration.gyr_random_walk);
    serializer.write("calib_acc_noise_density", calibration.acc_noise_density);
    serializer.write("calib_acc_random_walk", calibration.acc_random_walk);
    serializer.write("calib_gyr_bias_noise_density", calibration.gyr_bias_noise_density);
    serializer.write("calib_acc_bias_noise_density", calibration.acc_bias_noise_density);
    serializer.write("calib_imu_freq", calibration.imu_freq);
    serializer.write("calib_gravity", calibration.gravity);
    serializer.write("calib_gravity_vec", calibration.gravity_vec);
    serializer.write("calib_time_offset_imu_cam", calibration.time_offset_imu_cam);
    serializer.write("calib_min_imu_preint_time", calibration.min_imu_preint_time);
    serializer.write("calib_max_imu_preint_time", calibration.max_imu_preint_time);


    // Transformations and IMU data
    serializer.write("T_w_b", T_w_b);
    serializer.write("T_w_b_prev", T_w_b_prev); // Added for completeness
    serializer.write("T_s_c", T_s_c);
    serializer.write("lin_acc", lin_acc);
    serializer.write("ang_vel", ang_vel);

    // Keypoints and Descriptors (PR features)
    // Convert Eigen::MatrixX2d (KeypointType) and Eigen::Vector4d (AorsType) to flat vector for serialization
    std::vector<double> kps_distorted_flat(keypoints_distorted.size() * 2);
    for (size_t i = 0; i < keypoints_distorted.size(); ++i) {
        kps_distorted_flat[i*2] = keypoints_distorted[i][0];
        kps_distorted_flat[i*2 + 1] = keypoints_distorted[i][1];
    }
    serializer.write("keypoints_distorted_flat", std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(kps_distorted_flat.data()), reinterpret_cast<const uint8_t*>(kps_distorted_flat.data()) + kps_distorted_flat.size() * sizeof(double)));

    std::vector<double> aors_flat(keypoints_aors.size() * 4);
    for (size_t i = 0; i < keypoints_aors.size(); ++i) {
        aors_flat[i*4] = keypoints_aors[i][0];
        aors_flat[i*4 + 1] = keypoints_aors[i][1];
        aors_flat[i*4 + 2] = keypoints_aors[i][2];
        aors_flat[i*4 + 3] = keypoints_aors[i][3];
    }
    serializer.write("keypoints_aors_flat", std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(aors_flat.data()), reinterpret_cast<const uint8_t*>(aors_flat.data()) + aors_flat.size() * sizeof(double)));


    // Serialize cv::Mat descriptors as binary data
    if (!descriptors.empty()) {
        serializer.write("descriptors_rows", descriptors.rows);
        serializer.write("descriptors_cols", descriptors.cols);
        serializer.write("descriptors_type", descriptors.type()); // e.g., CV_8U, CV_32F

        size_t data_size = descriptors.total() * descriptors.elemSize();
        std::vector<uint8_t> data_vec(data_size);
        std::memcpy(data_vec.data(), descriptors.data, data_size);
        serializer.write("descriptors_data", data_vec);
    } else {
        serializer.write("descriptors_rows", 0); // Indicate empty
        serializer.write("descriptors_cols", 0);
        serializer.write("descriptors_type", 0);
        serializer.write("descriptors_data", std::vector<uint8_t>{});
    }

    // Keypoints and Descriptors (additional features) - similar approach
    std::vector<double> kps_add_distorted_flat(keypoints_distorted_add.size() * 2);
    for (size_t i = 0; i < keypoints_distorted_add.size(); ++i) {
        kps_add_distorted_flat[i*2] = keypoints_distorted_add[i][0];
        kps_add_distorted_flat[i*2 + 1] = keypoints_distorted_add[i][1];
    }
    serializer.write("keypoints_distorted_add_flat", std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(kps_add_distorted_flat.data()), reinterpret_cast<const uint8_t*>(kps_add_distorted_flat.data()) + kps_add_distorted_flat.size() * sizeof(double)));

    std::vector<double> aors_add_flat(keypoints_aors_add.size() * 4);
    for (size_t i = 0; i < keypoints_aors_add.size(); ++i) {
        aors_add_flat[i*4] = keypoints_aors_add[i][0];
        aors_add_flat[i*4 + 1] = keypoints_aors_add[i][1]; // Fixed: Was aors_add_flat[i*4 + 1] = aors_add_flat[i*4 + 1]
        aors_add_flat[i*4 + 2] = keypoints_aors_add[i][2];
        aors_add_flat[i*4 + 3] = keypoints_aors_add[i][3];
    }
    serializer.write("keypoints_aors_add_flat", std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(aors_add_flat.data()), reinterpret_cast<const uint8_t*>(aors_add_flat.data()) + aors_add_flat.size() * sizeof(double)));


    if (!descriptors_add.empty()) {
        serializer.write("descriptors_add_rows", descriptors_add.rows);
        serializer.write("descriptors_add_cols", descriptors_add.cols);
        serializer.write("descriptors_add_type", descriptors_add.type());

        size_t data_size = descriptors_add.total() * descriptors_add.elemSize();
        std::vector<uint8_t> data_vec(data_size);
        std::memcpy(data_vec.data(), descriptors_add.data, data_size);
        serializer.write("descriptors_add_data", data_vec);
    } else {
        serializer.write("descriptors_add_rows", 0);
        serializer.write("descriptors_add_cols", 0);
        serializer.write("descriptors_add_type", 0);
        serializer.write("descriptors_add_data", std::vector<uint8_t>{});
    }


    serializer.write("T_sref_s", T_sref_s);

    serializer.write("id_predecessor_first", id_predecessor.first);
    serializer.write("id_predecessor_second", id_predecessor.second);
    serializer.write("id_reference_first", id_reference.first);
    serializer.write("id_reference_second", id_reference.second);
    serializer.write("id_successor_first", id_successor.first);
    serializer.write("id_successor_second", id_successor.second);

    serializer.write("img_dim_x_min", img_dim_x_min);
    serializer.write("img_dim_y_min", img_dim_y_min);
    serializer.write("img_dim_x_max", img_dim_x_max);
    serializer.write("img_dim_y_max", img_dim_y_max);

    // Serialize the image itself. Using JPEG/PNG compression for cv::Mat is common.
    if (!img.empty()) { // !!! FIX: Changed from 'image' to 'img' !!!
        std::vector<uint8_t> image_buffer;
        // Use JPEG compression for smaller size. Quality 80%.
        cv::imencode(".jpg", img, image_buffer, {cv::IMWRITE_JPEG_QUALITY, 80}); // !!! FIX: Changed from 'image' to 'img' !!!
        serializer.write("image_data", image_buffer);
        serializer.write("image_rows", img.rows); // !!! FIX: Changed from 'image' to 'img' !!!
        serializer.write("image_cols", img.cols); // !!! FIX: Changed from 'image' to 'img' !!!
        serializer.write("image_type", img.type()); // Store type (e.g., CV_8UC1 for grayscale) // !!! FIX: Changed from 'image' to 'img' !!!
    } else {
        serializer.write("image_data", std::vector<uint8_t>{});
        serializer.write("image_rows", 0);
        serializer.write("image_cols", 0);
        serializer.write("image_type", 0);
    }
}

void MsgKeyframe::deserialize(IDeserializer& deserializer) {
    // IMPORTANT: Deserialize the message type first
    std::string message_type_read = deserializer.readString("message_type");
    // You might want to assert or check if message_type_read matches "Keyframe" here
    if (message_type_read != getType()) {
        std::cerr << "MsgKeyframe: Deserialization type mismatch. Expected '" << getType() << "', got '" << message_type_read << "'" << std::endl;
        // Handle error, maybe throw exception or set invalid state
    }

    // Basic Keyframe info
    id.first = deserializer.readInt("id_first");
    id.second = deserializer.readInt("id_second");
    timestamp = deserializer.readDouble("timestamp");
    is_update_msg = deserializer.readBool("is_update_msg");

    // New: Deserialize calibration struct members individually
    calibration.T_SC = deserializer.readTransform("calib_T_SC");
    calibration.cam_model = static_cast<eCamModel>(deserializer.readInt("calib_cam_model"));
    calibration.dist_model = static_cast<eDistortionModel>(deserializer.readInt("calib_dist_model"));

    // Deserialize dist_coeffs (Eigen::DynamicVectorType)
    std::vector<uint8_t> dist_coeffs_bytes = deserializer.readBinary("calib_dist_coeffs");
    if (!dist_coeffs_bytes.empty() && dist_coeffs_bytes.size() % sizeof(double) == 0) {
        calibration.dist_coeffs.resize(dist_coeffs_bytes.size() / sizeof(double));
        std::memcpy(calibration.dist_coeffs.data(), dist_coeffs_bytes.data(), dist_coeffs_bytes.size());
    } else {
        calibration.dist_coeffs.setZero(); // Or resize(0) depending on desired empty state
    }

    calibration.img_width = deserializer.readDouble("calib_img_width");
    calibration.img_height = deserializer.readDouble("calib_img_height");
    calibration.fx = deserializer.readDouble("calib_fx");
    calibration.fy = deserializer.readDouble("calib_fy");
    calibration.cx = deserializer.readDouble("calib_cx");
    calibration.cy = deserializer.readDouble("calib_cy");
    calibration.gyr_noise_density = deserializer.readDouble("calib_gyr_noise_density");
    calibration.gyr_random_walk = deserializer.readDouble("calib_gyr_random_walk");
    calibration.acc_noise_density = deserializer.readDouble("calib_acc_noise_density");
    calibration.acc_random_walk = deserializer.readDouble("calib_acc_random_walk");
    calibration.gyr_bias_noise_density = deserializer.readDouble("calib_gyr_bias_noise_density");
    calibration.acc_bias_noise_density = deserializer.readDouble("calib_acc_bias_noise_density");
    calibration.imu_freq = deserializer.readDouble("calib_imu_freq");
    calibration.gravity = deserializer.readDouble("calib_gravity");
    calibration.gravity_vec = deserializer.readVector3d("calib_gravity_vec");
    calibration.time_offset_imu_cam = deserializer.readDouble("calib_time_offset_imu_cam");
    calibration.min_imu_preint_time = deserializer.readDouble("calib_min_imu_preint_time");
    calibration.max_imu_preint_time = deserializer.readDouble("calib_max_imu_preint_time");


    // Transformations and IMU data
    T_w_b = deserializer.readTransform("T_w_b");
    T_w_b_prev = deserializer.readTransform("T_w_b_prev"); // Added for completeness
    T_s_c = deserializer.readTransform("T_s_c");
    lin_acc = deserializer.readVector3d("lin_acc");
    ang_vel = deserializer.readVector3d("ang_vel");

    // Keypoints and Descriptors (PR features)
    std::vector<uint8_t> kps_distorted_flat_bytes = deserializer.readBinary("keypoints_distorted_flat");
    if (!kps_distorted_flat_bytes.empty() && kps_distorted_flat_bytes.size() % sizeof(double) == 0) {
        std::vector<double> kps_distorted_flat(kps_distorted_flat_bytes.size() / sizeof(double));
        std::memcpy(kps_distorted_flat.data(), kps_distorted_flat_bytes.data(), kps_distorted_flat_bytes.size());
        keypoints_distorted.resize(kps_distorted_flat.size() / 2);
        for (size_t i = 0; i < keypoints_distorted.size(); ++i) {
            keypoints_distorted[i][0] = kps_distorted_flat[i*2];
            keypoints_distorted[i][1] = kps_distorted_flat[i*2 + 1];
        }
    } else {
        keypoints_distorted.clear();
    }

    std::vector<uint8_t> aors_flat_bytes = deserializer.readBinary("keypoints_aors_flat");
    if (!aors_flat_bytes.empty() && aors_flat_bytes.size() % sizeof(double) == 0) {
        std::vector<double> aors_flat(aors_flat_bytes.size() / sizeof(double));
        std::memcpy(aors_flat.data(), aors_flat_bytes.data(), aors_flat_bytes.size());
        keypoints_aors.resize(aors_flat.size() / 4);
        for (size_t i = 0; i < keypoints_aors.size(); ++i) {
            keypoints_aors[i][0] = aors_flat[i*4];
            keypoints_aors[i][1] = aors_flat[i*4 + 1];
            keypoints_aors[i][2] = aors_flat[i*4 + 2];
            keypoints_aors[i][3] = aors_flat[i*4 + 3];
        }
    } else {
        keypoints_aors.clear();
    }


    int desc_rows = deserializer.readInt("descriptors_rows");
    int desc_cols = deserializer.readInt("descriptors_cols");
    int desc_type = deserializer.readInt("descriptors_type");
    std::vector<uint8_t> descriptors_data = deserializer.readBinary("descriptors_data");

    if (desc_rows > 0 && desc_cols > 0 && !descriptors_data.empty()) {
        descriptors = cv::Mat(desc_rows, desc_cols, desc_type, descriptors_data.data()).clone();
    } else {
        descriptors = cv::Mat(); // Empty matrix
    }

    // Keypoints and Descriptors (additional features)
    std::vector<uint8_t> kps_add_distorted_flat_bytes = deserializer.readBinary("keypoints_distorted_add_flat");
    if (!kps_add_distorted_flat_bytes.empty() && kps_add_distorted_flat_bytes.size() % sizeof(double) == 0) {
        std::vector<double> kps_add_distorted_flat(kps_add_distorted_flat_bytes.size() / sizeof(double));
        std::memcpy(kps_add_distorted_flat.data(), kps_add_distorted_flat_bytes.data(), kps_add_distorted_flat_bytes.size());
        keypoints_distorted_add.resize(kps_add_distorted_flat.size() / 2);
        for (size_t i = 0; i < keypoints_distorted_add.size(); ++i) {
            keypoints_distorted_add[i][0] = kps_add_distorted_flat[i*2];
            keypoints_distorted_add[i][1] = kps_add_distorted_flat[i*2 + 1];
        }
    } else {
        keypoints_distorted_add.clear();
    }

    std::vector<uint8_t> aors_add_flat_bytes = deserializer.readBinary("keypoints_aors_add_flat");
    if (!aors_add_flat_bytes.empty() && aors_add_flat_bytes.size() % sizeof(double) == 0) {
        std::vector<double> aors_add_flat(aors_add_flat_bytes.size() / sizeof(double));
        std::memcpy(aors_add_flat.data(), aors_add_flat_bytes.data(), aors_add_flat_bytes.size());
        keypoints_aors_add.resize(aors_add_flat.size() / 4);
        for (size_t i = 0; i < keypoints_aors_add.size(); ++i) {
            keypoints_aors_add[i][0] = aors_add_flat[i*4];
            keypoints_aors_add[i][1] = aors_add_flat[i*4 + 1];
            keypoints_aors_add[i][2] = aors_add_flat[i*4 + 2];
            keypoints_aors_add[i][3] = aors_add_flat[i*4 + 3];
        }
    } else {
        keypoints_aors_add.clear();
    }


    int desc_add_rows = deserializer.readInt("descriptors_add_rows");
    int desc_add_cols = deserializer.readInt("descriptors_add_cols");
    int desc_add_type = deserializer.readInt("descriptors_add_type");
    std::vector<uint8_t> descriptors_add_data = deserializer.readBinary("descriptors_add_data");

    if (desc_add_rows > 0 && desc_add_cols > 0 && !descriptors_add_data.empty()) {
        descriptors_add = cv::Mat(desc_add_rows, desc_add_cols, desc_add_type, descriptors_add_data.data()).clone();
    } else {
        descriptors_add = cv::Mat(); // Empty matrix
    }


    T_sref_s = deserializer.readTransform("T_sref_s");

    id_predecessor.first = deserializer.readInt("id_predecessor_first");
    id_predecessor.second = deserializer.readInt("id_predecessor_second");
    id_reference.first = deserializer.readInt("id_reference_first");
    id_reference.second = deserializer.readInt("id_reference_second");
    id_successor.first = deserializer.readInt("id_successor_first");
    id_successor.second = deserializer.readInt("id_successor_second");

    img_dim_x_min = deserializer.readDouble("img_dim_x_min");
    img_dim_y_min = deserializer.readDouble("img_dim_y_min");
    img_dim_x_max = deserializer.readDouble("img_dim_x_max");
    img_dim_y_max = deserializer.readDouble("img_dim_y_max");

    // Deserialize image
    std::vector<uint8_t> image_data = deserializer.readBinary("image_data");
    int image_rows = deserializer.readInt("image_rows");
    int image_cols = deserializer.readInt("image_cols");
    int image_type = deserializer.readInt("image_type");

    if (!image_data.empty() && image_rows > 0 && image_cols > 0) {
        // Decode image from buffer. Assuming it was encoded as JPEG.
        img = cv::imdecode(image_data, cv::IMREAD_UNCHANGED); // !!! FIX: Changed 'image' to 'img' !!!
        if (img.empty()) { // !!! FIX: Changed 'image' to 'img' !!!
            std::cerr << "MsgKeyframe: Failed to imdecode image data." << std::endl;
        }
        // Resize and convert type if necessary (might be different from original if imdecode changes it)
        if (img.rows != image_rows || img.cols != image_cols || img.type() != image_type) { // !!! FIX: Changed 'image' to 'img' !!!
             // For example, if original was CV_8UC1 but JPEG decoded to CV_8UC3, handle it.
             // For now, simple resize and type conversion is enough to prevent crashes.
             // Deeper checks and conversions might be needed.
             cv::resize(img, img, cv::Size(image_cols, image_rows)); // !!! FIX: Changed 'image' to 'img' !!!
             img.convertTo(img, image_type); // !!! FIX: Changed 'image' to 'img' !!!
        }

    } else {
        img = cv::Mat(); // Empty matrix // !!! FIX: Changed 'image' to 'img' !!!
    }
}

void MsgKeyframe::setImage(const cv::Mat& img) {
    this->img = img.clone(); // Ensure deep copy // !!! FIX: Changed from 'image' to 'img' !!!
}

const cv::Mat& MsgKeyframe::getImage() const {
    return img; // !!! FIX: Changed from 'image' to 'img' !!!
}

} // namespace covins
