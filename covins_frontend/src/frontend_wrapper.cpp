#include "frontend_wrapper.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <typeinfo> // For dynamic_cast type checking (though getType() is preferred)
#include <cmath>    // For std::abs

// Include ROS for ros::param::get
#include <ros/ros.h>

// NEW ABSTRACTION LAYER INCLUDES
#include <covins/comm_abstraction/CommunicatorFactory.hpp>
#include <covins/comm_messages/MsgImage.hpp>    // For handling incoming image messages
#include <covins/comm_messages/MsgOdometry.hpp> // For handling incoming odometry messages
#include <covins/comm_messages/MsgKeyframe.hpp> // For handling outgoing keyframe messages
#include <covins/covins_base/config_comm.hpp> // For covins_params

namespace covins {

// Define defpair here as a constant, to avoid macro redefinition issues from typedefs_base.hpp
// If KFRANGE and MAPRANGE are defined elsewhere, you might need to adjust this.
// For now, based on your previous usage, a simple -1, -1 pair.
// Note: If KFRANGE and MAPRANGE are actually defined as constants somewhere, you'd use them.
// If this is a placeholder, -1,-1 is fine.
const std::pair<int, int> defpair = {-1, -1};

// COUTERROR, COUTWARN, COUTINFO are now stream macros from typedefs_base.hpp
// NO LONGER NEED: const char* COUTERROR = "[ERROR]";

FrontendWrapper::FrontendWrapper() {
  // Constructor: Initialize parameters, but defer communication setup to run()
  // as it depends on ROS initialization which might not be complete here.
}

auto FrontendWrapper::run()-> void {

  COUTINFO << "Run Wrapper ....." << std::endl;

  std::string config_file;
  // Use ros::param::get directly.
  if (!ros::param::get("~config_file", config_file)) {
      COUTERROR << " Failed to get config_file parameter." << std::endl;
      return;
  }

  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
      COUTERROR << " Failed to open config file: " << config_file << std::endl;
      return;
  }

  // Parse camera and ORB parameters
  bool b_parse_cam = ParseCamParamFile(fSettings);
  if(!b_parse_cam)
  {
      COUTERROR << "Error with the camera parameters in the config file" << std::endl;
      return;
  }

  bool b_parse_orb = ParseORBParamFile(fSettings);
  if(!b_parse_orb)
  {
      COUTERROR << "Error with the ORB parameters in the config file" << std::endl;
      return;
  }

  // Initialize synchronization tolerance from settings
  cv::FileNode node = fSettings["sync_tolerance"];
  if(!node.empty() && node.isReal())
  {
      sync_tolerance_ = node.real();
      COUTINFO << "Synchronization tolerance set to: " << sync_tolerance_ << " seconds" << std::endl;
  }
  else
  {
      sync_tolerance_ = 0.05; // Default tolerance if not specified
      COUTWARN << "sync_tolerance not found in config. Using default: " << sync_tolerance_ << " seconds" << std::endl;
  }


  // NEW COVINS Comm Integration using Abstraction Layer
  covins_params::ShowParamsComm(); // Use the static method from the namespace

  std::string protocol_type = covins_params::GetCommProtocolType();
  std::string serialization_format = covins_params::GetCommSerializationFormat();
  std::string comm_address = covins_params::GetServerIP();
  int comm_port = covins_params::GetPort(); // Now returns int directly

  comm_ = CommunicatorFactory::createCommunicator(protocol_type, serialization_format, comm_address, comm_port);

  if (!comm_) {
      COUTERROR << " Failed to create communication interface! Protocol: " << protocol_type << std::endl;
      return;
  }

  // Register the generic callback for incoming messages
  comm_->registerReceiveCallback(
      std::bind(&FrontendWrapper::genericMessageCallback, this, std::placeholders::_1)
  );

  // Connect the communicator
  if (!comm_->connect(comm_address, comm_port)) {
      COUTERROR << " Failed to connect communication interface!" << std::endl;
      return;
  }

  // Get ID from back-end (This part of logic needs to be correctly implemented
  // in the ICommunicator or a higher-level handshake process to actually
  // retrieve the client ID. For now, it's a placeholder.)
  client_id_ = 0; // TEMPORARY: Assign a default client ID for now.
                   // Proper client ID assignment needs a backend message.
  COUTINFO << "Set client ID (temporary): " << client_id_ << std::endl;
  // ------------------

  // Remaining initializations
  // prev_pos_, curr_pos_, prev_quat_, curr_quat_ initialized when first odometry arrives
  // Twc_prev_ also initialized when first odometry arrives
  prev_pos_ = Eigen::Vector3d::Zero();
  curr_pos_ = Eigen::Vector3d::Zero();
  prev_quat_ = Eigen::Quaterniond::Identity();
  curr_quat_ = Eigen::Quaterniond::Identity();
  Twc_prev_ = TransformType::Identity(); // Will be updated by first odom
  prev_ts_ = 0;
  curr_ts_ = 0;
  kf_count_ = 0;

  // Keep the main thread alive while the communicator is connected.
  // The actual message processing happens in genericMessageCallback,
  // which is triggered by the ICommunicator's internal mechanism (e.g., a spin thread in Ros1Communicator).
  while (comm_->isConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
  }
  COUTINFO << "FrontendWrapper exiting main loop." << std::endl;
}

/**
 * @brief Converts image, camera parameters and features to message format.
 * @param msg The message containing the converted data.
 * @param img The image to be converted.
 * @param T_wc The transform from world to camera coordinates.
 * @param T_wc_prev The transform from the previous world to camera coordinates.
 * @param client_id The ID of the client.
 * @param index The index of the image.
 * @param ts The timestamp of the image.
 */
void FrontendWrapper::convertToMsg(covins::MsgKeyframe &msg, cv::Mat &img,
                                   TransformType T_wc, TransformType T_wc_prev,int client_id,
                                   int index, double ts) {

  msg.is_update_msg = false;

  msg.id.first = index;
  msg.id.second = client_id;
  msg.timestamp = ts;

  // Assign directly to msg.calibration members.
  // These are members of `msg.calibration` which is of type `VI_Calibration`.
  msg.calibration.T_SC = Tsc_;
  msg.calibration.cam_model = covins::eCamModel::PINHOLE; // Use covins::eCamModel
  msg.calibration.dist_model = covins::eDistortionModel::RADTAN; // Use covins::eDistortionModel

  covins::TypeDefs::precision_t fx = K_.at<float>(0,0);
  covins::TypeDefs::precision_t fy = K_.at<float>(1,1);
  covins::TypeDefs::precision_t cx = K_.at<float>(0,2);
  covins::TypeDefs::precision_t cy = K_.at<float>(1,2);
  covins::TypeDefs::precision_t k1 = DistCoef_.at<float>(0);
  covins::TypeDefs::precision_t k2 = DistCoef_.at<float>(1);
  covins::TypeDefs::precision_t p1 = DistCoef_.at<float>(2);
  covins::TypeDefs::precision_t p2 = DistCoef_.at<float>(3);

  // Use TypeDefs::DynamicVectorType as defined in typedefs_base.hpp
  covins::TypeDefs::DynamicVectorType dist_coeffs_vec;
  dist_coeffs_vec.resize(4); // Or DistCoef_.rows if it can vary
  dist_coeffs_vec << k1, k2, p1, p2;

  if (is_fisheye_) {
    // For fisheye, the dist_coeffs_vec needs to be appropriate for fisheye model (often 4 or 5 coeffs)
    // Here we reset it to zeros if that's the intended behavior for the message, but it should
    // ideally reflect the actual fisheye distortion coefficients.
    dist_coeffs_vec.setZero(4); // Assuming 4 coeffs for fisheye if resetting to zero.
    cv::Size size = {img.cols, img.rows};
    cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    cv::Mat map1;
    cv::Mat map2;
    cv::fisheye::initUndistortRectifyMap(K_, DistCoef_, E, K_, size, CV_16SC2, map1, map2);

    cv::Mat undistort_img; // Use a distinct name to avoid overwriting 'img' prematurely
    cv::remap(img, undistort_img, map1, map2, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

    img = undistort_img; // Assign undistorted image back to img
 }

  // Assign the full VI_Calibration struct to msg.calibration
  // Pass dist_coeffs_vec, not DistCoef_ (which is cv::Mat)
  msg.calibration = covins::VI_Calibration(
      Tsc_,
      msg.calibration.cam_model, // Use the model already set above
      msg.calibration.dist_model, // Use the distortion model already set above
      dist_coeffs_vec, // Use the Eigen vector for distortion coefficients
      (double)img.cols, (double)img.rows, // Ensure img dimensions are doubles
      fx, fy, cx, cy,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81, // Default IMU noise/bias, gravity values
      Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0 // Default time offset, min/max preint time
  );

  msg.img_dim_x_min = 0.0;
  msg.img_dim_y_min = 0.0;
  msg.img_dim_x_max = img.cols;
  msg.img_dim_y_max = img.rows;

  msg.keypoints_aors.reserve(n_feat_pr_);
  msg.keypoints_distorted.reserve(n_feat_pr_);
  msg.keypoints_undistorted.reserve(n_feat_pr_);
  // Ensure descriptors are cleared before reserving to avoid issues with previous data if not needed
  // msg.descriptors.release(); // cv::Mat manages its own memory, but useful to clear if not explicitly handled.
  // msg.descriptors.reserve(n_feat_pr_); // cv::Mat does not have reserve like std::vector

  msg.keypoints_aors_add.reserve(n_feat_);
  msg.keypoints_distorted_add.reserve(n_feat_);
  msg.keypoints_undistorted_add.reserve(n_feat_);
  // msg.descriptors_add.release();
  // msg.descriptors_add.reserve(n_feat_);


  //Place Recognition Features
  std::vector<cv::KeyPoint> cv_keypoints;
  cv_keypoints.reserve(n_feat_pr_);

  cv::Mat new_descriptors;
  (*orb_extractor_PR_)(img, cv::Mat(), cv_keypoints, new_descriptors);

  for(size_t i=0;i<cv_keypoints.size();++i) {
      covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
      aors << cv_keypoints[i].angle, static_cast<float>(cv_keypoints[i].octave),
          cv_keypoints[i].response, cv_keypoints[i].size;

      covins::TypeDefs::KeypointType kp_eigen;
      kp_eigen[0] = static_cast<float>(cv_keypoints[i].pt.x);
      kp_eigen[1] = static_cast<float>(cv_keypoints[i].pt.y);
      msg.keypoints_aors.push_back(aors);
      msg.keypoints_distorted.push_back(kp_eigen);

  }
  msg.descriptors = new_descriptors.clone();


  // Features to be used for Pose Estimation
  std::vector<cv::KeyPoint> cv_keypoints_add;
  cv_keypoints_add.reserve(n_feat_);
  cv::Mat new_descriptors_add;

  if (fType_ == "ORB") {
    (*orb_extractor_)(img, cv::Mat(), cv_keypoints_add, new_descriptors_add);
  } else if (fType_ == "SIFT") {
    sift_detector_->detectAndCompute(img, cv::Mat(), cv_keypoints_add,
                                     new_descriptors_add);
  } else {
    COUTERROR << " Feature Type Not Supported: Select ORB or SIFT" << std::endl;
    exit(-1);
  }

  for(size_t i=0;i<cv_keypoints_add.size();++i) {
      covins::TypeDefs::AorsType aors; //Angle,Octave,Response,Size
      aors << cv_keypoints_add[i].angle, static_cast<float>(cv_keypoints_add[i].octave), cv_keypoints_add[i].response, cv_keypoints_add[i].size;

      covins::TypeDefs::KeypointType kp_eigen;
      kp_eigen[0] = static_cast<float>(cv_keypoints_add[i].pt.x);
      kp_eigen[1] = static_cast<float>(cv_keypoints_add[i].pt.y);
      msg.keypoints_aors_add.push_back(aors);
      msg.keypoints_distorted_add.push_back(kp_eigen);
  }

    msg.descriptors_add = new_descriptors_add.clone();

  msg.T_s_c = Tsc_;
  msg.lin_acc = covins::TypeDefs::Vector3Type::Zero();
  msg.ang_vel = covins::TypeDefs::Vector3Type::Zero();

  TransformType T_w_sref = TransformType::Identity();
  TransformType T_w_s = TransformType::Identity();

  if (is_odom_imu_frame_) {
    T_w_sref = T_wc_prev;
    T_w_s = T_wc;
  } else {
    // If in Camera frame, convert it to IMU frame before sending to backend
    T_w_sref = T_wc_prev * Tsc_.inverse();
    T_w_s = T_wc * Tsc_.inverse();
  }

  msg.T_sref_s = T_w_sref.inverse() * T_w_s;

  // Assuming KFRANGE and MAPRANGE are defined elsewhere as integer constants
  // For now, using default pair values.
  int KFRANGE_VAL = -1; // Placeholder if not defined as macro/const
  int MAPRANGE_VAL = -1; // Placeholder if not defined as macro/const

  if (index == 0)
    msg.id_predecessor = std::make_pair(KFRANGE_VAL, MAPRANGE_VAL); // Use std::make_pair
  else {
    msg.id_predecessor.first = index-1;
    msg.id_predecessor.second = client_id;
    msg.id_reference.first = index-1;
    msg.id_reference.second = client_id;
  }
  msg.id_successor = std::make_pair(KFRANGE_VAL, MAPRANGE_VAL); // Use std::make_pair
}


/**
 * @brief Generic callback function for messages received from the abstraction layer.
 *
 * This function is called when any IMessage object is received from the
 * ICommunicator. It identifies the message type (Image, Odometry, etc.),
 * stores it in a buffer, and then triggers the synchronization logic.
 *
 * @param received_message A unique_ptr to the received IMessage object.
 *
 * @return void
 */
auto FrontendWrapper::genericMessageCallback(std::unique_ptr<IMessage> received_message) -> void {
  if (!received_message) {
      COUTERROR << " Received null message in genericMessageCallback." << std::endl;
      return;
  }

  std::lock_guard<std::mutex> lock(buffer_mutex_); // Protect access to buffers

  // Process incoming messages and push them into the correct buffer
  if (received_message->getType() == "Image") {
      // Assuming MsgImage is correctly derived from IMessage and has a valid move constructor/assignment
      image_buffer_.push_back(std::unique_ptr<MsgImage>(static_cast<MsgImage*>(received_message.release())));
  } else if (received_message->getType() == "Odometry") {
      // Assuming MsgOdometry is correctly derived from IMessage
      odom_buffer_.push_back(std::unique_ptr<MsgOdometry>(static_cast<MsgOdometry*>(received_message.release())));
  } else {
      COUTERROR << " Unexpected message type received: " << received_message->getType() << std::endl;
      return;
  }

  // Attempt to process synchronized messages immediately after a new message arrives
  processSynchronizedMessages();
}

/**
 * @brief Processes messages from internal buffers to find and handle synchronized pairs.
 *
 * This function implements the approximate time synchronization logic. It iterates
 * through the image and odometry buffers, looking for pairs whose timestamps are
 * within the configured sync_tolerance_. Old, unsynchronized messages are discarded.
 *
 * NOTE: This function assumes `buffer_mutex_` is already locked by the caller.
 *
 * @return void
 */
auto FrontendWrapper::processSynchronizedMessages() -> void {
    while (!image_buffer_.empty() && !odom_buffer_.empty()) {
        const auto& img_msg = image_buffer_.front();
        const auto& odom_msg = odom_buffer_.front();

        double img_ts = img_msg->getTimestamp();
        double odom_ts = odom_msg->getTimestamp();

        // Check for synchronization based on tolerance
        if (std::abs(img_ts - odom_ts) < sync_tolerance_) {
            // Found a synchronized pair! Process it.
            cv::Mat img = img_msg->getImage();
            Eigen::Matrix4d T_wc_eigen = odom_msg->getTransform();

            // Check if image is valid before processing
            if (img.empty()) {
                COUTERROR << " Received empty image in synchronized processing. Discarding pair." << std::endl;
                image_buffer_.pop_front(); // Discard problematic image
                odom_buffer_.pop_front();  // Discard corresponding odometry
                continue; // Try next pair
            }

            // --- Continue with original Keyframe Generation Logic ---
            curr_pos_ = T_wc_eigen.block<3,1>(0,3);
            curr_quat_ = Eigen::Quaterniond(T_wc_eigen.block<3,3>(0,0));

            auto quat_ang = curr_quat_.angularDistance(prev_quat_);
            auto trans_diff = (curr_pos_ - prev_pos_).norm();

            TransformType T_wc = T_wc_eigen;
            TransformType T_wc_prev = Twc_prev_;

            // Use curr_ts_ (member variable) instead of undeclared curr_ts
            if (trans_diff > t_min_ || quat_ang > r_min_) {
                COUTINFO << "Generated New KF with id: " << kf_count_<< std::endl;
                MsgKeyframe msg_kf;
                this->convertToMsg(msg_kf, img, T_wc, T_wc_prev, client_id_, kf_count_, curr_ts_); // FIX: Use curr_ts_

                // Send MsgKeyframe via the abstract communicator
                if (comm_ && comm_->isConnected()) {
                    comm_->send(msg_kf);
                } else {
                    COUTERROR << " Communicator not connected, cannot send keyframe." << std::endl;
                }

                prev_quat_ = curr_quat_;
                prev_pos_ = curr_pos_;
                Twc_prev_ = T_wc; // Update Twc_prev_ with the current T_wc
                prev_ts_ = curr_ts_;
                kf_count_++;
            }
            // --- End Existing Keyframe Generation Logic ---

            // Pop processed messages from buffers
            image_buffer_.pop_front();
            odom_buffer_.pop_front();

        } else if (img_ts < odom_ts - sync_tolerance_) {
            // Image is too old, discard it as it's too far before the current odometry
            COUTINFO << "Discarding old image (TS: " << img_ts << ") as it's too far before odometry (TS: " << odom_ts << ")" << std::endl;
            image_buffer_.pop_front();
        } else if (odom_ts < img_ts - sync_tolerance_) {
            // Odometry is too old, discard it as it's too far before the current image
            COUTINFO << "Discarding old odometry (TS: " << odom_ts << ") as it's too far before image (TS: " << img_ts << ")" << std::endl;
            odom_buffer_.pop_front();
        } else {
            // No synchronized pair found at the front of buffers, and neither is significantly too old.
            // Wait for more messages to arrive.
            break;
        }
    }
}


// ParseCamParamFile and ParseORBParamFile remain largely unchanged,
// but any ROS_INFO/ERROR calls inside them should be updated to std::cout/cerr.
bool FrontendWrapper::ParseCamParamFile(cv::FileStorage &fSettings)
{
    DistCoef_ = cv::Mat::zeros(4,1,CV_32F);
    COUTINFO << std::endl << "Camera Parameters: " << std::endl;
    bool b_miss_params = false;

    is_odom_imu_frame_ = int(fSettings["odom_in_imu_frame"]);

    is_fisheye_ = int(fSettings["is_fisheye"]);

    cv::FileNode node = fSettings["t_min"];

    if(!node.empty())
    {
        t_min_ = node.real();
    }

    node = fSettings["r_min"];
    if(!node.empty())
    {
        r_min_ = node.real();
    }


    std::string sCameraName = fSettings["Camera.type"];

    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        // Camera calibration parameters

        cv::FileNode node = fSettings["Camera.fx"];

        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            COUTERROR << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            COUTERROR << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            COUTERROR << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            COUTERROR << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(0) = node.real();
        }
        else
        {
            COUTERROR << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(1) = node.real();
        }
        else
        {
            COUTERROR << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(2) = node.real();
        }
        else
        {
            COUTERROR << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.at<float>(3) = node.real();
        }
        else
        {
            COUTERROR << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            DistCoef_.resize(5);
            DistCoef_.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        COUTINFO << "- Camera: Pinhole" << std::endl;
        COUTINFO << "- fx: " << fx << std::endl;
        COUTINFO << "- fy: " << fy << std::endl;
        COUTINFO << "- cx: " << cx << std::endl;
        COUTINFO << "- cy: " << cy << std::endl;
        COUTINFO << "- k1: " << DistCoef_.at<float>(0) << std::endl;
        COUTINFO << "- k2: " << DistCoef_.at<float>(1) << std::endl;

        COUTINFO << "- p1: " << DistCoef_.at<float>(2) << std::endl;
        COUTINFO << "- p2: " << DistCoef_.at<float>(3) << std::endl;

        if(DistCoef_.rows==5)
            COUTINFO << "- k3: " << DistCoef_.at<float>(4) << std::endl;

        K_ = cv::Mat::eye(3,3,CV_32F);
        K_.at<float>(0,0) = fx;
        K_.at<float>(1,1) = fy;
        K_.at<float>(0,2) = cx;
        K_.at<float>(1,2) = cy;
    }

    else
    {
        COUTERROR << "*Not Supported Camera Sensor*" << std::endl;
        COUTERROR << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(b_miss_params)
    {
        return false;
    }

    // Params for Imu_Cam Transform

    cv::Mat Tbc;
    node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            COUTERROR << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        COUTERROR << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    COUTINFO << std::endl;
    COUTINFO << "Left camera to Imu Transform (Tbc): " << std::endl << Tbc << std::endl;

    cv::cv2eigen(Tbc, Tsc_);
    // Initialize with Tsc_ but these will be updated by the first incoming odometry message
    prev_pos_ = Tsc_.block<3, 1>(0, 3); // Position part of Tsc_
    Eigen::Quaterniond temp_quat(Tsc_.block<3,3>(0,0));
    prev_quat_ = temp_quat;
    Twc_prev_ = Tsc_; // Initialize previous world to camera transform
    return true;
}

bool FrontendWrapper::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nFeaturesPR, nLevels, fIniThFAST, fMinThFAST;
    std::string fType;
    float fScaleFactor;

    cv::FileNode node = fSettings["extractor.type"];
    if(!node.empty() && node.isString())
    {
        fType = node.operator std::string();
    }
    else
    {
        COUTERROR << "*extractor.type parameter doesn't exist or is not a string*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["extractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        COUTERROR << "*extractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nFeaturesPR"];
    if(!node.empty() && node.isInt())
    {
        nFeaturesPR = node.operator int();
    }
    else
    {
        COUTERROR << "*ORBextractor.nFeaturesPR parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        COUTERROR << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        COUTERROR << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        COUTERROR << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        COUTERROR << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    orb_extractor_.reset(new covins::ORBextractor(
        nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
    orb_extractor_PR_.reset(new covins::ORBextractor(
        nFeaturesPR, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));

    orb_detector_ = cv::ORB::create(nFeatures);
    sift_detector_ = cv::xfeatures2d::SIFT::create(nFeatures);

    COUTINFO << std::endl << "Extractor Parameters: " << std::endl;
    COUTINFO << "Feature Type: " << fType << std::endl;
    COUTINFO << "- Number of Features: " << nFeatures << std::endl;
    COUTINFO << "- Number of Features Place Rec: " << nFeaturesPR << std::endl;
    COUTINFO << "- Scale Levels: " << nLevels << std::endl;
    COUTINFO << "- Scale Factor: " << fScaleFactor << std::endl;
    COUTINFO << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    COUTINFO << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    n_feat_ = nFeatures;
    n_feat_pr_ = nFeaturesPR;
    fType_ = fType;

    return true;
}


} // namespace covins
