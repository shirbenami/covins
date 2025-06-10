#pragma once

// Removed ALL ROS1-specific includes for sensor input and message synchronization.

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>

#include "ORBextractor.h"
#include <covins/covins_base/config_comm.hpp>

// NEW ABSTRACTION LAYER INCLUDES:
#include <covins/comm_abstraction/ICommunicator.hpp>      // Defines the ICommunicator interface for sending/receiving data
#include <covins/comm_abstraction/IMessage.hpp>           // Defines the IMessage base class
#include <covins/comm_abstraction/CommunicatorFactory.hpp>// Factory to create ICommunicator instances (used in .cpp)
#include <covins/comm_serialization/ISerializer.hpp>      // For serialization/deserialization within IMessage

// Standard library includes for buffering and synchronization
#include <deque>    // For std::deque
#include <mutex>    // For std::mutex
#include <thread>   // For std::this_thread::sleep_for
#include <chrono>   // For std::chrono::milliseconds

// Forward declarations of concrete message types that will be handled by the buffers
namespace covins {
    class MsgImage;
    class MsgOdometry;
    class MsgKeyframe;
}

// ------------------

namespace covins {

class FrontendWrapper {

public:
  using TransformType = TypeDefs::TransformType;
  FrontendWrapper();
  ~FrontendWrapper(){};

  auto run() -> void;

  bool ParseCamParamFile(cv::FileStorage &fSettings);
  bool ParseORBParamFile(cv::FileStorage &fSettings);

  void convertToMsg(covins::MsgKeyframe &msg, cv::Mat &img, TransformType T_wc,
                    TransformType T_wc_prev, int client_id, int index,
                    double ts);

  // In the public section of FrontendWrapper class
  void sendKeyframeMessage(const MsgKeyframe& message);
  int getClientId() const { return client_id_; } // Add getter for client_id
  std::shared_ptr<ICommunicator> getCommunicator() const { return comm_; } // Add getter for communicator

  protected:
    // Generic callback for incoming messages from the abstraction layer.
    auto genericMessageCallback(std::unique_ptr<IMessage> received_message) -> void;

    // NEW: Buffers for incoming messages to allow for approximate synchronization
    std::deque<std::unique_ptr<MsgImage>> image_buffer_;
    std::deque<std::unique_ptr<MsgOdometry>> odom_buffer_;
    std::mutex buffer_mutex_; // Mutex to protect access to the buffers
    double sync_tolerance_;   // Tolerance for approximate synchronization (e.g., 0.05 seconds)

    // NEW: Private method to process messages from the buffers and perform synchronization
    auto processSynchronizedMessages() -> void;

    // Remaining member variables (unchanged from previous version)
    Eigen::Vector3d prev_pos_;
    Eigen::Vector3d curr_pos_;
    Eigen::Quaterniond prev_quat_;
    Eigen::Quaterniond curr_quat_;

    size_t kf_count_;
    double prev_ts_;
    double curr_ts_;
    cv::Mat K_;
    cv::Mat DistCoef_;
    bool is_fisheye_ = false;

    int n_feat_pr_;
    int n_feat_;
    std::string fType_;

    bool is_odom_imu_frame_ = false;
    double t_min_;
    double r_min_;

    TransformType Tsc_;
    TransformType Twc_;
    TransformType Twc_prev_;

    std::shared_ptr<covins::ORBextractor> orb_extractor_;
    std::shared_ptr<covins::ORBextractor> orb_extractor_PR_;
    cv::Ptr<cv::xfeatures2d::SIFT> sift_detector_;
    cv::Ptr<cv::ORB> orb_detector_;

    covins::TypeDefs::ThreadPtr thread_comm_;
    std::shared_ptr<covins::ICommunicator> comm_;
    int client_id_;

};

}
