#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "communicator.hpp"
#include "ORBextractor.h"
#include <covins/covins_base/config_comm.hpp>

namespace covins {

class FrontendWrapper {

public:
    using TransformType = TypeDefs::TransformType;

    FrontendWrapper();
    ~FrontendWrapper() {}

    void Init(const std::string& config_file);
    void ProcessFrame(const cv::Mat &img,
                      const Eigen::Vector3d &position,
                      const Eigen::Quaterniond &orientation,
                      double timestamp);

    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);

    void convertToMsg(covins::MsgKeyframe &msg, cv::Mat &img, TransformType T_wc,
                      TransformType T_wc_prev, int client_id, int index,
                      double ts);

protected:
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

    // COVINS integration
    covins::TypeDefs::ThreadPtr thread_comm_;
    std::shared_ptr<covins::Communicator> comm_;
    int client_id_;
};

}
