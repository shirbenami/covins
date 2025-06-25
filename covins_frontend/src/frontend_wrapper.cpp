#include "frontend_wrapper.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace covins {

FrontendWrapper::FrontendWrapper() {
    // Set defaults
    prev_pos_ = Eigen::Vector3d::Zero();
    curr_pos_ = Eigen::Vector3d::Zero();
    prev_quat_ = Eigen::Quaterniond::Identity();
    curr_quat_ = Eigen::Quaterniond::Identity();
    Twc_prev_ = TransformType::Identity();
    prev_ts_ = 0;
    curr_ts_ = 0;
    kf_count_ = 0;
}

void FrontendWrapper::Init(const std::string& config_file) {
    cv::FileStorage fSettings(config_file, cv::FileStorage::READ);

    if (!ParseCamParamFile(fSettings)) {
        std::cerr << "*Error with the camera parameters in the config file*" << std::endl;
    }

    if (!ParseORBParamFile(fSettings)) {
        std::cerr << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    covins_params::ShowParamsComm();
    comm_ = std::make_shared<covins::Communicator>(covins_params::GetServerIP(), covins_params::GetPort());
    thread_comm_ = std::make_shared<std::thread>(&covins::Communicator::Run, comm_);

    while (comm_->GetClientId() < 0) {
        usleep(1000);
    }
    client_id_ = comm_->GetClientId();
    std::cout << "received client ID: " << client_id_ << std::endl;
}

void FrontendWrapper::ProcessFrame(const cv::Mat &img_in,
                                   const Eigen::Vector3d &position,
                                   const Eigen::Quaterniond &orientation,
                                   double timestamp) {
    curr_pos_ = position;
    curr_quat_ = orientation;
    curr_ts_ = timestamp;

    double quat_ang = curr_quat_.angularDistance(prev_quat_);
    double trans_diff = (curr_pos_ - prev_pos_).norm();

    if (trans_diff <= t_min_ && quat_ang <= r_min_) return;

    std::cout << "Generated New KF with id: " << kf_count_ << std::endl;

    TransformType T_wc = TransformType::Identity();
    TransformType T_wc_prev = TransformType::Identity();

    T_wc.block<3, 1>(0, 3) = curr_pos_;
    T_wc.block<3, 3>(0, 0) = curr_quat_.toRotationMatrix();

    T_wc_prev.block<3, 1>(0, 3) = prev_pos_;
    T_wc_prev.block<3, 3>(0, 0) = prev_quat_.toRotationMatrix();

    cv::Mat img = img_in.clone();

    data_bundle map_chunk;
    MsgKeyframe msg_kf;
    convertToMsg(msg_kf, img, T_wc, T_wc_prev, client_id_, kf_count_, curr_ts_);
    map_chunk.keyframes.push_back(msg_kf);
    comm_->PassDataBundle(map_chunk);

    prev_quat_ = curr_quat_;
    prev_pos_ = curr_pos_;
    Twc_prev_ = Twc_;
    prev_ts_ = curr_ts_;
    kf_count_++;
}

}  // namespace covins
