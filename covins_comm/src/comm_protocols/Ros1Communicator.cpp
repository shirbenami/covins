//
// Created by user1 on 08/06/25.
//
#include <covins/comm_protocols/Ros1Communicator.hpp>
#include <covins/comm_serialization/ProtobufSerializer.hpp> // Example: assuming Protobuf is used for internal IMessage serialization
#include <covins/comm_messages/MsgImage.hpp>     // Concrete Image message type
#include <covins/comm_messages/MsgOdometry.hpp>  // Concrete Odometry message type
#include <covins/comm_messages/MsgKeyframe.hpp>  // Concrete Keyframe message type

#include <iostream> // For standard output/error, replace with ROS_INFO/ERROR in actual code

namespace covins {

Ros1Communicator::Ros1Communicator(const std::string& address, int port)
    : nh_("~"), it_(nh_), img_sub_(nullptr), odom_sub_(nullptr), sync_(nullptr),
      stop_ros_spin_thread_(false) {
    // ROS_INFO("Ros1Communicator constructor called. Address: %s, Port: %d", address.c_str(), port);
    // Note: For a typical ROS client, the address/port are often set via ROS_MASTER_URI
    // environment variable or passed through launch files, not directly here.
    // This constructor primarily prepares the member variables.
}

Ros1Communicator::~Ros1Communicator() {
    disconnect();
    // ROS_INFO("Ros1Communicator destructor called.");
}

bool Ros1Communicator::connect(const std::string& address, int port) {
    if (!ros::isInitialized()) {
        std::cerr << "ERROR: ROS is not initialized. Please ensure ros::init() is called." << std::endl;
        return false;
    }

    // Initialize ROS subscribers with message filters
    img_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/image_raw", 10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/odometry/filtered", 10);

    // Initialize message synchronizer for image and odometry
    sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *img_sub_, *odom_sub_);
    sync_->registerCallback(boost::bind(&Ros1Communicator::synchronizedSensorCallback, this, _1, _2));

    // Initialize publisher for outgoing keyframes
    keyframe_pub_ = nh_.advertise<sensor_msgs::Image>("/covins/keyframe", 10); // Placeholder for now.
    // In a real scenario, you'd likely have a custom ROS message type for keyframes.
    // E.g., ros::Publisher keyframe_pub_ = nh_.advertise<covins_ros_msgs::KeyframeMsg>("/covins/keyframe", 10);


    // Start a separate thread to spin ROS callbacks
    if (!ros_spin_thread_) {
        ros_spin_thread_ = std::make_unique<std::thread>(&Ros1Communicator::rosSpinThreadLoop, this);
    }

    // ROS_INFO("Ros1Communicator connected and initialized ROS subscribers/publishers.");
    return true;
}

bool Ros1Communicator::disconnect() {
    stop_ros_spin_thread_ = true;
    if (ros_spin_thread_ && ros_spin_thread_->joinable()) {
        ros_spin_thread_->join();
    }

    if (sync_) {
        delete sync_;
        sync_ = nullptr;
    }
    if (img_sub_) {
        delete img_sub_;
        img_sub_ = nullptr;
    }
    if (odom_sub_) {
        delete odom_sub_;
        odom_sub_ = nullptr;
    }

    // ROS_INFO("Ros1Communicator disconnected.");
    return true;
}

bool Ros1Communicator::send(const IMessage& message) {
    // Dynamic cast to determine the concrete message type
    if (message.getType() == "Keyframe") {
        const auto& keyframe_msg = static_cast<const MsgKeyframe&>(message);
        // Convert MsgKeyframe to a ROS1 message type (e.g., a custom ROS message or a generic sensor_msgs::Image for image part)
        // This is a placeholder. You'd need a custom ROS message definition for CovinsKeyframe.
        sensor_msgs::Image ros_image_msg;
        // Example: Convert keyframe_msg.image to ros_image_msg using cv_bridge
        try {
            cv_bridge::CvImage cv_img;
            cv_img.header.stamp = ros::Time(keyframe_msg.timestamp_); // Assuming timestamp_ in MsgKeyframe
            cv_img.header.frame_id = "camera_frame";
            cv_img.encoding = sensor_msgs::image_encodings::MONO8; // Or BGR8 etc.
            cv_img.image = keyframe_msg.getImage(); // Assuming MsgKeyframe has a getImage() that returns cv::Mat
            ros_image_msg = *cv_img.toImageMsg();
        } catch (cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
            return false;
        }

        keyframe_pub_.publish(ros_image_msg); // Publish the converted ROS message
        // ROS_INFO("Published MsgKeyframe (as sensor_msgs::Image for now).");
        return true;
    }
    // Add other message types as needed (e.g., "MapData", "LoopClosure")
    else {
        std::cerr << "WARNING: Ros1Communicator cannot send unknown IMessage type: " << message.getType() << std::endl;
        return false;
    }
}

std::unique_ptr<IMessage> Ros1Communicator::receive() {
    // For a ROS-based system primarily using callbacks, this method might just
    // process one round of callbacks or return nullptr to signal async nature.
    // A more complex implementation might buffer messages if `receive_callback_` is not set.
    ros::spinOnce(); // Process any pending callbacks
    return nullptr; // Indicate that messages are primarily handled via callback
}

bool Ros1Communicator::isConnected() const {
    return ros::ok();
}

void Ros1Communicator::registerReceiveCallback(std::function<void(std::unique_ptr<IMessage>)> callback) {
    receive_callback_ = std::move(callback);
    // ROS_INFO("Receive callback registered for Ros1Communicator.");
}

void Ros1Communicator::synchronizedSensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                                 const nav_msgs::OdometryConstPtr& odom_msg) {
    if (!receive_callback_) {
        // std::cerr << "WARNING: No receive callback registered in Ros1Communicator." << std::endl;
        return;
    }

    // Convert ROS Image message to MsgImage
    std::unique_ptr<MsgImage> covins_img_msg = std::make_unique<MsgImage>();
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8); // Adjust encoding if needed
        covins_img_msg->setImage(cv_ptr->image); // Assuming MsgImage has a setImage method
        covins_img_msg->setTimestamp(img_msg->header.stamp.toSec()); // Assuming setTimestamp
        // Add other relevant image metadata (frame_id, etc.) if MsgImage supports it
    } catch (cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    // Convert ROS Odometry message to MsgOdometry
    std::unique_ptr<MsgOdometry> covins_odom_msg = std::make_unique<MsgOdometry>();
    covins_odom_msg->setTimestamp(odom_msg->header.stamp.toSec()); // Assuming setTimestamp
    // Convert geometry_msgs/PoseWithCovariance to Eigen::Matrix4d or similar
    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_wc(odom_msg->pose.pose.orientation.w,
                            odom_msg->pose.pose.orientation.x,
                            odom_msg->pose.pose.orientation.y,
                            odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d t_wc(odom_msg->pose.pose.position.x,
                         odom_msg->pose.pose.position.y,
                         odom_msg->pose.pose.position.z);
    T_wc.block<3,3>(0,0) = q_wc.toRotationMatrix();
    T_wc.block<3,1>(0,3) = t_wc;
    covins_odom_msg->setTransform(T_wc); // Assuming setTransform
    // Add velocity, covariance, etc. if MsgOdometry supports it

    // Invoke the registered callback with the new IMessage objects
    // Note: You would typically pass *one* aggregated message or pass them sequentially.
    // For simplicity, we'll pass the image then odometry. In a real system,
    // you might define a new IMessage type like `MsgSynchronizedSensorData` that
    // contains both image and odometry.
    receive_callback_(std::move(covins_img_msg));
    receive_callback_(std::move(covins_odom_msg));
}

void Ros1Communicator::rosSpinThreadLoop() {
    // ROS_INFO("ROS spin thread started.");
    ros::Rate loop_rate(100); // Process callbacks at 100 Hz
    while (ros::ok() && !stop_ros_spin_thread_) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ROS_INFO("ROS spin thread stopped.");
}

} // namespace covins
