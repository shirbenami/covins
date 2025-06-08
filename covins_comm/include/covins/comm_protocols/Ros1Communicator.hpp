//
// Created by user1 on 08/06/25.
//

#ifndef ROS1COMMUNICATOR_HPP
#define ROS1COMMUNICATOR_HPP

#pragma once

// ROS1 specific includes for communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

// ROS1 message filters for synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Core abstraction layer interfaces
#include <covins/comm_abstraction/ICommunicator.hpp>
#include <covins/comm_abstraction/IMessage.hpp> // For handling generic incoming/outgoing messages
#include <covins/comm_abstraction/ISerializer.hpp> // For serializing IMessage objects to bytes

// OpenCV (for image processing within callbacks)
#include <opencv2/core/core.hpp>

// Other necessary standard library includes
#include <string>
#include <memory>
#include <functional> // For std::function

namespace covins {

// Forward declarations of concrete message types that Ros1Communicator will handle.
// These need to be defined elsewhere and inherit from IMessage.
class MsgImage;
class MsgOdometry;
class MsgKeyframe;

/**
 * @brief Concrete implementation of ICommunicator for ROS1 communication.
 *
 * This class acts as an adapter, translating generic ICommunicator calls
 * into ROS1-specific publishing and subscribing operations. It handles
 * the conversion between ROS1 messages (e.g., sensor_msgs::Image, nav_msgs::Odometry)
 * and the abstract IMessage types (e.g., MsgImage, MsgOdometry, MsgKeyframe).
 */
class Ros1Communicator : public ICommunicator {
public:
    /**
     * @brief Constructor for Ros1Communicator.
     * @param address The ROS master URI or hostname (often not directly used for client nodes).
     * @param port The ROS master port (often not directly used for client nodes).
     */
    Ros1Communicator(const std::string& address, int port);

    /**
     * @brief Destructor. Cleans up ROS resources.
     */
    virtual ~Ros1Communicator();

    /**
     * @brief Connects to the ROS1 system.
     * Initializes ROS node, publishers, subscribers, and message filters.
     * @param address Ignored for typical ROS1 client setup.
     * @param port Ignored for typical ROS1 client setup.
     * @return True if ROS node and communicators are initialized successfully.
     */
    bool connect(const std::string& address, int port) override;

    /**
     * @brief Disconnects from the ROS1 system.
     * Shuts down ROS node and cleans up resources.
     * @return True if disconnection was successful.
     */
    bool disconnect() override;

    /**
     * @brief Sends an IMessage object via ROS1.
     * This method will cast the IMessage to a concrete type (e.g., MsgKeyframe),
     * convert it to the corresponding ROS1 message, and publish it.
     * @param message The IMessage object to send.
     * @return True if the message was successfully published.
     */
    bool send(const IMessage& message) override;

    /**
     * @brief Receives an IMessage object from the ROS1 system (blocking/polling).
     *
     * In ROS1, this method is typically less efficient as callbacks are preferred.
     * It might block until a message is received or return nullptr if no message.
     * For a ROS callback-driven system, this often calls ros::spinOnce() or a dedicated thread.
     * For this design, we will primarily rely on the registered callback.
     *
     * @return A unique_ptr to the received IMessage object, or nullptr.
     */
    std::unique_ptr<IMessage> receive() override;

    /**
     * @brief Checks if the ROS1 node is running and active.
     * @return True if connected and ROS is operational.
     */
    bool isConnected() const override;

    /**
     * @brief Registers a callback function for asynchronous message reception.
     * The provided callback will be invoked when new messages are received from ROS topics.
     * @param callback The function to call with received IMessage objects.
     */
    void registerReceiveCallback(std::function<void(std::unique_ptr<IMessage>)> callback) override;

private:
    // --- ROS1 Specific Members ---
    ros::NodeHandle nh_; // Main ROS node handle
    image_transport::ImageTransport it_; // For image publishing/subscribing

    // ROS1 Subscribers for incoming sensor data
    // These will receive raw ROS messages and trigger conversion to IMessage
    message_filters::Subscriber<sensor_msgs::Image>* img_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;

    // ROS1 Message Filters for time synchronization of image and odometry
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy>* sync_;

    // ROS1 Publishers for outgoing messages (e.g., keyframes to backend)
    ros::Publisher keyframe_pub_; // Publisher for MsgKeyframe, converted to ROS msg type

    // Internal Callback from message_filters
    // This method is called when synchronized image and odometry messages are available
    void synchronizedSensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                    const nav_msgs::OdometryConstPtr& odom_msg);

    // Stored callback for general incoming IMessage objects
    std::function<void(std::unique_ptr<IMessage>)> receive_callback_;

    // Thread for ROS spinning if needed for non-blocking operation
    std::unique_ptr<std::thread> ros_spin_thread_;
    void rosSpinThreadLoop(); // The function to run in the spin thread
    bool stop_ros_spin_thread_; // Flag to signal thread to stop

    // To store pending messages if receive() is called and callback is not set,
    // or if the internal processing of the callback needs buffering.
    // For this design, we primarily use the callback, so this might be minimal.
    // std::queue<std::unique_ptr<IMessage>> message_buffer_;
    // std::mutex buffer_mutex_;
    // std::condition_variable buffer_cv_;
};

} // namespace covins

#endif //ROS1COMMUNICATOR_HPP
