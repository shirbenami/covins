#include "comm_ros2_bridge.hpp"
#include "ros_conversion.hpp"

namespace covins {

CommRos2Bridge::CommRos2Bridge(std::shared_ptr<rclcpp::Node> node) {
    ros2_pub_ = node->create_publisher<covins_msgs::msg::DataBundle>("covins_data_bundle", 10);
}

void CommRos2Bridge::PublishDataBundle(const covins::data_bundle& bundle) {
    covins_msgs::msg::DataBundle ros_msg = ToRosMsg(bundle);
    ros2_pub_->publish(ros_msg);
}

} // namespace covins