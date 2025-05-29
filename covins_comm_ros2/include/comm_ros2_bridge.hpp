#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <covins_msgs/msg/data_bundle.hpp>
#include <covins/covins_base/msgs/msg_keyframe.hpp>
#include <covins/covins_base/msgs/msg_landmark.hpp>
#include <covins/covins_base/communicator_base.hpp>

namespace covins {

class CommRos2Bridge {
public:
    explicit CommRos2Bridge(std::shared_ptr<rclcpp::Node> node);

    void PublishDataBundle(const covins::data_bundle& bundle);

private:
    rclcpp::Publisher<covins_msgs::msg::DataBundle>::SharedPtr ros2_pub_;
};

} // namespace covins