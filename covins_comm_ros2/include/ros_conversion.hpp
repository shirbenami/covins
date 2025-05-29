#pragma once

#include "../covins_comm/typedefs_base.hpp"
#include <covins_msgs/msg/keyframe.hpp>
#include <covins_msgs/msg/landmark.hpp>
#include <covins_msgs/msg/data_bundle.hpp>

namespace covins {

// Converts a MsgKeyframe to a ROS2 Keyframe message
covins_msgs::msg::Keyframe ToRosMsg(const MsgKeyframe& src);

// Converts a MsgLandmark to a ROS2 Landmark message
covins_msgs::msg::Landmark ToRosMsg(const MsgLandmark& src);

// Converts a data_bundle to a ROS2 DataBundle message
covins_msgs::msg::DataBundle ToRosMsg(const data_bundle& src);

} // namespace covins