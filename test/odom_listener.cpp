#include "odom_listener.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <functional>
#include <string>

using namespace TortoisebotWaypoints;

// Helper functions

double compYaw(const geometry_msgs::msg::Quaternion &orientation) {
  tf2::Quaternion q{orientation.x, orientation.y, orientation.z, orientation.w};
  double yaw{}, pitch{}, roll{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

// Class members

OdomListener::OdomListener(const std::string &node_name,
                           const rclcpp::NodeOptions &options)
    : Node{node_name, options},
      odom_subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
          kOdomTopicName, 1,
          std::bind(&OdomListener::odom_callback, this,
                    std::placeholders::_1))} {
  RCLCPP_INFO(this->get_logger(), "Started %s node.", node_name.c_str());
}

void OdomListener::odom_callback(
    const std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  yaw_ = compYaw(msg->pose.pose.orientation);
  position_ = msg->pose.pose.position;
}
