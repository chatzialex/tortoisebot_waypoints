#pragma once

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <optional>
#include <string>

namespace TortoisebotWaypoints {

class OdomListener : public rclcpp::Node {
public:
  OdomListener(const std::string &node_name = kNodeName,
               const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  const std::optional<geometry_msgs::msg::Point> &getPosition() {
    return position_;
  }
  const std::optional<double> &getYaw() { return yaw_; }

private:
  void odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  // State
  std::optional<geometry_msgs::msg::Point> position_{std::nullopt};
  std::optional<double> yaw_{std::nullopt};

  // Names
  static constexpr char kNodeName[]{"OdomListenerNode"};
  static constexpr char kOdomTopicName[]{"/odom"};
};

} // namespace TortoisebotWaypoints