#pragma once

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#include <memory>
#include <mutex>
#include <thread>

namespace TortoisebotWaypoints {

class TortoisebotActionServer : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ServerGoalHandle<WaypointAction>;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

  TortoisebotActionServer();
  ~TortoisebotActionServer();

private:
  // Settings

  static constexpr double kPi{3.1416};
  static constexpr char kNodeName[]{"TortoisebotActionServer"};
  static constexpr char kOdomTopicName[]{"/odom"};
  static constexpr char kCmdVelTopicName[]{"cmd_vel"};
  static constexpr char kActionName[]{"tortoisebot_as"};

  double yaw_precision_{kPi / 90.0}; // +/- 2 degrees allowed
  double dist_precision_{0.05};
  double max_angular_vel_error_yaw_{20*yaw_precision_};  // [rad]
  double max_speed_error_pos_{5*dist_precision_};  // [m]
  double max_angular_vel_{0.65};  // [rad/s]
  double max_speed_{0.3};  // [m/s]
  double loop_rate_{130};  // [Hz]

  // Callbacks

  void odom_callback(const std::shared_ptr<const Odometry> msg);

  // Goal Handlers

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  // Helper
  void cleanup_goal_threads();

  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Rate rate_{loop_rate_};
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;

  // State
  geometry_msgs::msg::Point position_{};
  double yaw_{};
  mutable std::mutex state_mutex_{};
  std::shared_ptr<GoalHandleWaypointAction> current_goal_handle_{};
  std::thread goal_thread_;
};

} // namespace TortoisebotWaypoints
