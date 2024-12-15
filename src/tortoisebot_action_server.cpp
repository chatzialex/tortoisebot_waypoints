#include "tortoisebot_waypoints/tortoisebot_action_server.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>

using namespace TortoisebotWaypoints;

// Shift angle to [-pi, pi)
double normalize_angle(double angle) {
  constexpr double pi{3.1416};
  angle = std::fmod(angle + pi, 2 * pi);

  if (angle < 0) {
    angle += 2 * pi;
  }

  return angle - pi;
}

TortoisebotActionServer::TortoisebotActionServer()
    : Node{kNodeName}, cmd_vel_pub_{this->create_publisher<Twist>(
                           kCmdVelTopicName, 10)},
      odom_sub_{this->create_subscription<Odometry>(
          kOdomTopicName, 1,
          std::bind(&TortoisebotActionServer::odom_callback, this,
                    std::placeholders::_1))},
      action_server_{rclcpp_action::create_server<WaypointAction>(
          this, kActionName,
          std::bind(&TortoisebotActionServer::handle_goal, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&TortoisebotActionServer::handle_cancel, this,
                    std::placeholders::_1),
          std::bind(&TortoisebotActionServer::handle_accepted, this,
                    std::placeholders::_1))} {
  RCLCPP_INFO(this->get_logger(), "Started %s action server.", kActionName);
}

TortoisebotActionServer::~TortoisebotActionServer() {
  cleanup_goal_threads();
}

void TortoisebotActionServer::odom_callback(
    const std::shared_ptr<const Odometry> msg) {
  tf2::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  double yaw{};
  double pitch{};
  double roll{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  std::lock_guard<std::mutex> lock{state_mutex_};
  position_ = msg->pose.pose.position;
  yaw_ = yaw;
}

rclcpp_action::GoalResponse TortoisebotActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /* uuid */,
    std::shared_ptr<const WaypointAction::Goal> goal) {

  RCLCPP_INFO(this->get_logger(), "Goal received: (x=%f, y=%f, theta=%f).",
              goal->position.x, goal->position.y, goal->yaw);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TortoisebotActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointAction> /* goal_handle */) {
  RCLCPP_INFO(this->get_logger(), "Cancelling goal...");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TortoisebotActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_INFO(this->get_logger(), "Preempting current goal...");
    current_goal_handle_->abort(std::make_shared<WaypointAction::Result>());
  }

  current_goal_handle_ = goal_handle;

  cleanup_goal_threads();
  goal_thread_ = std::thread{
      std::bind(&TortoisebotActionServer::execute, this, std::placeholders::_1),
      goal_handle};
}

void TortoisebotActionServer::execute(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  // Extract goal
  auto goal{goal_handle->get_goal()};
  auto feedback{std::make_shared<WaypointAction::Feedback>()};
  auto result{std::make_shared<WaypointAction::Result>()};

  double err_pos{}, desired_yaw{}, err_yaw{};
  bool keep_looping{true};

  while (rclcpp::ok() && keep_looping) {
    // Compute errors
    {
      std::lock_guard<std::mutex> lock{state_mutex_};
      err_pos = std::sqrt(std::pow(goal->position.x - position_.x, 2) +
                          std::pow(goal->position.y - position_.y, 2));
      desired_yaw = err_pos > dist_precision_
                        ? std::atan2(goal->position.y - position_.y,
                                     goal->position.x - position_.x)
                        : goal->yaw;
      err_yaw = normalize_angle(desired_yaw - yaw_);
    }

    // Main loop
    if (err_pos <= dist_precision_ && std::fabs(err_yaw) <= yaw_precision_) {
      RCLCPP_INFO(this->get_logger(), "Reached goal: (x=%f, y=%f, theta=%f).",
                  goal->position.x, goal->position.y, goal->yaw);
      if (goal_handle->is_active()) {
        result->success = true;
        goal_handle->succeed(result);
      }
      feedback->state = "idle";
      keep_looping = false;
    } else if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Cancelled goal: (x=%f, y=%f, theta=%f).",
                  goal->position.x, goal->position.y, goal->yaw);
      result->success = false;
      goal_handle->canceled(result);
      feedback->state = "idle";
      keep_looping = false;
    } else if (!goal_handle->is_active()) {
      RCLCPP_INFO(this->get_logger(), "Aborted goal: (x=%f, y=%f, theta=%f).",
                  goal->position.x, goal->position.y, goal->yaw);
      result->success = false;
      feedback->state = "idle";
      keep_looping = false;
    } else if (std::fabs(err_yaw) > yaw_precision_) {
      // Fix yaw
      feedback->state = "fix yaw";
      Twist cmd_vel_msg{};
      cmd_vel_msg.angular.z = std::max(std::min(err_yaw*max_angular_vel_/max_angular_vel_error_yaw_, max_angular_vel_), -max_angular_vel_);
      cmd_vel_pub_->publish(cmd_vel_msg);
    } else {
      // Move towards point
      feedback->state = "go to point";
      Twist cmd_vel_msg{};
      cmd_vel_msg.angular.z = std::max(std::min(err_yaw*max_angular_vel_/max_angular_vel_error_yaw_, max_angular_vel_), -max_angular_vel_);
      cmd_vel_msg.linear.x = std::max(std::min(err_pos*max_speed_/max_speed_error_pos_, max_speed_), -max_speed_);
      cmd_vel_pub_->publish(cmd_vel_msg);
    }

    feedback->position = position_;
    feedback->yaw = yaw_;
    goal_handle->publish_feedback(feedback);

    rate_.sleep();
  }

  // Stop robot
  Twist stop_msg{};
  cmd_vel_pub_->publish(stop_msg);
}

void TortoisebotActionServer::cleanup_goal_threads() {
  if (goal_thread_.joinable()) {
    goal_thread_.join();
  }
}
