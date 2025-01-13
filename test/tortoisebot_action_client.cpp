#include "tortoisebot_action_client.hpp"

#include "rclcpp/logging.hpp"

#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

using namespace TortoisebotWaypoints;
using GoalHandleSharedPtr =
    TortoisebotActionClient::GoalHandleWaypointAction::SharedPtr;
using WaypointActionWrappedResultFuture = std::shared_future<
    TortoisebotActionClient::GoalHandleWaypointAction::WrappedResult>;

TortoisebotActionClient::TortoisebotActionClient(
    const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node{node_name, options},
      action_client_{
          rclcpp_action::create_client<WaypointAction>(this, kActionName)} {
  RCLCPP_INFO(this->get_logger(), "Started %s action client.", kActionName);
}

std::optional<WaypointActionWrappedResultFuture>
TortoisebotActionClient::send_goal(const WaypointAction::Goal &goal) {
  using namespace std::placeholders;

  this->action_client_->wait_for_action_server(); // forever

  RCLCPP_INFO(this->get_logger(), "Sending goal: (x=%f, y=%f, theta=%f).",
              goal.position.x, goal.position.y, goal.yaw);
  auto send_goal_options{
      rclcpp_action::Client<WaypointAction>::SendGoalOptions()};
  send_goal_options.goal_response_callback =
      std::bind(&TortoisebotActionClient::goal_response_callback, this, _1);
  /* send_goal_options.feedback_callback =
      std::bind(&TortoisebotActionClient::feedback_callback, this, _1, _2);*/
  send_goal_options.result_callback =
      std::bind(&TortoisebotActionClient::result_callback, this, _1);

  auto goal_handle_future{
      this->action_client_->async_send_goal(goal, send_goal_options)};
  if (!goal_handle_future.get()) {
    return std::nullopt;
  }

  return action_client_->async_get_result(goal_handle_future.get());
}

void TortoisebotActionClient::goal_response_callback(
    std::shared_future<GoalHandleWaypointAction::SharedPtr> future) {
  auto goal_handle{future.get()};
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result.");
  }
}

void TortoisebotActionClient::feedback_callback(
    GoalHandleWaypointAction::SharedPtr,
    const std::shared_ptr<const WaypointAction::Feedback> feedback) {

  std::stringstream ss;
  ss << "Feedback received (x=" << feedback->position.x << ", "
     << "y=" << feedback->position.y << ", "
     << "yaw=" << feedback->yaw << "state=" << feedback->state << ").";

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void TortoisebotActionClient::result_callback(
    const GoalHandleWaypointAction::WrappedResult &result) {

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(),
                result.result->success ? "succeeded" : "failed");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
    break;
  }
}
