#include "tortoisebot_action_client.hpp"

#include "rclcpp/logging.hpp"

#include <future>
#include <memory>
#include <sstream>

using namespace TortoisebotWaypoints;
using GoalHandleSharedPtr =
    TortoisebotActionClient::GoalHandleWaypointAction::SharedPtr;

TortoisebotActionClient::TortoisebotActionClient()
    : Node{kNodeName}, action_client_{
                           rclcpp_action::create_client<WaypointAction>(
                               this, kActionName)} {
  RCLCPP_INFO(this->get_logger(), "Started %s action client.", kActionName);
}

TortoisebotActionClient::WaypointAction::Result::SharedPtr
TortoisebotActionClient::send_goal(const WaypointAction::Goal &goal) {
  using namespace std::placeholders;

  is_goal_done_.store(false);

  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting.");
    return nullptr;
  }

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
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                     goal_handle_future);
  if (!goal_handle_future.get()) {
    return nullptr;
  }

  while (!is_goal_done_.load()) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
  
  return result_;
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
     << "y=" << feedback->position.y << ", " << "yaw=" << feedback->yaw
     << "state=" << feedback->state << ").";

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void TortoisebotActionClient::result_callback(
    const GoalHandleWaypointAction::WrappedResult &result) {

  result_=nullptr;

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), result.result->success ? "succeeded" : "failed");
    result_=result.result;
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

  is_goal_done_.store(true);
}
