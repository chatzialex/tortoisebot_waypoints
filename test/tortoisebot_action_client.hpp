#pragma once

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "tortoisebot_waypoints/action/detail/waypoint_action__struct.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <functional>
#include <future>
#include <memory>

namespace TortoisebotWaypoints {

class TortoisebotActionClient : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ClientGoalHandle<WaypointAction>;

  TortoisebotActionClient();

  std::shared_future<GoalHandleWaypointAction::SharedPtr>
  send_goal(const WaypointAction::Goal &goal);

private:
  void goal_response_callback(
      std::shared_future<GoalHandleWaypointAction::SharedPtr> future);

  void feedback_callback(
      GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback);

  void result_callback(const GoalHandleWaypointAction::WrappedResult &result);

  static constexpr char kNodeName[]{"TortoisebotActionClient"};
  static constexpr char kActionName[]{"tortoisebot_as"};

  rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
}; // class TortoisebotActionClient

} // namespace TortoisebotWaypoints