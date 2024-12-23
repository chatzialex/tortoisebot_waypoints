#include "rclcpp/node.hpp"
#include "tortoisebot_action_client.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "tortoisebot_waypoints/tortoisebot_action_server.hpp"
#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <thread>

using namespace TortoisebotWaypoints;
using Odometry = nav_msgs::msg::Odometry;
using Quaternion = geometry_msgs::msg::Quaternion;
using Point = geometry_msgs::msg::Point;
using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using WaypointActionResultFuture = std::shared_future<
    TortoisebotActionClient::GoalHandleWaypointAction::SharedPtr>;

// Helper functions

double compErrorPosition(const Point &p1, const Point &p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

double compYaw(const Quaternion &orientation) {
  tf2::Quaternion q{orientation.x, orientation.y, orientation.z, orientation.w};
  double yaw{}, pitch{}, roll{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

// Shift angle to [-pi, pi)
double normalize_angle(double angle) {
  constexpr double pi{3.1416};
  angle = std::fmod(angle + pi, 2 * pi);

  if (angle < 0) {
    angle += 2 * pi;
  }

  return angle - pi;
}

// Test fixture

class TortoisebotActionServerTests : public ::testing::Test {
public:
protected:
  static void SetUpTestSuite();

  static void odom_callback(const std::shared_ptr<const Odometry> msg);
  static void send_goal(const WaypointAction::Goal& goal);
  static void get_pose();

  // Tolerances
  static constexpr double kPi{3.1416};
  static constexpr double position_tol{0.05};
  static constexpr double yaw_tol{kPi / 90}; // +/- 2 degree allowed

  // Goal
  static WaypointAction::Goal goal;

  // State
  static std::optional<Point> position_;
  static std::optional<double> yaw_;
  static WaypointAction::Result::SharedPtr action_result_;

  // Names
  static constexpr char kNodeName[]{"TortoisebotActionServerTestsNode"};
  static constexpr char kOdomTopicName[]{"/odom"};
}; // class TortoisebotActionServerTests

// initialization of static members
std::optional<Point> TortoisebotActionServerTests::position_ = std::nullopt;
std::optional<double> TortoisebotActionServerTests::yaw_ = std::nullopt;
WaypointAction::Result::SharedPtr TortoisebotActionServerTests::action_result_ = nullptr;
WaypointAction::Goal TortoisebotActionServerTests::goal = [] {
  WaypointAction::Goal g;
  g.position.x = 1.0;
  g.position.y = 0.3;
  g.yaw = 1.57;
  return g;
}();

void TortoisebotActionServerTests::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
  send_goal(goal);
  get_pose();
  rclcpp::shutdown();
}

void TortoisebotActionServerTests::odom_callback(
    const std::shared_ptr<const Odometry> msg) {
  yaw_ = compYaw(msg->pose.pose.orientation);
  position_ = msg->pose.pose.position;
}

void
TortoisebotActionServerTests::send_goal(const WaypointAction::Goal& goal) {
  auto action_server{std::make_shared<TortoisebotActionServer>()};
  auto executor{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()};
  executor->add_node(action_server);
  auto spin_thread{std::thread([&]() { executor->spin(); })};

  auto action_client{std::make_shared<TortoisebotActionClient>()};
  action_result_ = action_client->send_goal(goal);

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

void TortoisebotActionServerTests::get_pose() {
  auto test_node{rclcpp::Node::make_shared(kNodeName)};
  auto odom_subscription{test_node->create_subscription<Odometry>(
      kOdomTopicName, 1,
      std::bind(&TortoisebotActionServerTests::odom_callback,
                std::placeholders::_1))};

  while (!yaw_ || ! position_) {
    rclcpp::spin_some(test_node);
  }
}

TEST_F(TortoisebotActionServerTests, PositionErrorTest) {
  ASSERT_TRUE(action_result_ && action_result_->success) << "Action failed to execute successfully.";

  const auto error_position{compErrorPosition(goal.position, position_.value())};
  EXPECT_TRUE(error_position < position_tol) << "Position too far from goal (" + std::to_string(error_position) + ">" + std::to_string(position_tol) + ".";
}

TEST_F(TortoisebotActionServerTests, YawErrorTest) {
  ASSERT_TRUE(action_result_ && action_result_->success) << "Action failed to execute successfully.";

  const auto error_yaw{std::fabs(normalize_angle(goal.yaw - yaw_.value()))};
  EXPECT_TRUE(error_yaw < yaw_tol) << "Yaw too far from goal (" + std::to_string(error_yaw) + ">" + std::to_string(yaw_tol) + ".";
}
