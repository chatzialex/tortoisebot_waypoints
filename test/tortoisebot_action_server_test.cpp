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
#include <thread>
#include <iostream>

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

// Test fixture

class TortoisebotActionServerTests : public ::testing::Test {
public:
protected:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  static void odom_callback(const std::shared_ptr<const Odometry> msg);
  static WaypointAction::Result::SharedPtr send_goal(const WaypointAction::Goal& goal);

  // Tolerances
  static constexpr double kPi{3.1416};
  static constexpr double position_tol{0.05};
  static constexpr double yaw_tol{kPi / 90}; // +/- 2 degree allowed

  // Goal
  static WaypointAction::Goal goal;

  // State
  static Point position_;
  static double yaw_;
  static WaypointAction::Result::SharedPtr action_result_;

  // Names
  static constexpr char kNodeName[]{"TortoisebotActionServerTestsNode"};
  static constexpr char kOdomTopicName[]{"/odom"};

  // ROS interfaces
  static rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  static std::shared_ptr<rclcpp::Node> node_;
}; // class TortoisebotActionServerTests

// initialization of static members
Point TortoisebotActionServerTests::position_ = Point();
double TortoisebotActionServerTests::yaw_ = {};
WaypointAction::Result::SharedPtr TortoisebotActionServerTests::action_result_ = nullptr;
rclcpp::Subscription<Odometry>::SharedPtr
    TortoisebotActionServerTests::odom_sub_ = nullptr;
std::shared_ptr<rclcpp::Node> TortoisebotActionServerTests::node_ = nullptr;
WaypointAction::Goal TortoisebotActionServerTests::goal = [] {
  WaypointAction::Goal g;
  g.position.x = 1.0;
  g.yaw = 1.57;
  return g;
}();

void TortoisebotActionServerTests::SetUpTestSuite() {
  rclcpp::init(0, nullptr);

  action_result_=send_goal(goal);

  node_ = rclcpp::Node::make_shared(kNodeName);
  node_->create_subscription<Odometry>(
      kOdomTopicName, 1,
      std::bind(&TortoisebotActionServerTests::odom_callback,
                std::placeholders::_1));
}

void TortoisebotActionServerTests::odom_callback(
    const std::shared_ptr<const Odometry> msg) {
  yaw_ = compYaw(msg->pose.pose.orientation);
  position_ = msg->pose.pose.position;
}

WaypointAction::Result::SharedPtr
TortoisebotActionServerTests::send_goal(const WaypointAction::Goal& goal) {
  auto action_server{std::make_shared<TortoisebotActionServer>()};
  auto executor{std::make_shared<rclcpp::executors::SingleThreadedExecutor>()};
  executor->add_node(action_server);
  auto spin_thread{std::thread([&]() { executor->spin(); })};

  auto action_client{std::make_shared<TortoisebotActionClient>()};
  auto action_result{action_client->send_goal(goal)};

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return action_result;
}

void TortoisebotActionServerTests::TearDownTestSuite() {
  rclcpp::shutdown();
}

TEST_F(TortoisebotActionServerTests, PositionErrorTest) {}

TEST_F(TortoisebotActionServerTests, YawErrorTest) {}
