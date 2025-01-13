#include "odom_listener.hpp"
#include "tortoisebot_action_client.hpp"
#include "tortoisebot_waypoints/tortoisebot_action_server.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#include "gtest/gtest.h"

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <thread>

using namespace TortoisebotWaypoints;
using Point = geometry_msgs::msg::Point;
using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using WaypointActionResultFuture = std::shared_future<
    TortoisebotActionClient::GoalHandleWaypointAction::SharedPtr>;

// Helper functions

double compErrorPosition(const Point &p1, const Point &p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
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
  static void TearDownTestSuite();

  void SetUp() override;
  void TearDown() override;

  // Tolerances
  static constexpr double kPi{3.1416};
  static constexpr double position_tol{0.05};
  static constexpr double yaw_tol{kPi / 90}; // +/- 2 degree allowed

  // Goal
  WaypointAction::Goal goal{[] {
    WaypointAction::Goal g;
    g.position.x = 1.0;
    g.position.y = 0.3;
    g.yaw = 1.57;
    return g;
  }()};

  // ROS
  std::shared_ptr<TortoisebotActionServer> action_server_{};
  std::shared_ptr<TortoisebotActionClient> action_client_{};
  std::shared_ptr<OdomListener> odom_listener_{};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
  std::thread executor_thread_{};
}; // class TortoisebotActionServerTests

void TortoisebotActionServerTests::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void TortoisebotActionServerTests::TearDownTestSuite() { rclcpp::shutdown(); }

void TortoisebotActionServerTests::SetUp() {
  action_server_ = std::make_shared<TortoisebotActionServer>();
  odom_listener_ = std::make_shared<OdomListener>();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(action_server_);
  executor_->add_node(odom_listener_);
  executor_thread_ = std::thread([&]() { executor_->spin(); });

  action_client_ = std::make_shared<TortoisebotActionClient>();
}

void TortoisebotActionServerTests::TearDown() {
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

TEST_F(TortoisebotActionServerTests, TestAction) {
  std::optional<geometry_msgs::msg::Point> position_{std::nullopt};
  std::optional<double> yaw_{std::nullopt};

  while (!(yaw_ = odom_listener_->getYaw()) ||
         !(position_ = odom_listener_->getPosition())) {
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  WaypointAction::Result::SharedPtr action_result{
      action_client_->send_goal(goal)};

  yaw_ = odom_listener_->getYaw();
  position_ = odom_listener_->getPosition();

  ASSERT_TRUE(action_result && action_result->success)
      << "Action failed to execute successfully.";

  const auto error_position{
      compErrorPosition(goal.position, position_.value())};
  EXPECT_TRUE(error_position < position_tol)
      << "Position too far from goal (" + std::to_string(error_position) + ">" +
             std::to_string(position_tol) + ".";

  const auto error_yaw{std::fabs(normalize_angle(goal.yaw - yaw_.value()))};
  EXPECT_TRUE(error_yaw < yaw_tol) << "Yaw too far from goal (" +
                                          std::to_string(error_yaw) + ">" +
                                          std::to_string(yaw_tol) + ".";
}
