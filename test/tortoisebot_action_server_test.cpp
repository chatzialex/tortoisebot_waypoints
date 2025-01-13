#include "odom_listener.hpp"
#include "rclcpp/logging.hpp"
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
#include <limits>
#include <memory>
#include <optional>
#include <thread>

using namespace TortoisebotWaypoints;
using Point = geometry_msgs::msg::Point;
using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using WaypointActionWrappedResultFuture = std::shared_future<
    TortoisebotActionClient::GoalHandleWaypointAction::WrappedResult>;

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
  action_client_ = std::make_shared<TortoisebotActionClient>();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(action_server_);
  executor_->add_node(odom_listener_);
  executor_->add_node(action_client_);
  executor_thread_ = std::thread([&]() { executor_->spin(); });
}

void TortoisebotActionServerTests::TearDown() {
  executor_->cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

TEST_F(TortoisebotActionServerTests, TestAction) {
  std::optional<geometry_msgs::msg::Point> position{std::nullopt};
  std::optional<double> yaw{std::nullopt};

  while (!(yaw = odom_listener_->getYaw()) ||
         !(position = odom_listener_->getPosition())) {
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
  }

  std::optional<WaypointActionWrappedResultFuture> result_future{
      action_client_->send_goal(goal)};
  ASSERT_TRUE(result_future.has_value())
      << "Action request rejected by server.";

  double error_position{}, error_yaw{},
      error_position_best{std::numeric_limits<double>::infinity()},
      error_yaw_best{std::numeric_limits<double>::infinity()};
  bool error_position_improving{false}, error_yaw_improving{false};
  while (result_future.value().wait_for(std::chrono::seconds{10}) !=
         std::future_status::ready) {

    yaw = odom_listener_->getYaw();
    position = odom_listener_->getPosition();
    error_position = compErrorPosition(goal.position, position.value());
    error_yaw = std::fabs(normalize_angle(goal.yaw - yaw.value()));
    error_position_improving =
        error_position < error_position_best || error_position <= position_tol;
    error_yaw_improving = error_yaw < error_yaw_best || error_yaw <= yaw_tol;

    ASSERT_TRUE(error_position_improving || error_yaw_improving)
        << "Action seems stuck for too long.";

    if (error_position < error_position_best) {
      error_position_best = error_position;
    }
    if (error_yaw < error_yaw_best) {
      error_yaw_best = error_yaw;
    }
  }

  ASSERT_TRUE(result_future.value().get().result->success)
      << "Action failed to execute successfully.";

  position = odom_listener_->getPosition();
  yaw = odom_listener_->getYaw();

  error_position = compErrorPosition(goal.position, position.value());
  EXPECT_TRUE(error_position <= position_tol)
      << "Position too far from goal (" + std::to_string(error_position) + ">" +
             std::to_string(position_tol) + ".";

  error_yaw = std::fabs(normalize_angle(goal.yaw - yaw.value()));
  EXPECT_TRUE(error_yaw <= yaw_tol) << "Yaw too far from goal (" +
                                           std::to_string(error_yaw) + ">" +
                                           std::to_string(yaw_tol) + ".";
}
