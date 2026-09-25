/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"

namespace frontier_exploration_ros2
{
namespace
{

using Trigger = std_srvs::srv::Trigger;

geometry_msgs::msg::Pose make_pose(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

FrontierCandidate make_frontier(double x, double y, int size = 1)
{
  return FrontierCandidate{{x, y}, {x, y}, size};
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

TEST(ControlCoreSessionTests, StopExplorationSessionDisablesScheduling)
{
  int frontier_search_calls = 0;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{1'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.frontier_search =
    [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      return FrontierSearchResult{};
    };

  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);
  core.map = OccupancyGrid2d(build_grid(10, 10, 0));
  core.costmap = OccupancyGrid2d(build_grid(10, 10, 0));
  core.stop_exploration_session("test stop");
  core.try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(ControlCoreSessionTests, StartExplorationSessionResetsSessionState)
{
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);

  geometry_msgs::msg::PoseStamped persistent_start_pose;
  persistent_start_pose.pose = make_pose(2.0, 3.0);
  core.start_pose = persistent_start_pose;
  core.pending_frontier_sequence = {make_frontier(1.0, 1.0)};
  core.pending_frontier_selection_mode = "mrtsp";
  core.return_to_start_completed = true;
  core.no_frontiers_reported = true;
  core.frontier_suppression_ = std::make_unique<FrontierSuppression>(FrontierSuppressionConfig{});
  core.map = OccupancyGrid2d(build_grid(10, 10, 0));
  core.costmap = OccupancyGrid2d(build_grid(10, 10, 0));
  core.map_generation = 1;
  core.costmap_generation = 2;

  core.start_exploration_session();

  EXPECT_TRUE(core.exploration_enabled);
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);
  EXPECT_TRUE(core.pending_frontier_sequence.empty());
  EXPECT_TRUE(core.pending_frontier_selection_mode.empty());
  EXPECT_FALSE(core.return_to_start_completed);
  EXPECT_FALSE(core.no_frontiers_reported);
  EXPECT_FALSE(core.map.has_value());
  EXPECT_FALSE(core.costmap.has_value());
  EXPECT_EQ(core.map_generation, 0);
  EXPECT_EQ(core.costmap_generation, 0);
  EXPECT_FALSE(core.frontier_suppression_);
}

TEST(ControlCoreSessionTests, StopAndStartSessionsPreserveOriginalStartPose)
{
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, callbacks);

  geometry_msgs::msg::PoseStamped persistent_start_pose;
  persistent_start_pose.pose = make_pose(4.0, 6.0);
  core.start_pose = persistent_start_pose;

  core.stop_exploration_session("test stop");
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);

  core.start_exploration_session();
  ASSERT_TRUE(core.start_pose.has_value());
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.x, persistent_start_pose.pose.position.x);
  EXPECT_DOUBLE_EQ(core.start_pose->pose.position.y, persistent_start_pose.pose.position.y);
}

class FrontierControlNodeTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    helper_node_ = std::make_shared<rclcpp::Node>("frontier_control_test_helper");
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_->remove_node(node_);
      node_.reset();
    }
    if (helper_node_) {
      executor_->remove_node(helper_node_);
      helper_node_.reset();
    }
    executor_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void create_node(bool autostart, bool control_service_enabled = true)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("control.autostart", autostart),
      rclcpp::Parameter("control.service_enabled", control_service_enabled),
      rclcpp::Parameter("completion.event_enabled", false),
      rclcpp::Parameter("suppression.enabled", false),
    });
    node_ = std::make_shared<FrontierExplorerNode>(options);
    executor_->add_node(node_);
  }

  std::shared_ptr<Trigger::Response> call_trigger(const std::string & service_name)
  {
    auto client = helper_node_->create_client<Trigger>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      ADD_FAILURE() << service_name << " did not become ready";
      return nullptr;
    }

    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (
      executor_->spin_until_future_complete(future, std::chrono::seconds(2)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << service_name << " call timed out";
      return nullptr;
    }
    return future.get();
  }

  bool wait_for_condition(
    const std::function<bool()> & predicate,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    return predicate();
  }

  bool wait_for_control_service_availability(
    bool expected_available,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    auto client = helper_node_->create_client<Trigger>("/frontier_explorer/start");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      const bool available = client->wait_for_service(std::chrono::milliseconds(0));
      if (available == expected_available) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    return client->wait_for_service(std::chrono::milliseconds(0)) == expected_available;
  }

  // Latest exploration state reported on /diagnostics, or empty before the first message.
  std::string reported_state()
  {
    if (!diagnostics_sub_) {
      diagnostics_sub_ = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,
        [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          for (const auto & status : msg->status) {
            if (status.name == "/frontier_explorer: exploration") {
              reported_state_ = status.message;
            }
          }
        });
    }
    return reported_state_;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::string reported_state_;
  std::shared_ptr<FrontierExplorerNode> node_;
};

TEST_F(FrontierControlNodeTests, AutostartFalseKeepsSubscriptionsInactive)
{
  create_node(false);

  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));
}

TEST_F(FrontierControlNodeTests, AutostartTrueCreatesSubscriptions)
{
  create_node(true);

  ASSERT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(2000)));
}

TEST_F(FrontierControlNodeTests, ControlServiceCanBeDisabledWhenAutostartIsTrue)
{
  create_node(true, false);

  EXPECT_FALSE(node_->hasControlService());
  EXPECT_TRUE(wait_for_condition(
    [this]() { return node_->hasActiveExplorationSubscriptions(); },
    std::chrono::milliseconds(2000)));
  EXPECT_TRUE(wait_for_control_service_availability(false));
}

TEST_F(FrontierControlNodeTests, ColdIdleForcesControlServiceOn)
{
  create_node(false, false);

  EXPECT_TRUE(node_->hasControlService());
  EXPECT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));
  EXPECT_TRUE(wait_for_control_service_availability(true));
}

TEST_F(FrontierControlNodeTests, StartServiceActivatesSubscriptions)
{
  create_node(false);

  const auto response = call_trigger("/frontier_explorer/start");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  ASSERT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));

  const auto repeated = call_trigger("/frontier_explorer/start");
  ASSERT_NE(repeated, nullptr);
  EXPECT_TRUE(repeated->success);
  EXPECT_EQ(repeated->message, "Exploration is already running");
  EXPECT_TRUE(node_->hasActiveExplorationSubscriptions());
}

TEST_F(FrontierControlNodeTests, StopServiceReturnsNodeToColdIdle)
{
  create_node(true);
  ASSERT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));

  const auto response = call_trigger("/frontier_explorer/stop");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  ASSERT_TRUE(wait_for_condition([this]() { return !node_->hasActiveExplorationSubscriptions(); }));

  const auto restarted = call_trigger("/frontier_explorer/start");
  ASSERT_NE(restarted, nullptr);
  EXPECT_TRUE(restarted->success);
  EXPECT_TRUE(wait_for_condition([this]() { return node_->hasActiveExplorationSubscriptions(); }));
}

TEST_F(FrontierControlNodeTests, DiagnosticsFollowStartAndStop)
{
  reported_state();
  create_node(false);
  ASSERT_TRUE(wait_for_condition([this]() { return reported_state() == "idle"; }, std::chrono::milliseconds(3000)));

  ASSERT_NE(call_trigger("/frontier_explorer/start"), nullptr);
  EXPECT_TRUE(wait_for_condition([this]() { return reported_state() == "exploring"; }, std::chrono::milliseconds(3000)));

  ASSERT_NE(call_trigger("/frontier_explorer/stop"), nullptr);
  EXPECT_TRUE(wait_for_condition([this]() { return reported_state() == "idle"; }, std::chrono::milliseconds(3000)));
}

TEST_F(FrontierControlNodeTests, ExploresFromLatchedMapAndCostmaps)
{
  // A robot that has not moved yet gets one latched map from SLAM and one latched
  // full costmap from Nav2 (sent at activation, then only on geometry changes),
  // both published before exploring starts. Frontiers must still be found.
  nav_msgs::msg::OccupancyGrid map = build_grid(40, 40, -1);
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.origin.position.x = -2.0;
  map.info.origin.position.y = -2.0;
  for (int y = 10; y < 30; ++y) {
    for (int x = 10; x < 30; ++x) {
      map.data[static_cast<std::size_t>(y * 40 + x)] = 0;
    }
  }
  nav_msgs::msg::OccupancyGrid costmap = build_grid(40, 40, 0);
  costmap.header.frame_id = "map";
  costmap.info = map.info;

  auto map_pub = helper_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local());
  map_pub->publish(map);
  auto costmap_pub = helper_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", rclcpp::QoS(1).reliable().transient_local());
  costmap_pub->publish(costmap);
  auto local_costmap_pub = helper_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::QoS(1).reliable().transient_local());
  local_costmap_pub->publish(costmap);
  tf2_ros::StaticTransformBroadcaster tf_broadcaster(helper_node_);
  geometry_msgs::msg::TransformStamped robot_pose;
  robot_pose.header.frame_id = "map";
  robot_pose.child_frame_id = "base_footprint";
  robot_pose.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(robot_pose);

  std::size_t frontier_points = 0;
  auto marker_sub = helper_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/explore/frontiers", 10,
    [&frontier_points](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
      for (const auto & marker : msg->markers) {
        frontier_points += marker.points.size();
      }
    });

  create_node(true);
  EXPECT_TRUE(wait_for_condition(
    [&frontier_points]() {return frontier_points > 0;},
    std::chrono::milliseconds(5000)));
}

TEST_F(FrontierControlNodeTests, StopWhileIdleIsANoOp)
{
  create_node(false);

  const auto response = call_trigger("/frontier_explorer/stop");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Exploration is not running");
  EXPECT_FALSE(node_->hasActiveExplorationSubscriptions());
}

}  // namespace
}  // namespace frontier_exploration_ros2
