#include <gtest/gtest.h>

#include <action_msgs/msg/goal_status.hpp>

#include <map>
#include <string>

#include "frontier_exploration_ros2/exploration_status.hpp"

namespace frontier_exploration_ros2
{
namespace
{

using action_msgs::msg::GoalStatus;
using diagnostic_msgs::msg::DiagnosticStatus;

std::map<std::string, std::string> values_of(const DiagnosticStatus & status)
{
  std::map<std::string, std::string> values;
  for (const auto & entry : status.values) {
    values[entry.key] = entry.value;
  }
  return values;
}

geometry_msgs::msg::PoseStamped make_goal(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(ExplorationStatusTests, TracksGoalLifecycleAcrossPreemption)
{
  ExplorationStatus status;
  status.session_started(100.0);
  status.frontiers_updated(4);

  status.goal_dispatched(1, "frontier", make_goal(1.0, 2.0));
  status.goal_feedback(1, 3.5);
  auto values = values_of(status.build("n", ExplorationStatus::Runtime::RUNNING, 102.0));
  EXPECT_EQ(values["state"], "exploring");
  EXPECT_EQ(values["goal_kind"], "frontier");
  EXPECT_EQ(values["goal_x"], "1.00");
  EXPECT_EQ(values["distance_remaining"], "3.50");
  EXPECT_EQ(values["frontiers"], "4");
  EXPECT_EQ(values["elapsed_s"], "2.00");

  // A preempting goal is dispatched before the cancelled one reports back.
  status.goal_dispatched(2, "frontier", make_goal(5.0, 6.0));
  status.goal_result(1, GoalStatus::STATUS_CANCELED);
  values = values_of(status.build("n", ExplorationStatus::Runtime::RUNNING, 103.0));
  EXPECT_EQ(values["goal_x"], "5.00");
  EXPECT_EQ(values["distance_remaining"], "");

  status.goal_result(2, GoalStatus::STATUS_ABORTED);
  status.warning("frontier finished with status ABORTED", 103.0);
  const auto aborted = status.build("n", ExplorationStatus::Runtime::RUNNING, 104.0);
  values = values_of(aborted);
  EXPECT_EQ(aborted.level, DiagnosticStatus::WARN);
  EXPECT_EQ(values["goal_kind"], "");
  EXPECT_EQ(values["goals_dispatched"], "2");
  EXPECT_EQ(values["goals_canceled"], "1");
  EXPECT_EQ(values["goals_failed"], "1");
  EXPECT_EQ(values["last_problem"], "frontier finished with status ABORTED");

  const auto later = status.build("n", ExplorationStatus::Runtime::RUNNING, 120.0);
  EXPECT_EQ(later.level, DiagnosticStatus::OK);
}

TEST(ExplorationStatusTests, CompletedSessionReportsCompleteUntilRestarted)
{
  ExplorationStatus status;
  status.session_started(10.0);
  status.goal_dispatched(1, "return_to_start", make_goal(0.0, 0.0));
  EXPECT_EQ(status.build("n", ExplorationStatus::Runtime::RUNNING, 11.0).message, "returning to start");

  status.goal_result(1, GoalStatus::STATUS_SUCCEEDED);
  status.session_finished(12.0);
  auto values = values_of(status.build("n", ExplorationStatus::Runtime::IDLE, 30.0));
  EXPECT_EQ(values["state"], "complete");
  EXPECT_EQ(values["elapsed_s"], "2.00");
  EXPECT_EQ(values["goals_succeeded"], "1");

  status.session_started(40.0);
  values = values_of(status.build("n", ExplorationStatus::Runtime::RUNNING, 41.0));
  EXPECT_EQ(values["state"], "exploring");
  EXPECT_EQ(values["goals_succeeded"], "0");
}

}  // namespace
}  // namespace frontier_exploration_ros2
