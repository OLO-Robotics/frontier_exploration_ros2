/*
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

#include "frontier_exploration_ros2/exploration_status.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace frontier_exploration_ros2
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

std::string format_number(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool within(const std::optional<double> & event_s, double now_s)
{
  return event_s.has_value() && now_s - *event_s <= ExplorationStatus::kProblemHoldSeconds;
}

}  // namespace

void ExplorationStatus::session_started(double now_s)
{
  *this = ExplorationStatus{};
  session_started_s_ = now_s;
}

void ExplorationStatus::session_finished(double now_s)
{
  finished_ = true;
  session_finished_s_ = now_s;
  goal_.reset();
}

void ExplorationStatus::goal_dispatched(
  int dispatch_id,
  const std::string & kind,
  const geometry_msgs::msg::PoseStamped & pose)
{
  goal_ = ActiveGoal{dispatch_id, kind, pose, std::nullopt};
  ++dispatched_;
}

void ExplorationStatus::goal_rejected(int dispatch_id)
{
  ++failed_;
  clear_goal(dispatch_id);
}

void ExplorationStatus::goal_feedback(int dispatch_id, double distance_remaining)
{
  if (goal_.has_value() && goal_->dispatch_id == dispatch_id) {
    goal_->distance_remaining = distance_remaining;
  }
}

void ExplorationStatus::goal_result(int dispatch_id, int8_t status)
{
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      ++succeeded_;
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      ++canceled_;
      break;
    default:
      ++failed_;
      break;
  }
  clear_goal(dispatch_id);
}

void ExplorationStatus::frontiers_updated(std::size_t count)
{
  frontiers_ = count;
}

void ExplorationStatus::warning(const std::string & message, double now_s)
{
  last_problem_ = message;
  last_warning_s_ = now_s;
}

void ExplorationStatus::error(const std::string & message, double now_s)
{
  last_problem_ = message;
  last_error_s_ = now_s;
}

void ExplorationStatus::clear_goal(int dispatch_id)
{
  // Results of superseded dispatches must not clear the goal that replaced them.
  if (goal_.has_value() && goal_->dispatch_id == dispatch_id) {
    goal_.reset();
  }
}

DiagnosticStatus ExplorationStatus::build(
  const std::string & name,
  Runtime runtime,
  double now_s) const
{
  DiagnosticStatus status;
  status.name = name;
  status.hardware_id = name;

  switch (runtime) {
    case Runtime::RUNNING:
      status.message = goal_.has_value() && goal_->kind == "return_to_start" ?
        "returning to start" : "exploring";
      break;
    case Runtime::STOPPING:
      status.message = "stopping";
      break;
    case Runtime::IDLE:
      status.message = finished_ ? "complete" : "idle";
      break;
  }

  status.level = within(last_error_s_, now_s) ? DiagnosticStatus::ERROR :
    within(last_warning_s_, now_s) ? DiagnosticStatus::WARN : DiagnosticStatus::OK;

  auto add = [&status](const std::string & key, const std::string & value) {
      KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  add("state", status.message);
  add("goal_kind", goal_.has_value() ? goal_->kind : "");
  add("goal_x", goal_.has_value() ? format_number(goal_->pose.pose.position.x) : "");
  add("goal_y", goal_.has_value() ? format_number(goal_->pose.pose.position.y) : "");
  add("goal_yaw", goal_.has_value() ? format_number(yaw_of(goal_->pose.pose.orientation)) : "");
  add(
    "distance_remaining",
    goal_.has_value() && goal_->distance_remaining.has_value() ?
    format_number(*goal_->distance_remaining) : "");
  add("frontiers", std::to_string(frontiers_));
  add("goals_dispatched", std::to_string(dispatched_));
  add("goals_succeeded", std::to_string(succeeded_));
  add("goals_failed", std::to_string(failed_));
  add("goals_canceled", std::to_string(canceled_));
  // A completed session keeps reporting its total duration until the next start.
  const bool timed = session_started_s_.has_value() && (runtime != Runtime::IDLE || finished_);
  add(
    "elapsed_s",
    timed ? format_number(session_finished_s_.value_or(now_s) - *session_started_s_) : "");
  add("last_problem", last_problem_);
  return status;
}

}  // namespace frontier_exploration_ros2
