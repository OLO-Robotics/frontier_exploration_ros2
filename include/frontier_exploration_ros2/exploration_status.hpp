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

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace frontier_exploration_ros2
{

// Accumulates session and goal events into a diagnostics status entry.
class ExplorationStatus
{
public:
  enum class Runtime
  {
    IDLE,
    RUNNING,
    STOPPING,
  };

  // Warnings and errors raise the entry level for this long after they are logged.
  static constexpr double kProblemHoldSeconds{10.0};

  void session_started(double now_s);
  void session_finished(double now_s);
  void goal_dispatched(int dispatch_id, const std::string & kind, const geometry_msgs::msg::PoseStamped & pose);
  void goal_rejected(int dispatch_id);
  void goal_feedback(int dispatch_id, double distance_remaining);
  void goal_result(int dispatch_id, int8_t status);
  void frontiers_updated(std::size_t count);
  void warning(const std::string & message, double now_s);
  void error(const std::string & message, double now_s);

  [[nodiscard]] diagnostic_msgs::msg::DiagnosticStatus build(
    const std::string & name,
    Runtime runtime,
    double now_s) const;

private:
  struct ActiveGoal
  {
    int dispatch_id{0};
    std::string kind;
    geometry_msgs::msg::PoseStamped pose;
    std::optional<double> distance_remaining;
  };

  void clear_goal(int dispatch_id);

  std::optional<ActiveGoal> goal_;
  std::optional<double> session_started_s_;
  std::optional<double> session_finished_s_;
  bool finished_{false};
  std::size_t frontiers_{0};
  std::size_t dispatched_{0};
  std::size_t succeeded_{0};
  std::size_t failed_{0};
  std::size_t canceled_{0};
  std::string last_problem_;
  std::optional<double> last_warning_s_;
  std::optional<double> last_error_s_;
};

}  // namespace frontier_exploration_ros2
