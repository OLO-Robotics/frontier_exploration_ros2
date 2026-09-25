/*
Copyright 2026 Mert Guler

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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/debug/debug_analyzer.hpp"
#include "frontier_exploration_ros2/debug/debug_markers.hpp"
#include "frontier_exploration_ros2/qos_utils.hpp"
#include "frontier_debug_observer_parameters.hpp"
#include "frontier_explorer_parameters.hpp"

namespace frontier_exploration_ros2::debug
{

namespace
{

constexpr const char * kChunkCacheTopic = "explore/debug/chunk_cache";

}  // namespace

class FrontierDebugObserverNode : public rclcpp::Node
{
public:
  explicit FrontierDebugObserverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("frontier_debug_observer", options)
  {
    // This node is intentionally passive: it subscribes, analyzes, and publishes
    // RViz overlays, but it never sends Nav2 goals or mutates explorer state.
    read_parameters();
    create_ros_interfaces();
    RCLCPP_INFO(
      get_logger(),
      "Frontier debug observer ready: map='%s', costmap='%s', local_costmap='%s'",
      map_topic_.c_str(),
      costmap_topic_.c_str(),
      local_costmap_topic_.c_str());
  }

private:
  void read_parameters()
  {
    // Explorer parameters mirror the explorer node so the same file drives both.
    // Parameters are read once; the observer does not reconfigure at runtime.
    const auto explorer = frontier_explorer_params::ParamListener(
      get_node_parameters_interface(), get_logger()).get_params();
    const auto observer = frontier_debug_observer_params::ParamListener(
      get_node_parameters_interface(), get_logger()).get_params();

    map_topic_ = explorer.topics.map;
    costmap_topic_ = explorer.topics.costmap;
    local_costmap_topic_ = explorer.topics.local_costmap;
    global_frame_ = explorer.frames.global;
    robot_base_frame_ = explorer.frames.robot_base;

    analyzer_config_.frontier_map_optimization_enabled = explorer.map_optimization.enabled;
    analyzer_config_.sigma_s = explorer.map_optimization.sigma_s;
    analyzer_config_.sigma_r = explorer.map_optimization.sigma_r;
    analyzer_config_.dilation_kernel_radius_cells =
      static_cast<int>(explorer.map_optimization.dilation_kernel_radius_cells);
    analyzer_config_.sensor_effective_range_m = explorer.ordering.sensor_effective_range_m;
    analyzer_config_.weight_distance_wd = explorer.ordering.weight_distance;
    analyzer_config_.weight_gain_ws = explorer.ordering.weight_gain;
    analyzer_config_.max_linear_speed_vmax = explorer.ordering.max_linear_speed;
    analyzer_config_.max_angular_speed_wmax = explorer.ordering.max_angular_speed;
    analyzer_config_.mrtsp_solver = explorer.ordering.solver;
    analyzer_config_.dp_solver_candidate_limit =
      static_cast<std::size_t>(explorer.ordering.dp.candidate_limit);
    analyzer_config_.dp_planning_horizon =
      static_cast<std::size_t>(explorer.ordering.dp.planning_horizon);
    analyzer_config_.occ_threshold = static_cast<int>(explorer.frontier.occ_threshold);
    analyzer_config_.min_frontier_size_cells = static_cast<int>(explorer.frontier.min_size_cells);
    analyzer_config_.frontier_candidate_min_goal_distance_m =
      explorer.frontier.candidate_min_goal_distance_m;
    analyzer_config_.frontier_selection_min_distance = explorer.frontier.selection_min_distance;
    analyzer_config_.frontier_visit_tolerance = explorer.frontier.visit_tolerance;

    update_rate_hz_ = observer.debug.update_rate_hz;
    marker_config_.frame_id = global_frame_;
    marker_config_.point_scale = observer.debug.marker_scale;
    marker_config_.selected_scale = observer.debug.selected_marker_scale;
    marker_config_.line_width = observer.debug.line_width;
    marker_config_.text_scale = observer.debug.text_scale;
    marker_config_.labels_enabled = observer.debug.labels_enabled;
    marker_config_.label_top_n = static_cast<std::size_t>(observer.debug.label_top_n);
    marker_config_.edge_top_n = static_cast<std::size_t>(observer.debug.edge_top_n);

    show_raw_frontiers_ = observer.debug.show.raw_frontiers;
    show_optimized_frontiers_ = observer.debug.show.optimized_frontiers;
    show_mrtsp_scores_ = observer.debug.show.mrtsp_scores;
    show_mrtsp_order_ = observer.debug.show.mrtsp_order;
    show_dp_pruning_ = observer.debug.show.dp_pruning;
    show_decision_map_ = observer.debug.show.decision_map;

    // MRTSP order still needs MRTSP scores because it depends on the same cost matrix.
    analyzer_config_.analyze_mrtsp_scores = show_mrtsp_scores_ || show_mrtsp_order_;
    analyzer_config_.analyze_dp_pruning = show_dp_pruning_;

    raw_frontiers_topic_ = observer.debug.topics.raw_frontiers;
    optimized_frontiers_topic_ = observer.debug.topics.optimized_frontiers;
    mrtsp_scores_topic_ = observer.debug.topics.mrtsp_scores;
    mrtsp_order_topic_ = observer.debug.topics.mrtsp_order;
    dp_pruning_topic_ = observer.debug.topics.dp_pruning;
    decision_map_topic_ = observer.debug.topics.decision_map;

    topic_qos_profiles_ = resolve_topic_qos_profiles(
      explorer.qos.map.durability,
      explorer.qos.map.reliability,
      explorer.qos.map.depth,
      explorer.qos.costmap.durability,
      explorer.qos.costmap.reliability,
      explorer.qos.costmap.depth,
      explorer.qos.local_costmap.reliability,
      explorer.qos.local_costmap.depth);
  }

  void create_ros_interfaces()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions only cache the latest grids. The timer copies the cached
    // values under a mutex, then performs analysis without holding the lock.
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      topic_qos_profiles_.make_map_qos(topic_qos_profiles_.map_durability),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_ = OccupancyGrid2d(msg);
      });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      topic_qos_profiles_.make_costmap_qos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        costmap_ = OccupancyGrid2d(msg);
      });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_,
      topic_qos_profiles_.make_local_costmap_qos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_costmap_ = OccupancyGrid2d(msg);
      });

    raw_frontiers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      raw_frontiers_topic_,
      10);
    optimized_frontiers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      optimized_frontiers_topic_,
      10);
    mrtsp_scores_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      mrtsp_scores_topic_,
      10);
    mrtsp_order_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      mrtsp_order_topic_,
      10);
    dp_pruning_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      dp_pruning_topic_,
      10);
    decision_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      decision_map_topic_,
      10);
    chunk_cache_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      kChunkCacheTopic,
      10);

    // The periodic timer makes debug output predictable and avoids running heavy
    // analysis directly inside map or costmap subscription callbacks.
    analysis_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      std::bind(&FrontierDebugObserverNode::analysis_timer_callback, this));
  }

  std::optional<geometry_msgs::msg::Pose> get_current_pose()
  {
    try {
      // The observer uses the latest available transform to match RViz and map
      // overlays. Missing TF simply skips this tick instead of blocking startup.
      const auto transform = tf_buffer_->lookupTransform(
        global_frame_,
        robot_base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & exc) {
      throttled_warn(
        "frontier_debug_observer_tf",
        "Could not transform " + robot_base_frame_ + " -> " + global_frame_ + ": " + exc.what());
      return std::nullopt;
    }
  }

  void throttled_warn(const std::string & key, const std::string & message)
  {
    // Missing maps, TF, or analysis inputs can happen during startup. Throttling
    // keeps logs useful while still showing persistent configuration problems.
    const int64_t now_ns = get_clock()->now().nanoseconds();
    auto & last_time = warning_times_[key];
    const int64_t throttle_ns = static_cast<int64_t>(5.0 * 1e9);
    if (!last_time.has_value() || now_ns - *last_time >= throttle_ns) {
      last_time = now_ns;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  void analysis_timer_callback()
  {
    std::optional<OccupancyGrid2d> map;
    std::optional<OccupancyGrid2d> costmap;
    std::optional<OccupancyGrid2d> local_costmap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map = map_;
      costmap = costmap_;
      local_costmap = local_costmap_;
    }

    // A global costmap is required because frontier eligibility and blocked-goal
    // checks depend on it. The local costmap remains optional for debug analysis.
    if (!map.has_value() || !costmap.has_value()) {
      throttled_warn(
        "frontier_debug_observer_maps",
        "Waiting for map and global costmap before publishing frontier debug overlays");
      return;
    }

    const auto current_pose = get_current_pose();
    if (!current_pose.has_value()) {
      return;
    }

    try {
      // The analyzer returns a complete, immutable snapshot for this tick. Each
      // publisher then converts the same snapshot into a separate RViz layer.
      const FrontierDebugSnapshot snapshot = analyze_frontier_debug_snapshot(
        *current_pose,
        *map,
        *costmap,
        local_costmap,
        analyzer_config_,
        decision_map_workspace_);

      if (show_raw_frontiers_) {
        raw_frontiers_pub_->publish(make_raw_frontier_markers(snapshot, marker_config_));
      }
      if (show_optimized_frontiers_) {
        optimized_frontiers_pub_->publish(make_optimized_frontier_markers(snapshot, marker_config_));
      }
      if (show_mrtsp_scores_) {
        mrtsp_scores_pub_->publish(make_mrtsp_score_markers(snapshot, marker_config_));
      }
      if (show_mrtsp_order_) {
        mrtsp_order_pub_->publish(make_mrtsp_order_markers(snapshot, *current_pose, marker_config_));
      }
      if (show_dp_pruning_) {
        dp_pruning_pub_->publish(make_dp_pruning_markers(snapshot, marker_config_));
      }
      if (show_decision_map_) {
        decision_map_pub_->publish(snapshot.decision_map_msg);
      }
      chunk_cache_pub_->publish(make_decision_map_chunk_cache_markers(snapshot, marker_config_));
      if (!first_successful_publish_logged_) {
        // A one-time success message confirms that all required inputs were
        // received and at least one complete overlay set reached the publishers.
        first_successful_publish_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Frontier debug overlays published successfully: raw_frontiers=%zu, optimized_frontiers=%zu, candidates=%zu, active_mode='%s'",
          snapshot.raw_frontiers.size(),
          snapshot.optimized_frontiers.size(),
          snapshot.candidates.size(),
          snapshot.active_selection_mode.c_str());
      }
    } catch (const std::exception & exc) {
      throttled_warn(
        "frontier_debug_observer_analysis",
        std::string("Could not build frontier debug snapshot: ") + exc.what());
    }
  }

  DebugAnalyzerConfig analyzer_config_;
  DebugMarkerConfig marker_config_;
  TopicQosProfiles topic_qos_profiles_;

  std::string map_topic_;
  std::string costmap_topic_;
  std::string local_costmap_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string raw_frontiers_topic_;
  std::string optimized_frontiers_topic_;
  std::string mrtsp_scores_topic_;
  std::string mrtsp_order_topic_;
  std::string dp_pruning_topic_;
  std::string decision_map_topic_;

  double update_rate_hz_{1.0};
  bool show_raw_frontiers_{true};
  bool show_optimized_frontiers_{true};
  bool show_mrtsp_scores_{true};
  bool show_mrtsp_order_{true};
  bool show_dp_pruning_{true};
  bool show_decision_map_{true};
  bool first_successful_publish_logged_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrtsp_scores_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrtsp_order_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dp_pruning_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr decision_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr chunk_cache_pub_;
  rclcpp::TimerBase::SharedPtr analysis_timer_;

  std::mutex data_mutex_;
  std::optional<OccupancyGrid2d> map_;
  std::optional<OccupancyGrid2d> costmap_;
  std::optional<OccupancyGrid2d> local_costmap_;
  DecisionMapWorkspace decision_map_workspace_;
  std::unordered_map<std::string, std::optional<int64_t>> warning_times_;
};

}  // namespace frontier_exploration_ros2::debug

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frontier_exploration_ros2::debug::FrontierDebugObserverNode>());
  rclcpp::shutdown();
  return 0;
}
