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

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
#include "frontier_exploration_ros2/nav2_compat.hpp"
#include "frontier_explorer_parameters.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rcutils/logging.h>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <visualization_msgs/msg/marker.hpp>

namespace frontier_exploration_ros2
{

namespace
{

constexpr std::size_t kStartupMapRateSampleCount = 3;
constexpr double kStartupMapRateStabilityTolerance = 0.25;

// Adapts Nav2 goal handle API to the core's transport-agnostic interface.
class NavigateGoalHandleAdapter : public GoalHandleInterface
{
public:
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using CancelRequester = std::function<void(
    const std::shared_ptr<NavigateGoalHandle> &,
    std::function<void(bool accepted, const std::string & error_message)>)>;

  NavigateGoalHandleAdapter(
    std::shared_ptr<NavigateGoalHandle> handle,
    CancelRequester cancel_requester)
  : handle_(std::move(handle)), cancel_requester_(std::move(cancel_requester))
  {
  }

  void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) override
  {
    cancel_requester_(handle_, std::move(callback));
  }

private:
  std::shared_ptr<NavigateGoalHandle> handle_;
  CancelRequester cancel_requester_;
};

FrontierExplorerCoreParams toCoreParams(const frontier_explorer_params::Params & params)
{
  FrontierExplorerCoreParams core;
  core.map_topic = params.topics.map;
  core.costmap_topic = params.topics.costmap;
  core.local_costmap_topic = params.topics.local_costmap;
  core.navigate_to_pose_action_name = params.actions.navigate_to_pose;
  core.global_frame = params.frames.global;
  core.robot_base_frame = params.frames.robot_base;
  core.frontier_marker_topic = params.topics.frontier_markers;
  core.selected_frontier_topic = params.topics.selected_frontier;
  core.optimized_map_topic = params.topics.optimized_map;
  core.frontier_marker_scale = params.visualization.marker_scale;
  core.frontier_map_optimization_enabled = params.map_optimization.enabled;
  core.sigma_s = params.map_optimization.sigma_s;
  core.sigma_r = params.map_optimization.sigma_r;
  core.dilation_kernel_radius_cells = static_cast<int>(params.map_optimization.dilation_kernel_radius_cells);
  core.sensor_effective_range_m = params.ordering.sensor_effective_range_m;
  core.weight_distance_wd = params.ordering.weight_distance;
  core.weight_gain_ws = params.ordering.weight_gain;
  core.max_linear_speed_vmax = params.ordering.max_linear_speed;
  core.max_angular_speed_wmax = params.ordering.max_angular_speed;
  core.mrtsp_solver = params.ordering.solver;
  core.dp_solver_candidate_limit = static_cast<std::size_t>(params.ordering.dp.candidate_limit);
  core.dp_planning_horizon = static_cast<std::size_t>(params.ordering.dp.planning_horizon);
  core.occ_threshold = static_cast<int>(params.frontier.occ_threshold);
  core.min_frontier_size_cells = static_cast<int>(params.frontier.min_size_cells);
  core.frontier_candidate_min_goal_distance_m = params.frontier.candidate_min_goal_distance_m;
  core.frontier_selection_min_distance = params.frontier.selection_min_distance;
  core.escape_enabled = params.escape.enabled;
  core.frontier_visit_tolerance = params.frontier.visit_tolerance;
  core.goal_preemption_enabled = params.preemption.enabled;
  core.goal_skip_on_blocked_goal = params.preemption.skip_on_blocked_goal;
  core.goal_preemption_min_interval_s = params.preemption.min_interval_s;
  core.goal_preemption_lidar_range_m = params.preemption.lidar.range_m;
  core.goal_preemption_lidar_fov_deg = params.preemption.lidar.fov_deg;
  core.goal_preemption_lidar_ray_step_deg = params.preemption.lidar.ray_step_deg;
  core.goal_preemption_complete_if_within_m = params.preemption.complete_if_within_m;
  core.goal_preemption_lidar_min_reveal_length_m = params.preemption.lidar.min_reveal_length_m;
  core.goal_preemption_lidar_yaw_offset_deg = params.preemption.lidar.yaw_offset_deg;
  core.post_goal_settle_enabled = params.post_goal_settle.enabled;
  core.post_goal_min_settle = params.post_goal_settle.min_settle_s;
  core.map_processing_rate_hz = params.map_processing.rate_hz;
  core.return_to_start_on_complete = params.completion.return_to_start;
  core.all_frontiers_suppressed_behavior = params.completion.all_suppressed_behavior;
  core.frontier_suppression_enabled = params.suppression.enabled;
  core.frontier_suppression_attempt_threshold = static_cast<int>(params.suppression.attempt_threshold);
  core.frontier_suppression_base_size_m = params.suppression.base_size_m;
  core.frontier_suppression_expansion_size_m = params.suppression.expansion_size_m;
  core.frontier_suppression_timeout_s = params.suppression.timeout_s;
  core.frontier_suppression_no_progress_timeout_s = params.suppression.no_progress_timeout_s;
  core.frontier_suppression_progress_epsilon_m = params.suppression.progress_epsilon_m;
  core.frontier_suppression_startup_grace_period_s = params.suppression.startup_grace_period_s;
  core.frontier_suppression_max_attempt_records = static_cast<int>(params.suppression.max_attempt_records);
  core.frontier_suppression_max_regions = static_cast<int>(params.suppression.max_regions);
  return core;
}

}  // namespace

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer", options)
{
  const auto params = frontier_explorer_params::ParamListener(
    this->get_node_parameters_interface(), this->get_logger()).get_params();
  params_ = toCoreParams(params);
  autostart_ = params.control.autostart;
  control_service_enabled_ = params.control.service_enabled;
  if (!autostart_ && !control_service_enabled_) {
    control_service_enabled_ = true;
    RCLCPP_WARN(
      this->get_logger(),
      "control.service_enabled=false is ignored when control.autostart=false; control service remains enabled");
  }
  completion_event_config_.enabled = params.completion.event_enabled;
  completion_event_config_.topic = params.topics.completion_event;

  try {
    // Parse and validate QoS strings once at startup for explicit, predictable runtime profiles.
    topic_qos_profiles_ = resolve_topic_qos_profiles(
      params.qos.map.durability,
      params.qos.map.reliability,
      params.qos.map.depth,
      params.qos.costmap.reliability,
      params.qos.costmap.depth,
      params.qos.local_costmap.reliability,
      params.qos.local_costmap.depth);
  } catch (const std::exception & exc) {
    throw std::runtime_error(std::string("Invalid QoS parameter configuration: ") + exc.what());
  }

  map_qos_autodetect_on_startup_ = params.qos.map.autodetect_on_startup;
  map_qos_autodetect_timeout_s_ = params.qos.map.autodetect_timeout_s;

  navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    params_.navigate_to_pose_action_name);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (completion_event_config_.enabled) {
    auto completion_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    completion_qos.reliable();
    completion_qos.transient_local();
    completion_event_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      completion_event_config_.topic,
      completion_qos);
  }
  frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.frontier_marker_topic,
    10);
  selected_frontier_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    params_.selected_frontier_topic,
    10);
  optimized_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    params_.optimized_map_topic,
    10);
  if (control_service_enabled_) {
    control_service_ = this->create_service<srv::ControlExploration>(
      "control_exploration",
      std::bind(
        &FrontierExplorerNode::handleControlRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    RCLCPP_INFO(
      this->get_logger(),
      "Control service ready: '%s'",
      control_service_->get_service_name());
  } else {
    RCLCPP_INFO(this->get_logger(), "Control service is disabled by configuration");
  }

  // Core callbacks keep core logic independent from ROS transport and threading details.
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [this]() {return this->get_clock()->now().nanoseconds();};
  callbacks.get_current_pose = [this]() {return this->getCurrentPose();};
  callbacks.wait_for_action_server = [this](double timeout_sec) {
      // Core uses a bounded wait to avoid hard-blocking in dispatch path.
      return navigate_to_pose_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(timeout_sec)));
    };
  callbacks.dispatch_goal_request = [this](const GoalDispatchRequest & request) {
      this->dispatchGoalRequest(request);
    };
  callbacks.publish_frontier_markers = [this](const FrontierSequence & frontiers) {
      this->publishFrontierMarkers(frontiers);
    };
  callbacks.publish_selected_frontier_pose = [this](const geometry_msgs::msg::PoseStamped & pose) {
      this->publishSelectedFrontierPose(pose);
    };
  callbacks.publish_optimized_map = [this](const nav_msgs::msg::OccupancyGrid & map_msg) {
      this->publishOptimizedMap(map_msg);
    };
  callbacks.on_exploration_complete = [this]() {
      this->publishCompletionEvent();
    };
  callbacks.debug_outputs_enabled = [this]() {
      return this->debugOutputsEnabled();
    };
  callbacks.log_debug = [this](const std::string & message) {
      RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_info = [this](const std::string & message) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_warn = [this](const std::string & message) {
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_error = [this](const std::string & message) {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    };
  core_ = std::make_unique<FrontierExplorerCore>(params_, callbacks);
  runtime_state_ = RuntimeState::COLD_IDLE;
  if (!autostart_) {
    core_->stop_exploration_session("Frontier exploration initialized in cold idle mode");
  } else {
    startExplorationRuntime();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Frontier explorer initialized with autostart=%s, control_service_enabled=%s, map '%s', global costmap '%s', local costmap '%s', frontier action '%s'",
    autostart_ ? "true" : "false",
    control_service_enabled_ ? "true" : "false",
    params_.map_topic.c_str(),
    params_.costmap_topic.c_str(),
    params_.local_costmap_topic.c_str(),
    params_.navigate_to_pose_action_name.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Decision-map config: optimization=%s, sigma_s=%.2f, sigma_r=%.2f, dilation_radius=%d, occ_threshold=%d, min_frontier_size_cells=%d, frontier_candidate_min_goal_distance_m=%.2f, debug_outputs=%s",
    frontierMapOptimizationEnabled() ? "true" : "false",
    params_.sigma_s,
    params_.sigma_r,
    params_.dilation_kernel_radius_cells,
    params_.occ_threshold,
    params_.min_frontier_size_cells,
    params_.frontier_candidate_min_goal_distance_m,
    debugOutputsEnabled() ? "debug-log-level" : "disabled");
  RCLCPP_INFO(
    this->get_logger(),
    "MRTSP config: solver='%s', dp_candidate_limit=%zu, dp_planning_horizon=%zu, sensor_effective_range_m=%.2f, weight_distance_wd=%.2f, weight_gain_ws=%.2f, max_linear_speed_vmax=%.2f, max_angular_speed_wmax=%.2f",
    core_->params.mrtsp_solver.c_str(),
    core_->params.dp_solver_candidate_limit,
    core_->params.dp_planning_horizon,
    params_.sensor_effective_range_m,
    params_.weight_distance_wd,
    params_.weight_gain_ws,
    params_.max_linear_speed_vmax,
    params_.max_angular_speed_wmax);
  RCLCPP_INFO(
    this->get_logger(),
    "Using post-goal settle config: enabled=%s, post_goal_min_settle=%.2fs, map_processing_rate_hz=%.2f, all_frontiers_suppressed_behavior=%s",
    params_.post_goal_settle_enabled ? "true" : "false",
    params_.post_goal_min_settle,
    params_.map_processing_rate_hz,
    params_.all_frontiers_suppressed_behavior.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "QoS config: map=[durability=%s,reliability=%s,depth=%zu], "
    "costmap=[durability=volatile,reliability=%s,depth=%zu], "
    "local_costmap=[durability=volatile,reliability=%s,depth=%zu%s%s]",
    durability_policy_to_string(topic_qos_profiles_.map_durability).c_str(),
    reliability_policy_to_string(topic_qos_profiles_.map_reliability).c_str(),
    topic_qos_profiles_.map_depth,
    reliability_policy_to_string(topic_qos_profiles_.costmap_reliability).c_str(),
    topic_qos_profiles_.costmap_depth,
    reliability_policy_to_string(topic_qos_profiles_.local_costmap_reliability).c_str(),
    topic_qos_profiles_.local_costmap_depth,
    topic_qos_profiles_.local_costmap_reliability_inherited ? " (inherit reliability)" : "",
    topic_qos_profiles_.local_costmap_depth_inherited ? " (inherit depth)" : "");
  if (completion_event_config_.enabled) {
    RCLCPP_INFO(
      this->get_logger(),
      "Completion event enabled: topic='%s'",
      completion_event_config_.topic.c_str());
  }
  if (!autostart_ && control_service_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Explorer is in cold idle. Send a start request via the '%s' service or use "
      "'frontier_exploration_ctl start' or "
      "'ros2 run frontier_exploration_ros2 frontier_exploration_ctl start'.",
      control_service_->get_service_name());
  }
  if (params_.frontier_suppression_enabled) {
    RCLCPP_INFO(
      this->get_logger(),
      "Frontier suppression enabled: threshold=%d, base=%.2fm, expansion=%.2fm, timeout=%.2fs, no_progress_timeout=%.2fs, startup_grace=%.2fs, max_attempts=%d, max_regions=%d",
      params_.frontier_suppression_attempt_threshold,
      params_.frontier_suppression_base_size_m,
      params_.frontier_suppression_expansion_size_m,
      params_.frontier_suppression_timeout_s,
      params_.frontier_suppression_no_progress_timeout_s,
      params_.frontier_suppression_startup_grace_period_s,
      params_.frontier_suppression_max_attempt_records,
      params_.frontier_suppression_max_regions);
  }
}

FrontierExplorerNode::~FrontierExplorerNode()
{
  if (core_) {
    core_->request_shutdown();
  }
}

bool FrontierExplorerNode::hasActiveExplorationSubscriptions() const
{
  return static_cast<bool>(map_sub_) &&
         static_cast<bool>(costmap_sub_) &&
         static_cast<bool>(local_costmap_sub_);
}

bool FrontierExplorerNode::hasControlService() const
{
  return static_cast<bool>(control_service_);
}

bool FrontierExplorerNode::quitRequested() const
{
  return quit_requested_;
}

void FrontierExplorerNode::createMapSubscription(rclcpp::DurabilityPolicy map_durability)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.map_topic,
    topic_qos_profiles_.make_map_qos(map_durability),
    std::bind(&FrontierExplorerNode::occupancyGridCallback, this, std::placeholders::_1));
  // Reassigning map_sub_ replaces previous subscription instance.
}

void FrontierExplorerNode::ensureMapProcessingTimer()
{
  if (params_.map_processing_rate_hz <= 0.0) {
    return;
  }

  if (!effective_map_processing_rate_hz_.has_value() || *effective_map_processing_rate_hz_ <= 0.0) {
    return;
  }

  if (map_processing_timer_) {
    return;
  }

  map_processing_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / *effective_map_processing_rate_hz_)),
    std::bind(&FrontierExplorerNode::mapProcessingTimerCallback, this));
}

void FrontierExplorerNode::resetMapProcessingCalibrationWindow()
{
  last_map_arrival_at_.reset();
  startup_map_interval_samples_s_.clear();
}

bool FrontierExplorerNode::maybeFinalizeMapProcessingRateEstimate()
{
  if (params_.map_processing_rate_hz <= 0.0) {
    return false;
  }

  if (effective_map_processing_rate_hz_.has_value()) {
    return false;
  }

  if (startup_map_interval_samples_s_.size() < kStartupMapRateSampleCount) {
    return false;
  }

  auto sorted_samples = startup_map_interval_samples_s_;
  const auto middle = sorted_samples.begin() + static_cast<std::ptrdiff_t>(sorted_samples.size() / 2U);
  std::nth_element(sorted_samples.begin(), middle, sorted_samples.end());
  const double median_interval_s = *middle;
  const auto [min_it, max_it] = std::minmax_element(sorted_samples.begin(), sorted_samples.end());
  const bool invalid_interval = median_interval_s <= 0.0 || !std::isfinite(median_interval_s);
  const bool unstable_samples =
    !invalid_interval &&
    *min_it > 0.0 &&
    ((*max_it - *min_it) / median_interval_s) > kStartupMapRateStabilityTolerance;

  if (invalid_interval || unstable_samples) {
    effective_map_processing_rate_hz_ = params_.map_processing_rate_hz;
    if (unstable_samples) {
      RCLCPP_WARN(
        this->get_logger(),
        "Startup /map timing samples were unstable; using configured map_processing_rate_hz=%.2f Hz",
        params_.map_processing_rate_hz);
    }
  } else {
    const double observed_hz = 1.0 / median_interval_s;
    const double capped_observed_hz = observed_hz >= 1.0 ? std::floor(observed_hz) : observed_hz;
    effective_map_processing_rate_hz_ = std::min(
      params_.map_processing_rate_hz,
      std::max(0.1, capped_observed_hz));
    RCLCPP_INFO(
      this->get_logger(),
      "Calibrated /map processing rate from startup samples: observed=%.2f Hz, configured=%.2f Hz, effective=%.2f Hz",
      observed_hz,
      params_.map_processing_rate_hz,
      *effective_map_processing_rate_hz_);
  }

  ensureMapProcessingTimer();
  return true;
}

void FrontierExplorerNode::startExplorationRuntime()
{
  completion_event_published_ = false;
  pending_quit_after_stop_ = false;
  quit_requested_ = false;
  runtime_state_ = RuntimeState::RUNNING;
  ensureDeferredShutdownTimerCanceled();
  ensureStopCompletionTimerCanceled();
  core_->start_exploration_session();

  if (params_.frontier_suppression_enabled) {
    suppression_activation_logged_ = false;
    suppression_activation_at_ =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(params_.frontier_suppression_startup_grace_period_s));
    if (params_.frontier_suppression_startup_grace_period_s <= 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Frontier suppression startup grace period elapsed; suppression is now active");
      suppression_activation_logged_ = true;
    }
    ensureWatchdogTimer();
  }

  createMapSubscription(topic_qos_profiles_.map_durability);
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.costmap_topic,
    topic_qos_profiles_.make_costmap_qos(),
    std::bind(&FrontierExplorerNode::costmapCallback, this, std::placeholders::_1));
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.local_costmap_topic,
    topic_qos_profiles_.make_local_costmap_qos(),
    std::bind(&FrontierExplorerNode::localCostmapCallback, this, std::placeholders::_1));
  if (params_.map_processing_rate_hz > 0.0) {
    if (effective_map_processing_rate_hz_.has_value()) {
      ensureMapProcessingTimer();
    } else {
      resetMapProcessingCalibrationWindow();
    }
  }

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    map_received_once_ = false;
    map_autodetect_complete_logged_ = false;
    if (map_qos_autodetect_on_startup_) {
      map_qos_autodetect_ = MapQosStartupAutodetect(true, topic_qos_profiles_.map_durability);
      map_autodetect_started_at_ = std::chrono::steady_clock::now();
    } else {
      map_qos_autodetect_.reset();
    }
  }

  if (map_qos_autodetect_on_startup_) {
    map_autodetect_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(map_qos_autodetect_timeout_s_)),
      std::bind(&FrontierExplorerNode::mapAutodetectTimeoutCallback, this));
    logMapAutodetectStart(topic_qos_profiles_.map_durability);
  }
}

void FrontierExplorerNode::ensureWatchdogTimer()
{
  if (!params_.frontier_suppression_enabled || suppression_watchdog_timer_) {
    return;
  }

  const double watchdog_period_s = std::clamp(
    params_.frontier_suppression_no_progress_timeout_s / 4.0,
    0.25,
    1.0);
  suppression_watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(watchdog_period_s)),
    std::bind(&FrontierExplorerNode::suppressionWatchdogCallback, this));
}

void FrontierExplorerNode::enterColdIdle()
{
  runtime_state_ = RuntimeState::COLD_IDLE;
  map_sub_.reset();
  costmap_sub_.reset();
  local_costmap_sub_.reset();
  map_autodetect_timer_.reset();
  map_processing_timer_.reset();
  suppression_watchdog_timer_.reset();
  suppression_activation_logged_ = false;
  suppression_activation_at_.reset();
  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    map_qos_autodetect_.reset();
    map_received_once_ = false;
    map_autodetect_complete_logged_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    latest_pending_map_msg_.reset();
    pending_map_update_ = false;
  }
}

void FrontierExplorerNode::ensureControlTimerCanceled()
{
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  scheduled_control_request_.reset();
}

void FrontierExplorerNode::ensureStopCompletionTimerCanceled()
{
  if (stop_completion_timer_) {
    stop_completion_timer_->cancel();
    stop_completion_timer_.reset();
  }
}

void FrontierExplorerNode::ensureDeferredShutdownTimerCanceled()
{
  if (deferred_shutdown_timer_) {
    deferred_shutdown_timer_->cancel();
    deferred_shutdown_timer_.reset();
  }
}

uint8_t FrontierExplorerNode::controlState() const
{
  if (scheduled_control_request_.has_value()) {
    return scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_START ?
           srv::ControlExploration::Request::STATE_START_SCHEDULED :
           srv::ControlExploration::Request::STATE_STOP_SCHEDULED;
  }

  switch (runtime_state_) {
    case RuntimeState::RUNNING:
      return srv::ControlExploration::Request::STATE_RUNNING;
    case RuntimeState::STOPPING:
      return srv::ControlExploration::Request::STATE_STOPPING;
    case RuntimeState::SHUTDOWN_PENDING:
      return srv::ControlExploration::Request::STATE_SHUTDOWN_PENDING;
    case RuntimeState::COLD_IDLE:
    default:
      return srv::ControlExploration::Request::STATE_IDLE;
  }
}

std::string FrontierExplorerNode::controlStateMessage() const
{
  switch (controlState()) {
    case srv::ControlExploration::Request::STATE_RUNNING:
      return "running";
    case srv::ControlExploration::Request::STATE_START_SCHEDULED:
      return "start scheduled";
    case srv::ControlExploration::Request::STATE_STOP_SCHEDULED:
      return "stop scheduled";
    case srv::ControlExploration::Request::STATE_STOPPING:
      return "stopping";
    case srv::ControlExploration::Request::STATE_SHUTDOWN_PENDING:
      return "shutdown pending";
    case srv::ControlExploration::Request::STATE_IDLE:
    default:
      return "idle";
  }
}

void FrontierExplorerNode::scheduleControlRequest(
  uint8_t action,
  double delay_seconds,
  bool quit_after_stop)
{
  ensureControlTimerCanceled();
  scheduled_control_request_ = ScheduledControlRequest{action, quit_after_stop};
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay_seconds)),
    std::bind(&FrontierExplorerNode::controlTimerCallback, this));
}

void FrontierExplorerNode::requestStopExplorationRuntime(
  bool quit_after_stop,
  const std::string & reason)
{
  ensureControlTimerCanceled();
  pending_quit_after_stop_ = quit_after_stop;
  core_->stop_exploration_session(reason);
  publishFrontierMarkers({});
  enterColdIdle();

  if (core_->ready_for_shutdown()) {
    if (pending_quit_after_stop_) {
      runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
      deferred_shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&FrontierExplorerNode::deferredShutdownCallback, this));
    }
    return;
  }

  runtime_state_ = RuntimeState::STOPPING;
  stop_completion_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&FrontierExplorerNode::stopCompletionPollCallback, this));
}

void FrontierExplorerNode::handleControlRequest(
  const std::shared_ptr<srv::ControlExploration::Request> request,
  std::shared_ptr<srv::ControlExploration::Response> response)
{
  const double delay_seconds = static_cast<double>(request->delay_seconds);
  const bool start_is_scheduled =
    scheduled_control_request_.has_value() &&
    scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_START;
  const bool stop_is_scheduled =
    scheduled_control_request_.has_value() &&
    scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_STOP;
  if (delay_seconds < 0.0) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "delay_seconds must be non-negative";
    return;
  }

  if (request->quit_after_stop && request->action != srv::ControlExploration::Request::ACTION_STOP) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "quit_after_stop is only valid for stop requests";
    return;
  }

  if (
    request->action != srv::ControlExploration::Request::ACTION_START &&
    request->action != srv::ControlExploration::Request::ACTION_STOP)
  {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Unsupported control action";
    return;
  }

  if (request->action == srv::ControlExploration::Request::ACTION_START) {
    if (delay_seconds <= 0.0) {
      // Immediate start is the strongest operator intent and should clear any stale timer.
      ensureControlTimerCanceled();
    }

    if (runtime_state_ == RuntimeState::RUNNING) {
      response->accepted = delay_seconds <= 0.0;
      response->scheduled = false;
      response->state = controlState();
      response->message = delay_seconds <= 0.0 && stop_is_scheduled ?
        "Exploration is already running; cleared scheduled stop" :
        delay_seconds <= 0.0 ?
        "Exploration is already running" :
        "Cannot schedule a future start while exploration is already running";
      return;
    }
    if (runtime_state_ == RuntimeState::STOPPING || runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
      response->accepted = false;
      response->scheduled = false;
      response->state = controlState();
      response->message = "Exploration is stopping or shutting down";
      return;
    }
    if (delay_seconds > 0.0) {
      scheduleControlRequest(request->action, delay_seconds, false);
      response->accepted = true;
      response->scheduled = true;
      response->state = controlState();
      response->message = "Scheduled exploration start";
      return;
    }

    startExplorationRuntime();
    response->accepted = true;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Exploration started";
    return;
  }

  if (delay_seconds > 0.0) {
    if (runtime_state_ == RuntimeState::COLD_IDLE && !start_is_scheduled) {
      response->accepted = false;
      response->scheduled = false;
      response->state = controlState();
      response->message = "Cannot schedule a stop while exploration is idle";
      return;
    }

    scheduleControlRequest(request->action, delay_seconds, request->quit_after_stop);
    response->accepted = true;
    response->scheduled = true;
    response->state = controlState();
    response->message = "Scheduled exploration stop";
    return;
  }

  if (runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Shutdown is already pending";
    return;
  }

  requestStopExplorationRuntime(
    request->quit_after_stop,
    request->quit_after_stop ?
    "Stopping exploration and shutting down the node" :
    "Stopping exploration");
  response->accepted = true;
  response->scheduled = false;
  response->state = controlState();
  response->message = request->quit_after_stop ?
    "Stopping exploration and preparing node shutdown" :
    "Stopping exploration";
}

void FrontierExplorerNode::controlTimerCallback()
{
  if (!scheduled_control_request_.has_value()) {
    ensureControlTimerCanceled();
    return;
  }

  const ScheduledControlRequest scheduled_request = *scheduled_control_request_;
  ensureControlTimerCanceled();

  if (scheduled_request.action == srv::ControlExploration::Request::ACTION_START) {
    if (runtime_state_ == RuntimeState::COLD_IDLE) {
      startExplorationRuntime();
    }
    return;
  }

  if (runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
    return;
  }

  requestStopExplorationRuntime(
    scheduled_request.quit_after_stop,
    scheduled_request.quit_after_stop ?
    "Stopping exploration and shutting down the node" :
    "Stopping exploration");
}

void FrontierExplorerNode::stopCompletionPollCallback()
{
  if (!core_ || !core_->ready_for_shutdown()) {
    return;
  }

  ensureStopCompletionTimerCanceled();
  if (pending_quit_after_stop_) {
    runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
    deferred_shutdown_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&FrontierExplorerNode::deferredShutdownCallback, this));
    return;
  }

  runtime_state_ = RuntimeState::COLD_IDLE;
}

void FrontierExplorerNode::deferredShutdownCallback()
{
  ensureDeferredShutdownTimerCanceled();
  runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
  quit_requested_ = true;
}

void FrontierExplorerNode::mapAutodetectTimeoutCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  // Work in two phases:
  //   1) Decide state transition under lock.
  //   2) Perform ROS side effects (re-subscribe / log) outside lock.
  std::optional<rclcpp::DurabilityPolicy> switched_durability;
  bool should_log_failed_completion = false;
  rclcpp::DurabilityPolicy selected_durability = topic_qos_profiles_.map_durability;

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    if (
      !map_qos_autodetect_.has_value() ||
      map_autodetect_complete_logged_ ||
      !map_qos_autodetect_->active() ||
      map_received_once_)
    {
      return;
    }

    switched_durability = map_qos_autodetect_->on_timeout();
    // Read active durability after transition so logs and subscription stay consistent.
    selected_durability = map_qos_autodetect_->active_durability();
    if (!switched_durability.has_value() && !map_qos_autodetect_->active()) {
      // No more fallback attempts remain; emit terminal failed completion once.
      map_autodetect_complete_logged_ = true;
      should_log_failed_completion = true;
    }
  }

  if (switched_durability.has_value()) {
    // Re-create map subscription with fallback durability for the second and final attempt.
    createMapSubscription(*switched_durability);
    logMapAutodetectSwitch(*switched_durability);
    return;
  }

  if (should_log_failed_completion) {
    if (map_autodetect_timer_) {
      // Cancel timer after terminal state to keep autodetect strictly startup-only.
      map_autodetect_timer_->cancel();
    }
    logMapAutodetectComplete("failed", selected_durability);
  }
}

void FrontierExplorerNode::logMapAutodetectStart(rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  // One-shot startup marker for log filtering.
  RCLCPP_INFO(
    this->get_logger(),
    "[qos-autodetect] START selected=%s timeout=%.2fs",
    durability_policy_to_string(selected_durability).c_str(),
    map_qos_autodetect_timeout_s_);
}

void FrontierExplorerNode::logMapAutodetectSwitch(rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  // Warn-level by design: initial durability did not match publisher QoS.
  RCLCPP_WARN(
    this->get_logger(),
    "[qos-autodetect] SWITCH selected=%s elapsed=%.2fs",
    durability_policy_to_string(selected_durability).c_str(),
    mapAutodetectElapsedSeconds());
}

void FrontierExplorerNode::logMapAutodetectComplete(
  const std::string & result,
  rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  const std::string selected = durability_policy_to_string(selected_durability);
  const double elapsed = mapAutodetectElapsedSeconds();
  if (result == "failed") {
    // Failed means neither initial nor fallback durability received map within timeout windows.
    RCLCPP_WARN(
      this->get_logger(),
      "[qos-autodetect] COMPLETE result=%s selected=%s elapsed=%.2fs",
      result.c_str(),
      selected.c_str(),
      elapsed);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[qos-autodetect] COMPLETE result=%s selected=%s elapsed=%.2fs",
    result.c_str(),
    selected.c_str(),
    elapsed);
}

double FrontierExplorerNode::mapAutodetectElapsedSeconds() const
{
  if (!map_qos_autodetect_on_startup_) {
    return 0.0;
  }
  // Steady clock keeps elapsed time robust against ROS/system clock jumps.
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - map_autodetect_started_at_).count();
}

void FrontierExplorerNode::mapProcessingTimerCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING || !core_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    pending_map_update_ = false;
    latest_pending_map_msg_.reset();
  }

  core_->processPendingMapUpdate();
}

void FrontierExplorerNode::suppressionWatchdogCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  if (
    !suppression_activation_logged_ &&
    suppression_activation_at_.has_value() &&
    std::chrono::steady_clock::now() >= *suppression_activation_at_)
  {
    RCLCPP_INFO(this->get_logger(), "Frontier suppression startup grace period elapsed; suppression is now active");
    suppression_activation_logged_ = true;
  }

  if (core_) {
    core_->evaluate_active_goal_progress_timeout();
  }
}

void FrontierExplorerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  bool should_log_complete = false;
  std::string complete_result;
  rclcpp::DurabilityPolicy selected_durability = topic_qos_profiles_.map_durability;

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    if (!map_received_once_) {
      // Autodetect is startup-only; first valid map finalizes the selected durability.
      map_received_once_ = true;
      if (map_qos_autodetect_.has_value() && !map_autodetect_complete_logged_) {
        complete_result = map_qos_autodetect_->fallback_attempted() ? "fallback" : "initial";
        selected_durability = map_qos_autodetect_->active_durability();
        map_qos_autodetect_->on_map_received();
        map_autodetect_complete_logged_ = true;
        should_log_complete = true;
      }
    }
  }

  if (should_log_complete && map_autodetect_timer_) {
    // COMPLETE log is emitted exactly once when first map is accepted.
    map_autodetect_timer_->cancel();
    logMapAutodetectComplete(complete_result, selected_durability);
  }

  const OccupancyGrid2d map_msg(msg);
  if (params_.map_processing_rate_hz <= 0.0) {
    core_->occupancyGridCallback(map_msg);
    return;
  }

  const auto arrival_at = std::chrono::steady_clock::now();
  if (!effective_map_processing_rate_hz_.has_value()) {
    if (last_map_arrival_at_.has_value()) {
      const double interval_s = std::chrono::duration<double>(arrival_at - *last_map_arrival_at_).count();
      if (interval_s > 0.0) {
        startup_map_interval_samples_s_.push_back(interval_s);
      }
    }
    last_map_arrival_at_ = arrival_at;
  }

  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    latest_pending_map_msg_ = msg;
    pending_map_update_ = true;
  }

  core_->ingestRawMapUpdate(map_msg);
  maybeFinalizeMapProcessingRateEstimate();
  //   core_->handleUrgentRawMapUpdateForActiveGoal();
  // if (maybeFinalizeMapProcessingRateEstimate()) {
  //   mapProcessingTimerCallback();
  // }
}

void FrontierExplorerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }
  core_->costmapCallback(OccupancyGrid2d(msg));
}

void FrontierExplorerNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }
  core_->localCostmapCallback(OccupancyGrid2d(msg));
}

void FrontierExplorerNode::publishCompletionEvent()
{
  if (!completion_event_config_.enabled || !completion_event_pub_) {
    // Completion publishing stays dormant unless explicitly configured.
    return;
  }
  if (completion_event_published_) {
    // Frontier exhaustion may be observed repeatedly while the node remains alive.
    return;
  }

  completion_event_published_ = true;
  completion_event_pub_->publish(std_msgs::msg::Empty{});
  RCLCPP_INFO(
    this->get_logger(),
    "Published exploration completion event on '%s'",
    completion_event_config_.topic.c_str());
}

std::optional<geometry_msgs::msg::Pose> FrontierExplorerNode::getCurrentPose()
{
  try {
    // Bounded TF lookup keeps scheduler responsive under frame delays.
    const auto transform = tf_buffer_->lookupTransform(
      params_.global_frame,
      params_.robot_base_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));

    geometry_msgs::msg::Pose pose;
    // Convert transform directly to pose used by frontier/core decisions.
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return pose;
  } catch (const tf2::TransformException & exc) {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t throttle_ns = static_cast<int64_t>(tf_warning_throttle_seconds_ * 1e9);
    if (!last_tf_warning_time_ns_.has_value() || now_ns - *last_tf_warning_time_ns_ >= throttle_ns) {
      // Throttle repeated TF warnings during map startup or transient frame drops.
      last_tf_warning_time_ns_ = now_ns;
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform %s -> %s: %s",
        params_.robot_base_frame.c_str(),
        params_.global_frame.c_str(),
        exc.what());
    }
    return std::nullopt;
  }
}

void FrontierExplorerNode::publishFrontierMarkers(const FrontierSequence & frontiers)
{
  visualization_msgs::msg::MarkerArray marker_array;
  const auto publish_stamp = this->now();

  visualization_msgs::msg::Marker clear_marker;
  // Always clear previous marker namespace first to avoid stale points in RViz.
  clear_marker.header.frame_id = params_.global_frame;
  clear_marker.header.stamp = publish_stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (!frontiers.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.global_frame;
    marker.header.stamp = publish_stamp;
    marker.ns = "frontier_frontiers_primitive";
    marker.id = 0;
    // POINTS renders as screen-aligned squares in RViz, matching the MRTSP reference package.
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.scale.x = params_.frontier_marker_scale;
    marker.scale.y = params_.frontier_marker_scale;
    marker.color.a = 1.0;
    marker.color.r = 0.15;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    // Color/namespace choices stay stable for predictable RViz overlays and MRTSP parity.

    for (const auto & frontier : frontiers) {
      // Marker position reflects selected goal point for each frontier entry.
      const auto [frontier_x, frontier_y] = frontier_position(frontier);
      geometry_msgs::msg::Point point;
      point.x = frontier_x;
      point.y = frontier_y;
      point.z = 0.0;
      marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
  }

  frontier_marker_pub_->publish(marker_array);
}

void FrontierExplorerNode::publishSelectedFrontierPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!debugOutputsEnabled() || !selected_frontier_pub_) {
    return;
  }
  selected_frontier_pub_->publish(pose);
}

void FrontierExplorerNode::publishOptimizedMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  if (!debugOutputsEnabled() || !optimized_map_pub_) {
    return;
  }
  optimized_map_pub_->publish(map_msg);
}

bool FrontierExplorerNode::frontierMapOptimizationEnabled() const
{
  return params_.frontier_map_optimization_enabled;
}

bool FrontierExplorerNode::debugOutputsEnabled() const
{
  return rcutils_logging_logger_is_enabled_for(
    this->get_logger().get_name(),
    RCUTILS_LOG_SEVERITY_DEBUG);
}

void FrontierExplorerNode::dispatchGoalRequest(const GoalDispatchRequest & request)
{
  NavigateToPose::Goal goal_request;
  // Core provides fully prepared pose/action metadata in request.
  goal_request.pose = request.goal_pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, dispatch_id = request.dispatch_id](
    std::shared_ptr<NavigateGoalHandle> goal_handle)
    {
      std::shared_ptr<GoalHandleInterface> wrapped_handle;
      bool accepted = static_cast<bool>(goal_handle);
      if (accepted) {
        // Wrap Nav2 handle so core can request cancel through transport-agnostic interface.
        wrapped_handle = std::make_shared<NavigateGoalHandleAdapter>(
          goal_handle,
          [this](
            const std::shared_ptr<NavigateGoalHandle> & active_goal_handle,
            std::function<void(bool accepted, const std::string & error_message)> callback)
          {
            try {
              navigate_to_pose_client_->async_cancel_goal(
                active_goal_handle,
                [callback = std::move(callback)](
                  action_msgs::srv::CancelGoal::Response::SharedPtr response) mutable
                {
                  const bool accepted = response && !response->goals_canceling.empty();
                  callback(accepted, "");
                });
            } catch (const std::exception & exc) {
              callback(false, exc.what());
            }
          });
      }

      core_->goal_response_callback(
        dispatch_id,
        wrapped_handle,
        accepted,
        accepted ? "" : "Frontier goal was rejected");
    };

  options.feedback_callback = [this, dispatch_id = request.dispatch_id](
    std::shared_ptr<NavigateGoalHandle>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }
      core_->feedback_callback(feedback->distance_remaining, dispatch_id);
    };

  options.result_callback = [this, dispatch_id = request.dispatch_id](
    const NavigateGoalHandle::WrappedResult & wrapped_result)
    {
      const int status = mapResultCodeToGoalStatus(wrapped_result.code);
      int error_code = 0;
      std::string error_msg;
      if (wrapped_result.result) {
        // Nav2 result fields differ across ROS distros; extract when available.
        error_code = compat::extractNav2ResultErrorCode(*wrapped_result.result);
        error_msg = compat::extractNav2ResultErrorMessage(*wrapped_result.result);
      }

      core_->get_result_callback(
        dispatch_id,
        status,
        error_code,
        error_msg);
    };

  try {
    navigate_to_pose_client_->async_send_goal(goal_request, options);
  } catch (const std::exception & exc) {
    core_->goal_response_callback(
      request.dispatch_id,
      nullptr,
      false,
      std::string("Failed to send frontier goal: ") + exc.what());
  }
}

int FrontierExplorerNode::mapResultCodeToGoalStatus(rclcpp_action::ResultCode code)
{
  // Normalize transport-specific result codes into GoalStatus values expected by core.
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    case rclcpp_action::ResultCode::ABORTED:
      return action_msgs::msg::GoalStatus::STATUS_ABORTED;
    case rclcpp_action::ResultCode::CANCELED:
      return action_msgs::msg::GoalStatus::STATUS_CANCELED;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }
}

}  // namespace frontier_exploration_ros2
