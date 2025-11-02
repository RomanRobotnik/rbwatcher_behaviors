#include "rbwatcher_behaviors/behavior_tree_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/basic_types.h>

#include "rbwatcher_behaviors/actions/follow_waypoints_action.hpp"
#include "rbwatcher_behaviors/actions/navigate_to_pose_action.hpp"
#include "rbwatcher_behaviors/actions/ptz_tracker_action.hpp"
#include "rbwatcher_behaviors/actions/trigger_service_action.hpp"
#include "rbwatcher_behaviors/conditions/bool_topic_condition.hpp"

namespace rbwatcher_behaviors
{

BehaviorTreeActionServer::BehaviorTreeActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("behavior_tree_action_server", options)
{
  std::string default_tree_candidate;
  try {
    default_tree_candidate = ament_index_cpp::get_package_share_directory("rbwatcher_behaviors") + "/config/default_tree.xml";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to locate default tree file: %s", ex.what());
  }

  declare_parameter<std::string>("default_tree_path", default_tree_candidate);
  declare_parameter<double>("bt_tick_frequency", 20.0);

  default_tree_path_ = get_parameter("default_tree_path").as_string();
  tick_frequency_hz_ = get_parameter("bt_tick_frequency").as_double();
  tick_frequency_hz_ = std::max(1e-3, tick_frequency_hz_);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<ExecuteBehaviorTree>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "execute_behavior_tree",
    std::bind(&BehaviorTreeActionServer::handleGoal, this, _1, _2),
    std::bind(&BehaviorTreeActionServer::handleCancel, this, _1),
    std::bind(&BehaviorTreeActionServer::handleAccepted, this, _1));
}

rclcpp_action::GoalResponse BehaviorTreeActionServer::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteBehaviorTree::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received ExecuteBehaviorTree goal. tree_path='%s' xml_size=%zu",
    goal->tree_path.c_str(), goal->tree_xml.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BehaviorTreeActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Cancel request received for ExecuteBehaviorTree");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BehaviorTreeActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread(&BehaviorTreeActionServer::execute, this, goal_handle).detach();
}

void BehaviorTreeActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecuteBehaviorTree::Feedback>();
  auto result = std::make_shared<ExecuteBehaviorTree::Result>();

  std::string tree_xml;
  try {
    tree_xml = loadTree(*goal);
  } catch (const std::exception & ex) {
    result->success = false;
    result->error_message = ex.what();
    RCLCPP_ERROR(get_logger(), "Failed to load behavior tree: %s", ex.what());
    goal_handle->abort(result);
    return;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", shared_from_this());
  setInitialBlackboardValues(*goal, blackboard);

  // This is now a std::unique_ptr<BT::BehaviorTreeFactory>
  auto factory = createFactory(); 
  
  BT::Tree tree;
  try {
    // Use -> instead of .
    tree = factory->createTreeFromText(tree_xml, blackboard);
  } catch (const std::exception & ex) {
    result->success = false;
    result->error_message = ex.what();
    RCLCPP_ERROR(get_logger(), "Failed to instantiate behavior tree: %s", ex.what());
    goal_handle->abort(result);
    return;
  }
  /*auto blackboard = BT::Blackboard::create();
  blackboard->set("node", shared_from_this());
  setInitialBlackboardValues(*goal, blackboard);

  BT::BehaviorTreeFactory factory = createFactory();
  BT::Tree tree;
  try {
    tree = factory.createTreeFromText(tree_xml, blackboard);
  } catch (const std::exception & ex) {
    result->success = false;
    result->error_message = ex.what();
    RCLCPP_ERROR(get_logger(), "Failed to instantiate behavior tree: %s", ex.what());
    goal_handle->abort(result);
    return;
  }*/

  rclcpp::Rate loop_rate(tick_frequency_hz_);
  size_t tick_count = 0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      tree.haltTree();
      result->success = false;
      result->error_message = "Behavior tree execution canceled";
      goal_handle->canceled(result);
      return;
    }

    BT::NodeStatus status;
    try {
      status = tree.tickExactlyOnce();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Exception while ticking tree: %s", ex.what());
      result->success = false;
      result->error_message = ex.what();
      goal_handle->abort(result);
      return;
    }

    tick_count++;
    feedback->feedback_message = BT::toStr(status);
    if (status == BT::NodeStatus::RUNNING) {
      feedback->progress = std::min(0.99f, static_cast<float>(tick_count % 100) / 100.0f);
    } else if (status == BT::NodeStatus::SUCCESS) {
      feedback->progress = 1.0f;
    } else {
      feedback->progress = 0.0f;
    }
    goal_handle->publish_feedback(feedback);

    if (status == BT::NodeStatus::SUCCESS) {
      result->success = true;
      result->error_message.clear();
      goal_handle->succeed(result);
      return;
    }

    if (status == BT::NodeStatus::FAILURE) {
      result->success = false;
      result->error_message = "Behavior tree returned FAILURE";
      goal_handle->abort(result);
      return;
    }

    loop_rate.sleep();
  }

  result->success = false;
  result->error_message = "ROS context shutdown";
  goal_handle->abort(result);
}

std::string BehaviorTreeActionServer::loadTree(const ExecuteBehaviorTree::Goal & goal) const
{
  if (!goal.tree_xml.empty()) {
    return goal.tree_xml;
  }

  std::string requested_path;
  if (!goal.tree_path.empty()) {
    requested_path = goal.tree_path;
  } else {
    requested_path = default_tree_path_;
  }

  if (requested_path.empty()) {
    throw std::runtime_error("No behavior tree XML or path provided");
  }

  std::string resolved_path = requested_path;
  if (requested_path.front() != '/') {
    auto share_dir = ament_index_cpp::get_package_share_directory("rbwatcher_behaviors");
    resolved_path = share_dir + "/" + requested_path;
  }

  std::ifstream input(resolved_path);
  if (!input.is_open()) {
    throw std::runtime_error("Unable to open behavior tree file: " + resolved_path);
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

// Change the return type to std::unique_ptr
std::unique_ptr<BT::BehaviorTreeFactory> BehaviorTreeActionServer::createFactory() const
{
  // Create on the heap
  auto factory = std::make_unique<BT::BehaviorTreeFactory>();

  // Use -> instead of .
  factory->registerNodeType<FollowWaypointsAction>("FollowWaypoints");
  factory->registerNodeType<NavigateToPoseAction>("NavigateToPose");
  factory->registerNodeType<TriggerServiceAction>("TriggerService");
  factory->registerNodeType<PTZTrackerAction>("PTZTrackerAction");
  factory->registerNodeType<BoolTopicCondition>("BoolTopicCondition");
  
  return factory; // Return the unique_ptr
}

void BehaviorTreeActionServer::setInitialBlackboardValues(
  const ExecuteBehaviorTree::Goal & goal,
  BT::Blackboard::Ptr blackboard) const
{
  blackboard->set("target_pose", goal.target_pose);

  std::vector<geometry_msgs::msg::PoseStamped> waypoints = goal.target_waypoints;
  if (waypoints.empty() && !goal.target_pose.header.frame_id.empty()) {
    waypoints.push_back(goal.target_pose);
  }
  blackboard->set("target_waypoints", waypoints);

  std::uint32_t loops = goal.waypoint_loops > 0 ? goal.waypoint_loops : 1U;
  blackboard->set("waypoint_loops", loops);

  std::uint32_t start_index = goal.waypoint_start_index;
  if (start_index >= waypoints.size() && !waypoints.empty()) {
    start_index = 0U;
  }
  blackboard->set("waypoint_start_index", start_index);
}

}  // namespace rbwatcher_behaviors
