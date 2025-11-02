#include "rbwatcher_behaviors/actions/follow_waypoints_action.hpp"

#include <chrono>
#include <cstdint>

#include <behaviortree_cpp/blackboard.h>

namespace rbwatcher_behaviors
{

FollowWaypointsAction::FollowWaypointsAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  action_name_ = "/robot/follow_waypoints";
}

BT::PortsList FollowWaypointsAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
    BT::InputPort<std::uint32_t>("loops", 1U, "Number of times to repeat the waypoint list"),
    BT::InputPort<std::uint32_t>("start_index", 0U, "Index of the first waypoint to use"),
    BT::InputPort<std::string>("action_name", "/robot/follow_waypoints")
  };
}

BT::NodeStatus FollowWaypointsAction::onStart()
{
  resetGoalHandles();

  auto node = getRosNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("FollowWaypointsAction"), "Missing ROS node on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  auto waypoints_input = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
  if (!waypoints_input || waypoints_input->empty()) {
    RCLCPP_ERROR(node->get_logger(), "FollowWaypointsAction requires a non-empty waypoint list");
    return BT::NodeStatus::FAILURE;
  }

  auto loops_input = getInput<std::uint32_t>("loops");
  if (!loops_input) {
    RCLCPP_ERROR(node->get_logger(), "FollowWaypointsAction missing 'loops' input");
    return BT::NodeStatus::FAILURE;
  }

  auto start_index_input = getInput<std::uint32_t>("start_index");
  if (!start_index_input) {
    RCLCPP_ERROR(node->get_logger(), "FollowWaypointsAction missing 'start_index' input");
    return BT::NodeStatus::FAILURE;
  }

  auto requested_action_name = getInput<std::string>("action_name");
  if (requested_action_name && action_name_ != requested_action_name.value()) {
    action_client_.reset();
    action_name_ = requested_action_name.value();
  }

  auto client = getActionClient();
  if (!client) {
    RCLCPP_ERROR(node->get_logger(), "Unable to create action client for %s", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node->get_logger(), "FollowWaypoints action server %s not available", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  FollowWaypoints::Goal goal;
  goal.poses = waypoints_input.value();

  std::uint32_t sanitized_loops = loops_input.value();
  if (sanitized_loops == 0U) {
    sanitized_loops = 1U;
    RCLCPP_WARN(node->get_logger(), "FollowWaypointsAction received loops=0, defaulting to 1");
  }
  goal.number_of_loops = sanitized_loops;

  std::uint32_t sanitized_start_index = start_index_input.value();
  if (sanitized_start_index >= goal.poses.size()) {
    RCLCPP_WARN(node->get_logger(),
      "FollowWaypointsAction start_index=%u exceeds waypoint count=%zu, clamping to 0",
      sanitized_start_index, goal.poses.size());
    sanitized_start_index = 0U;
  }
  goal.goal_index = sanitized_start_index;

  goal_handle_future_ = client->async_send_goal(goal);
  if (!goal_handle_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send FollowWaypoints goal");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "FollowWaypoints goal with %zu waypoint(s) dispatched to %s",
    goal.poses.size(), action_name_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowWaypointsAction::onRunning()
{
  auto node = getRosNode();
  if (!node) {
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_) {
    if (goal_handle_future_.valid()) {
      auto status = goal_handle_future_.wait_for(std::chrono::milliseconds(0));
      if (status == std::future_status::ready) {
        goal_handle_ = goal_handle_future_.get();
        if (!goal_handle_) {
          RCLCPP_WARN(node->get_logger(), "FollowWaypoints goal rejected");
          return BT::NodeStatus::FAILURE;
        }
        auto client = getActionClient();
        if (!client) {
          RCLCPP_ERROR(node->get_logger(), "FollowWaypoints action client missing");
          return BT::NodeStatus::FAILURE;
        }
        result_future_ = client->async_get_result(goal_handle_);
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if (result_future_.valid()) {
    auto status = result_future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      auto wrapped_result = result_future_.get();
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (wrapped_result.result && !wrapped_result.result->missed_waypoints.empty()) {
            RCLCPP_WARN(node->get_logger(),
              "FollowWaypoints succeeded with %zu missed waypoint(s)",
              wrapped_result.result->missed_waypoints.size());
          } else {
            RCLCPP_INFO(node->get_logger(), "FollowWaypoints goal completed successfully");
          }
          return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::ABORTED:
          if (wrapped_result.result) {
            RCLCPP_WARN(node->get_logger(),
              "FollowWaypoints goal aborted (code=%u): %s",
              static_cast<unsigned>(wrapped_result.result->error_code),
              wrapped_result.result->error_msg.c_str());
          } else {
            RCLCPP_WARN(node->get_logger(), "FollowWaypoints goal aborted");
          }
          return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(node->get_logger(), "FollowWaypoints goal canceled");
          return BT::NodeStatus::FAILURE;
        default:
          RCLCPP_ERROR(node->get_logger(), "FollowWaypoints result unknown");
          return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

void FollowWaypointsAction::onHalted()
{
  if (goal_handle_ && getActionClient()) {
    (void)getActionClient()->async_cancel_goal(goal_handle_);
  }
  resetGoalHandles();
}

std::shared_ptr<rclcpp::Node> FollowWaypointsAction::getRosNode()
{
  if (auto node = cached_node_.lock()) {
    return node;
  }

  if (!config().blackboard) {
    return nullptr;
  }

  try {
    auto node = config().blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    cached_node_ = node;
    return node;
  } catch (const BT::RuntimeError &) {
    return nullptr;
  }
}

rclcpp_action::Client<FollowWaypointsAction::FollowWaypoints>::SharedPtr FollowWaypointsAction::getActionClient()
{
  auto node = getRosNode();
  if (!node) {
    return nullptr;
  }

  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<FollowWaypointsAction::FollowWaypoints>(node, action_name_);
  }

  return action_client_;
}

void FollowWaypointsAction::resetGoalHandles()
{
  goal_handle_future_ = std::shared_future<typename GoalHandleFollowWaypoints::SharedPtr>();
  goal_handle_.reset();
  result_future_ = std::shared_future<typename GoalHandleFollowWaypoints::WrappedResult>();
}

}  // namespace rbwatcher_behaviors
