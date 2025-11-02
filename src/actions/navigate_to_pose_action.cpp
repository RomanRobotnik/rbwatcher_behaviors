#include "rbwatcher_behaviors/actions/navigate_to_pose_action.hpp"

#include <chrono>
#include <utility>

#include <behaviortree_cpp/blackboard.h>

namespace rbwatcher_behaviors
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  action_name_ = "/navigate_to_pose";
}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("action_name", "/navigate_to_pose")
  };
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  resetGoalHandles();

  auto node = getRosNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Missing ROS node on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_msg) {
    RCLCPP_ERROR(node->get_logger(), "NavigateToPoseAction requires a goal PoseStamped");
    return BT::NodeStatus::FAILURE;
  }

  auto requested_action_name = getInput<std::string>("action_name");
  if (requested_action_name) {
    if (action_name_ != requested_action_name.value()) {
      action_client_.reset();
      action_name_ = requested_action_name.value();
    }
  }

  auto client = getActionClient();
  if (!client) {
    RCLCPP_ERROR(node->get_logger(), "Unable to create action client for %s", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node->get_logger(), "NavigateToPose action server %s not available", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose = goal_msg.value();

  goal_handle_future_ = client->async_send_goal(goal);
  if (!goal_handle_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send NavigateToPose goal");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "NavigateToPose goal dispatched to %s", action_name_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
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
          RCLCPP_WARN(node->get_logger(), "NavigateToPose goal rejected");
          return BT::NodeStatus::FAILURE;
        }
        auto client = getActionClient();
        if (!client) {
          RCLCPP_ERROR(node->get_logger(), "NavigateToPose action client missing");
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
          RCLCPP_INFO(node->get_logger(), "NavigateToPose goal reached");
          return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(node->get_logger(), "NavigateToPose goal aborted");
          return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(node->get_logger(), "NavigateToPose goal canceled");
          return BT::NodeStatus::FAILURE;
        default:
          RCLCPP_ERROR(node->get_logger(), "NavigateToPose result unknown");
          return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  if (goal_handle_ && getActionClient()) {
    (void)getActionClient()->async_cancel_goal(goal_handle_);
  }
  resetGoalHandles();
}

std::shared_ptr<rclcpp::Node> NavigateToPoseAction::getRosNode()
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

rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr NavigateToPoseAction::getActionClient()
{
  auto node = getRosNode();
  if (!node) {
    return nullptr;
  }

  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, action_name_);
  }

  return action_client_;
}

void NavigateToPoseAction::resetGoalHandles()
{
  goal_handle_future_ = std::shared_future<typename GoalHandleNavigateToPose::SharedPtr>();
  goal_handle_.reset();
  result_future_ = std::shared_future<typename GoalHandleNavigateToPose::WrappedResult>();
}

}  // namespace rbwatcher_behaviors
