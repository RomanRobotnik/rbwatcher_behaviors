#include "rbwatcher_behaviors/actions/ptz_tracker_action.hpp"

#include <behaviortree_cpp/blackboard.h>

namespace rbwatcher_behaviors
{

PTZTrackerAction::PTZTrackerAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  action_name_ = "/ptz_tracker/start_tracking";
}

BT::PortsList PTZTrackerAction::providedPorts()
{
  return {
    BT::InputPort<bool>("start", true, "Flag passed to the TrackTarget goal"),
    BT::InputPort<std::string>("action_name", "/ptz_tracker/start_tracking"),
    BT::OutputPort<std::string>("message", "Result message from PTZ tracker"),
    BT::OutputPort<double>("final_pan", "Final pan angle"),
    BT::OutputPort<double>("final_tilt", "Final tilt angle"),
    BT::OutputPort<double>("final_zoom", "Final zoom value")
  };
}

BT::NodeStatus PTZTrackerAction::onStart()
{
  resetGoalHandles();

  auto node = getRosNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("PTZTrackerAction"), "Missing ROS node on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  bool start_value = true;
  if (auto start_flag = getInput<bool>("start")) {
    start_value = start_flag.value();
  } else {
    RCLCPP_DEBUG(node->get_logger(),
      "PTZTrackerAction missing 'start' input, defaulting to true");
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
    RCLCPP_WARN(node->get_logger(), "PTZTracker action server %s not available", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  TrackTarget::Goal goal;
  goal.start = start_value;

  goal_handle_future_ = client->async_send_goal(goal);
  if (!goal_handle_future_.valid()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send PTZTracker goal");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "PTZTracker goal dispatched to %s", action_name_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PTZTrackerAction::onRunning()
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
          RCLCPP_WARN(node->get_logger(), "PTZTracker goal rejected");
          return BT::NodeStatus::SUCCESS;
        }
        auto client = getActionClient();
        if (!client) {
          RCLCPP_ERROR(node->get_logger(), "PTZTracker action client missing");
          return BT::NodeStatus::SUCCESS;
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
      const auto & result = wrapped_result.result;
      if (result) {
        setOutput("message", result->message);
        setOutput("final_pan", result->final_pan);
        setOutput("final_tilt", result->final_tilt);
        setOutput("final_zoom", result->final_zoom);
      }

      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node->get_logger(), "PTZTracker goal completed successfully");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(node->get_logger(), "PTZTracker goal aborted: %s",
            result ? result->message.c_str() : "no message");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node->get_logger(), "PTZTracker goal canceled");
          break;
        default:
          RCLCPP_WARN(node->get_logger(), "PTZTracker goal finished with unknown result code");
          break;
      }

      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void PTZTrackerAction::onHalted()
{
  if (goal_handle_ && getActionClient()) {
    (void)getActionClient()->async_cancel_goal(goal_handle_);
  }
  resetGoalHandles();
}

std::shared_ptr<rclcpp::Node> PTZTrackerAction::getRosNode()
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

rclcpp_action::Client<PTZTrackerAction::TrackTarget>::SharedPtr PTZTrackerAction::getActionClient()
{
  auto node = getRosNode();
  if (!node) {
    return nullptr;
  }

  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<TrackTarget>(node, action_name_);
  }

  return action_client_;
}

void PTZTrackerAction::resetGoalHandles()
{
  goal_handle_future_ = std::shared_future<typename GoalHandleTrackTarget::SharedPtr>();
  goal_handle_.reset();
  result_future_ = std::shared_future<typename GoalHandleTrackTarget::WrappedResult>();
}

}  // namespace rbwatcher_behaviors
