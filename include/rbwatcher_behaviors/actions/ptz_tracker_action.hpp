#pragma once

#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <ptz_tracker_interfaces/action/track_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rbwatcher_behaviors
{

class PTZTrackerAction : public BT::StatefulActionNode
{
public:
  using TrackTarget = ptz_tracker_interfaces::action::TrackTarget;
  using GoalHandleTrackTarget = rclcpp_action::ClientGoalHandle<TrackTarget>;

  PTZTrackerAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> getRosNode();
  rclcpp_action::Client<TrackTarget>::SharedPtr getActionClient();
  void resetGoalHandles();

  std::weak_ptr<rclcpp::Node> cached_node_;
  rclcpp_action::Client<TrackTarget>::SharedPtr action_client_;
  std::shared_future<typename GoalHandleTrackTarget::SharedPtr> goal_handle_future_;
  typename GoalHandleTrackTarget::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleTrackTarget::WrappedResult> result_future_;
  std::string action_name_;
};

}  // namespace rbwatcher_behaviors
