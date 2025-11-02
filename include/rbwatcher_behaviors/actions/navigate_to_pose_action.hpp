#pragma once

#include <memory>
#include <future>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rbwatcher_behaviors
{

class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> getRosNode();
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr getActionClient();
  void resetGoalHandles();

  std::weak_ptr<rclcpp::Node> cached_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  std::shared_future<typename GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;
  typename GoalHandleNavigateToPose::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleNavigateToPose::WrappedResult> result_future_;
  std::string action_name_;
};

}  // namespace rbwatcher_behaviors
