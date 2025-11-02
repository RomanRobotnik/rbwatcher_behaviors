#pragma once

#include <future>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rbwatcher_behaviors
{

class FollowWaypointsAction : public BT::StatefulActionNode
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  FollowWaypointsAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> getRosNode();
  rclcpp_action::Client<FollowWaypoints>::SharedPtr getActionClient();
  void resetGoalHandles();

  std::weak_ptr<rclcpp::Node> cached_node_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  std::shared_future<typename GoalHandleFollowWaypoints::SharedPtr> goal_handle_future_;
  typename GoalHandleFollowWaypoints::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleFollowWaypoints::WrappedResult> result_future_;
  std::string action_name_;
};

}  // namespace rbwatcher_behaviors
