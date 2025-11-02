#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rbwatcher_behaviors/action/execute_behavior_tree.hpp"

namespace rbwatcher_behaviors
{

class BehaviorTreeActionServer : public rclcpp::Node
{
public:
  using ExecuteBehaviorTree = rbwatcher_behaviors::action::ExecuteBehaviorTree;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBehaviorTree>;

  explicit BehaviorTreeActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteBehaviorTree::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  std::string loadTree(const ExecuteBehaviorTree::Goal & goal) const;
  //BT::BehaviorTreeFactory createFactory() const;
  std::unique_ptr<BT::BehaviorTreeFactory> createFactory() const; // <-- NEW
  void setInitialBlackboardValues(const ExecuteBehaviorTree::Goal & goal, BT::Blackboard::Ptr blackboard) const;

  rclcpp_action::Server<ExecuteBehaviorTree>::SharedPtr action_server_;
  std::string default_tree_path_;
  double tick_frequency_hz_;
};

}  // namespace rbwatcher_behaviors
