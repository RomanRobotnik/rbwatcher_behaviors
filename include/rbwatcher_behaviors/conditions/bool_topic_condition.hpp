#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rbwatcher_behaviors
{

class BoolTopicCondition : public BT::ConditionNode
{
public:
  BoolTopicCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> getRosNode();
  void ensureSubscription(const std::string & topic);

  std::weak_ptr<rclcpp::Node> cached_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::atomic<bool> last_value_;
  std::atomic<bool> has_message_;
  std::string topic_;
};

}  // namespace rbwatcher_behaviors
