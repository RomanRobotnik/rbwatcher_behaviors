#include "rbwatcher_behaviors/conditions/bool_topic_condition.hpp"

#include <behaviortree_cpp/blackboard.h>

namespace rbwatcher_behaviors
{

BoolTopicCondition::BoolTopicCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  last_value_(false),
  has_message_(false)
{
}

BT::PortsList BoolTopicCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/mission/ready", "Topic publishing std_msgs/Bool"),
    BT::InputPort<bool>("expected_value", true, "Expected boolean value")
  };
}

BT::NodeStatus BoolTopicCondition::tick()
{
  auto node = getRosNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("BoolTopicCondition"), "Missing ROS node on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  auto topic_name = getInput<std::string>("topic");
  if (!topic_name) {
    RCLCPP_ERROR(node->get_logger(), "BoolTopicCondition requires a topic name");
    return BT::NodeStatus::FAILURE;
  }

  ensureSubscription(topic_name.value());

  bool expected_value = true;
  if (auto expected_input = getInput<bool>("expected_value")) {
    expected_value = expected_input.value();
  }

  if (!has_message_.load()) {
    return BT::NodeStatus::RUNNING;
  }

  return last_value_.load() == expected_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

std::shared_ptr<rclcpp::Node> BoolTopicCondition::getRosNode()
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

void BoolTopicCondition::ensureSubscription(const std::string & topic)
{
  if (topic_ == topic && subscription_) {
    return;
  }

  auto node = getRosNode();
  if (!node) {
    return;
  }

  topic_ = topic;
  has_message_.store(false);
  auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    topic_, qos,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      last_value_.store(msg->data);
      has_message_.store(true);
    });
}

}  // namespace rbwatcher_behaviors
