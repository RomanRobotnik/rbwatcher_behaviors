#pragma once

#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rbwatcher_behaviors
{

class TriggerServiceAction : public BT::StatefulActionNode
{
public:
  using Trigger = std_srvs::srv::Trigger;

  TriggerServiceAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> getRosNode();
  void resetFuture();

  std::weak_ptr<rclcpp::Node> cached_node_;
  rclcpp::Client<Trigger>::SharedPtr client_;
  std::shared_future<typename Trigger::Response::SharedPtr> future_;
  std::string service_name_;
};

}  // namespace rbwatcher_behaviors
