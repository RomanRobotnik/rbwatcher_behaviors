#include "rbwatcher_behaviors/actions/trigger_service_action.hpp"

#include <chrono>

#include <behaviortree_cpp/blackboard.h>

namespace rbwatcher_behaviors
{

TriggerServiceAction::TriggerServiceAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  service_name_ = "/mission/trigger";
}

BT::PortsList TriggerServiceAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "/mission/trigger", "Trigger service name"),
    BT::InputPort<unsigned int>("timeout_ms", 2000U, "Service wait timeout (ms)"),
    BT::OutputPort<std::string>("response_message", "Response message from service")
  };
}

BT::NodeStatus TriggerServiceAction::onStart()
{
  resetFuture();

  auto node = getRosNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("TriggerServiceAction"), "Missing ROS node on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  auto requested_service_name = getInput<std::string>("service_name");
  if (requested_service_name) {
    if (service_name_ != requested_service_name.value()) {
      client_.reset();
      service_name_ = requested_service_name.value();
    }
  }

  unsigned int timeout_ms = 2000U;
  auto timeout_value = getInput<unsigned int>("timeout_ms");
  if (timeout_value) {
    timeout_ms = timeout_value.value();
  }

  auto node_shared = getRosNode();
  if (!node_shared) {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_) {
    client_ = node_shared->create_client<Trigger>(service_name_);
  }

  if (!client_) {
    RCLCPP_ERROR(node_shared->get_logger(), "Failed to create client for service %s", service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
    RCLCPP_WARN(node_shared->get_logger(), "Service %s not available", service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<Trigger::Request>();
  auto future_pair = client_->async_send_request(request);
  future_ = future_pair.future.share();
  if (!future_.valid()) {
    RCLCPP_ERROR(node_shared->get_logger(), "Failed to send Trigger service request");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TriggerServiceAction::onRunning()
{
  if (future_.valid()) {
    auto status = future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      auto response = future_.get();
      auto node = getRosNode();
      if (!node) {
        return BT::NodeStatus::FAILURE;
      }

      if (!response->success) {
        RCLCPP_WARN(node->get_logger(), "Trigger service %s responded with failure: %s",
          service_name_.c_str(), response->message.c_str());
        setOutput("response_message", response->message);
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(node->get_logger(), "Trigger service %s succeeded: %s",
        service_name_.c_str(), response->message.c_str());
      setOutput("response_message", response->message);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void TriggerServiceAction::onHalted()
{
  resetFuture();
}

std::shared_ptr<rclcpp::Node> TriggerServiceAction::getRosNode()
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

void TriggerServiceAction::resetFuture()
{
  future_ = std::shared_future<typename Trigger::Response::SharedPtr>();
}

}  // namespace rbwatcher_behaviors
