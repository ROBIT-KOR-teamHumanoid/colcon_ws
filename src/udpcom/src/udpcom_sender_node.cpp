#include "../include/udpcom/udpcom_sender_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace udpcom {
UDPComNode_Sender::UDPComNode_Sender(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("udpcom_node", options) {
  RCLCPP_INFO(get_logger(), "Creating UDP node");
  init_parameters();
  RCLCPP_INFO(get_logger(), "Parameters initialized");
}

UDPComNode_Sender::~UDPComNode_Sender() {
  RCLCPP_INFO(get_logger(), "Destroying node");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Sender::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");
  get_parameters();

  udpcom_sub_ = create_subscription<humanoid_interfaces::msg::Master2udp>(
      "master2udp", 10,
      std::bind(&UDPComNode_Sender::udpcom_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "identifier: %s , multicast address : %s , port : %d",
              identifier_.c_str(), multicast_address_.c_str(), port_);

  try {
    sender = std::make_unique<Multicast_sender>(
        sender_io_context_, multicast_address_, port_, identifier_);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error creating UDPMulticast: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Sender::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Sender::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Sender::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Sender::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void udpcom::UDPComNode_Sender::init_parameters() {
  declare_parameter("identifier", rclcpp::ParameterValue("identifier"));
  declare_parameter("multicast_address", rclcpp::ParameterValue("239.255.0.1"));
  declare_parameter("port", rclcpp::ParameterValue(9090));
}

void udpcom::UDPComNode_Sender::get_parameters() {
  identifier_ = get_parameter("identifier").as_string();
  multicast_address_ = get_parameter("multicast_address").as_string();
  port_ = get_parameter("port").as_int();
}

void udpcom::UDPComNode_Sender::udpcom_callback(
    const humanoid_interfaces::msg::Master2udp::SharedPtr msg) {
  sender->send(*msg);
}

}  // namespace udpcom

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<udpcom::UDPComNode_Sender>();

  auto configure_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure udp.");
    return 1;
  }

  auto activate_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate udp.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
