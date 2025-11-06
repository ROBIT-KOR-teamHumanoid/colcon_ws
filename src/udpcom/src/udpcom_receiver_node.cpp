#include "../include/udpcom/udpcom_receiver_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace udpcom {
UDPComNode_Receiver::UDPComNode_Receiver(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("udpcom_node", options) {
  RCLCPP_INFO(get_logger(), "Creating UDP node");
  init_parameters();
  RCLCPP_INFO(get_logger(), "Parameters initialized");
}

UDPComNode_Receiver::~UDPComNode_Receiver() {
  RCLCPP_INFO(get_logger(), "Destroying node");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Receiver::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");
  get_parameters();

  udpcom_pub_ =
      create_publisher<humanoid_interfaces::msg::Udp2master>("udp", 10);

  RCLCPP_INFO(get_logger(),
              "identifier: %s , multicast address : %s , port : %d",
              identifier_.c_str(), multicast_address_.c_str(), port_);

  try {
    receiver = std::make_unique<Multicast_receiver>(
        receiver_io_context_, multicast_address_, port_, identifier_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error creating UDPMulticast: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Receiver::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating");

  udpcom_pub_->on_activate();
  publish_timer_ =
      create_wall_timer(std::chrono::milliseconds(10),
                        std::bind(&UDPComNode_Receiver::udpcom_publish, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Receiver::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  udpcom_pub_->on_deactivate();
  publish_timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Receiver::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UDPComNode_Receiver::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void udpcom::UDPComNode_Receiver::init_parameters() {
  declare_parameter("identifier", rclcpp::ParameterValue("identifier"));
  declare_parameter("multicast_address", rclcpp::ParameterValue("239.255.0.1"));
  declare_parameter("port", rclcpp::ParameterValue(9090));
}

void udpcom::UDPComNode_Receiver::get_parameters() {
  identifier_ = get_parameter("identifier").as_string();
  multicast_address_ = get_parameter("multicast_address").as_string();
  port_ = get_parameter("port").as_int();
}

void udpcom::UDPComNode_Receiver::udpcom_publish() {
  if (!receiver->receive()) {
    return;
  }

  pubmsg.robotnum = receiver->multicast_msg.robotnum;
  pubmsg.robotcase = receiver->multicast_msg.robotcase;
  pubmsg.localx = receiver->multicast_msg.localx;
  pubmsg.localy = receiver->multicast_msg.localy;
  pubmsg.localyaw = receiver->multicast_msg.localyaw;
  pubmsg.balldist = receiver->multicast_msg.balldist;
  pubmsg.ballx = receiver->multicast_msg.ballx;
  pubmsg.bally = receiver->multicast_msg.bally;

  udpcom_pub_->publish(pubmsg);
}
}  // namespace udpcom

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<udpcom::UDPComNode_Receiver>();

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
