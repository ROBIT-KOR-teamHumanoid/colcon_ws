#ifndef UDP_COM_RECEIVER_NODE_HPP
#define UDP_COM_RECEIVER_NODE_HPP

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <thread>

#include "humanoid_interfaces/msg/udp2master.hpp"
#include "udpcom_multicast_receiver.hpp"

namespace udpcom {

class UDPComNode_Receiver : public rclcpp_lifecycle::LifecycleNode {
 public:
  UDPComNode_Receiver(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~UDPComNode_Receiver();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters);

 private:
  void init_parameters();
  void get_parameters();

  void udpcom_publish();

  std::unique_ptr<Multicast_receiver> receiver;

  humanoid_interfaces::msg::Udp2master pubmsg;

 private:
  rclcpp_lifecycle::LifecyclePublisher<
      humanoid_interfaces::msg::Udp2master>::SharedPtr udpcom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string identifier_;
  std::string multicast_address_;
  short port_;

  boost::asio::io_context receiver_io_context_;
};

}  // namespace udpcom

#endif
