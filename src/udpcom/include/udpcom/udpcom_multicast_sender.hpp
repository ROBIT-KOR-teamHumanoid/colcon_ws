#ifndef UDP_MULTICAST_SENDER_HPP
#define UDP_MULTICAST_SENDER_HPP

#include <array>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "humanoid_interfaces/msg/master2udp.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"

using boost::asio::ip::udp;

namespace udpcom {

class Multicast_sender {
 public:  // Functions
  Multicast_sender(boost::asio::io_context &io_context,
                   const std::string multicast_address, const int port,
                   const std::string identifier);
  ~Multicast_sender();

  void send(const humanoid_interfaces::msg::Master2udp &msg);

 protected:  // Functions
  void open_socket();

 private:  // Functions
  void send_message(std::string &msg, udp::endpoint &endpoint);

 private:  // Variables
  std::unique_ptr<udp::endpoint> sender_endpoint_;
  boost::asio::io_context &io_context_;
  std::unique_ptr<udp::socket> socket_;
  std::string multicast_address_;
  int port_;
  std::string identifier_;
};

}  // namespace udpcom
#endif  // UDP_MULTICAST_HPP
