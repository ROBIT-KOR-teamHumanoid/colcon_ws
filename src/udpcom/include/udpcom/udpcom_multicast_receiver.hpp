#ifndef UDP_MULTICAST_RECEIVER_HPP
#define UDP_MULTICAST_RECEIVER_HPP

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

class Multicast_receiver {
 public:  // Functions
  Multicast_receiver(boost::asio::io_context &io_context,
                     const std::string multicast_address, const int port,
                     const std::string identifier);
  ~Multicast_receiver();
  bool receive();

  humanoid_interfaces::msg::Udp2master multicast_msg;

 private:  // Functions
  void open_socket();
  void handle_receive(const boost::system::error_code &error,
                      std::size_t bytes_recvd);
  void start_receive();

 private:  // Variables
  std::unique_ptr<udp::endpoint> listen_endpoint_;
  std::array<char, 1024> receive_buffer_;
  bool received_;
  boost::asio::io_context &io_context_;
  std::unique_ptr<udp::socket> socket_;
  std::string multicast_address_;
  int port_;
  std::string identifier_;
};

}  // namespace udpcom
#endif  // UDP_MULTICAST_HPP
