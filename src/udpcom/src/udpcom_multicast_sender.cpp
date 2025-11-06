#include "udpcom/udpcom_multicast_sender.hpp"

namespace udpcom {

Multicast_sender::Multicast_sender(boost::asio::io_context &io_context,
                                   const std::string multicast_address,
                                   const int port, const std::string identifier)
    : io_context_(io_context),
      multicast_address_(multicast_address),
      port_(port),
      identifier_(identifier) {
  open_socket();
}

Multicast_sender::~Multicast_sender() {
  socket_->close();
  socket_.reset();
}

void Multicast_sender::open_socket() {
  sender_endpoint_ = std::make_unique<udp::endpoint>(
      boost::asio::ip::address::from_string(multicast_address_), port_);
  socket_ =
      std::make_unique<udp::socket>(io_context_, sender_endpoint_->protocol());
  socket_->set_option(udp::socket::reuse_address(true));
}

void Multicast_sender::send(const humanoid_interfaces::msg::Master2udp &msg) {
  std::string message;
  message =
      identifier_ + ": " + std::to_string(msg.robotnum) + "/" +
      std::to_string(msg.robotstate) + "/" + std::to_string(msg.robotcoorx) +
      "/" + std::to_string(msg.robotcoory) + "/" +
      std::to_string(msg.robotimuyaw) + "/" + std::to_string(msg.balldist) +
      "/" + std::to_string(msg.ballcoorx) + "/" + std::to_string(msg.ballcoory);

  send_message(message, *sender_endpoint_);

}

void Multicast_sender::send_message(std::string &msg, udp::endpoint &endpoint) {

  socket_->send_to(boost::asio::buffer(msg), endpoint);

}

}  // namespace udpcom
