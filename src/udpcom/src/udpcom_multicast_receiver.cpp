#include "udpcom/udpcom_multicast_receiver.hpp"

namespace udpcom {

Multicast_receiver::Multicast_receiver(boost::asio::io_context &io_context,
                                       const std::string multicast_address,
                                       const int port,
                                       const std::string identifier)
    : io_context_(io_context),
      multicast_address_(multicast_address),
      port_(port),
      identifier_(identifier) {
  open_socket();
}

Multicast_receiver::~Multicast_receiver() {
  socket_->close();
  socket_.reset();
}

void Multicast_receiver::open_socket() {
  listen_endpoint_ = std::make_unique<udp::endpoint>(
      boost::asio::ip::address::from_string("0.0.0.0"), port_);
  socket_ =
      std::make_unique<udp::socket>(io_context_, listen_endpoint_->protocol());
  socket_->set_option(udp::socket::reuse_address(true));
  socket_->bind(*listen_endpoint_);
  socket_->set_option(boost::asio::ip::multicast::join_group(
      boost::asio::ip::address::from_string(multicast_address_)));
}

bool Multicast_receiver::receive() {
  received_ = false;
  socket_->async_receive_from(
      boost::asio::buffer(receive_buffer_), *listen_endpoint_,
      std::bind(&Multicast_receiver::handle_receive, this,
                std::placeholders::_1, std::placeholders::_2));
  io_context_.run();
  io_context_.reset();
  return received_;
}

void Multicast_receiver::handle_receive(const boost::system::error_code &error,
                                        std::size_t bytes_recvd) {
  if (!error) {
    std::string received_message(receive_buffer_.data(), bytes_recvd);
    std::size_t pos = received_message.find(":");

    if (pos == std::string::npos) {
      return;
    }
    std::string sender_identifier = received_message.substr(0, pos);
    if (sender_identifier != identifier_) {
      std::string subs = received_message.substr(pos + 1);

      std::istringstream ss(subs);
      std::string subs1;

      int i = 0;

      while (getline(ss, subs1, '/')) {
        if (i == 0) {
          multicast_msg.robotnum = std::stoi(subs1);
        } else if (i == 1) {
          multicast_msg.robotcase = std::stoi(subs1);
        } else if (i == 2) {
          multicast_msg.localx = std::stoi(subs1);
        } else if (i == 3) {
          multicast_msg.localy = std::stoi(subs1);
        } else if (i == 4) {
          multicast_msg.localyaw = std::stoi(subs1);
        } else if (i == 5) {
          multicast_msg.balldist = std::stoi(subs1);
        } else if (i == 6) {
          multicast_msg.ballx = std::stoi(subs1);
        } else if (i == 7) {
          multicast_msg.bally = std::stoi(subs1);
        }
        ++i;
      }
      received_ = true;
      return;
    } else {
      received_ = false;
      return;
    }
  } else {
    std::cerr << "Receive error: " << error.message() << std::endl;
    received_ = false;
    return;
  }
}

}  // namespace udpcom
