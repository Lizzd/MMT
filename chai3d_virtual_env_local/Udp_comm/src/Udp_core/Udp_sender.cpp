//
// Created by zican on 24.06.22.
//

#include "../../include/Udp_core/Udp_sender.h"

Udp_sender::Udp_sender() :
    sock(io_con, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
    {   }

void Udp_sender::sendPacket(const std::string & msg,  std::string ip_address,unsigned short port) {
    sock.send_to(boost::asio::buffer(msg), boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ip_address), port));
}

void Udp_sender::sendPacket(const std::string & msg, boost::asio::ip::udp::endpoint client_endpoint){
    sock.send_to(boost::asio::buffer(msg), client_endpoint);
}


