//
// Created by zican on 24.06.22.
//

#include "../../include/Udp_core/Udp_receiver.h"

// Udp_receiver::io_con(boost::io_service)  is          default initialized
// Udp_receiver::client_endpoint(boost::endpoint) is    default initialized
// Upd_receiver::sock(boost socket) is                  initialized with Udp_receiver::io_con
//      the receiver is self is bind with an endpoint with local port number p
// this receiver is bind to port p and cares nothing about the incoming upd message
// we suggest using rcvPacket(char *data, size_t len, boost::asio::ip::udp::endpoint & clientEndpoint) to immediately acquire remote endpoint information
Udp_receiver::Udp_receiver(unsigned short p) :listen_port(p),
    sock(io_con, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), p))
    {   }

std::string Udp_receiver::rcv_packet()
{
    // a boost char buffer with changeable size, and we declare an initial size for it to reduce the resizing overhead
    std::string result;
    boost::array<char, 1024> rcv_buf;
    try{
        boost::asio::ip::udp::endpoint client_endpoint_;

        size_t in_length = sock.receive_from(boost::asio::buffer(rcv_buf), client_endpoint_);
        result=std::string(rcv_buf.begin(),rcv_buf.begin()+in_length);
        client_endpoint=client_endpoint_;
    }catch(std::exception & e){std::cerr<<"Udp_reciever:"<<e.what()<<std::endl;}



    return result;
}

boost::asio::ip::udp::endpoint Udp_receiver::getClientEndpoint(){
    return client_endpoint;
}
