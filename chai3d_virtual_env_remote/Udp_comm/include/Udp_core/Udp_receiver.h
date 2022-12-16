//
// Created by zican on 24.06.22.
//

#ifndef SIMPLE_UDP_CORE_UDP_RECEIVER_H
#define SIMPLE_UDP_CORE_UDP_RECEIVER_H

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>


class Udp_receiver {

public:
    //A receiver object that listens to the port p
    Udp_receiver(unsigned short p);
    //retrieve endpoint information right after receiving one msg
    std::string rcv_packet();
    //Returns the IP and port combination of the last socket that sent something to the Receiver
    boost::asio::ip::udp::endpoint getClientEndpoint();

    int listen_port;

private:

    //Newer boost implementations use io_context, the current one used for the lab is a bit older
    //If in the future the compiler complains about io_service comment it out and uncomment io_context

    //boost::io_service is the main interface to communicate with the System Calls of different OS
    boost::asio::io_service io_con;

    //boost::endpoint is the abstraction for one network connection
    boost::asio::ip::udp::endpoint client_endpoint;

    //boost::asio::io_context io_con;
    boost::asio::ip::udp::socket sock;
    static const uint32_t PACK_SIZE = 1024; 	//Maximum number of bytes in an UDP package

};


#endif //SIMPLE_UDP_CORE_UDP_RECEIVER_H
