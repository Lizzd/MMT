//
// Created by zican on 24.06.22.
//

#ifndef SIMPLE_UDP_CORE_UDP_SENDER_H
#define SIMPLE_UDP_CORE_UDP_SENDER_H

#include <iostream>
#include <string>
#include <boost/asio.hpp>


// Upd_sender cares nothing but sending, so it stores no information about the remote endpoint and whenever a send request is make, the remote endpoint information must be provided again
class Udp_sender {
public:

    Udp_sender();

    //Send len bytes starting from data ([*data..*(data+len)])
    //to client_endpoint which is an IP and port combination
    //If len is bigger than 1024 it throws an exception
    void sendPacket(const std::string & msg, boost::asio::ip::udp::endpoint client_endpoint);

    void sendPacket(const std::string & msg, std::string s,unsigned short p);



private:

    //Newer boost implementations use io_context, the current one used for the lab is a bit older
    //If in the future the compiler complains about io_service comment it out and uncomment io_context
    boost::asio::io_service io_con;
    //boost::asio::io_context io_con;
    boost::asio::ip::udp::socket sock;

    // we use ip address directly so local DNS resolving is not needed
    //boost::asio::ip::udp::resolver res;

};


#endif //SIMPLE_UDP_CORE_UDP_SENDER_H
