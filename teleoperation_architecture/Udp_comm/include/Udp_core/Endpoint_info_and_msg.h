//
// Created by zican on 22.09.22.
//

#ifndef TEST_CHAI3D_VIRTUAL_ENVIRONMENT_SERVER_COMM_AND_MSG_H
#define TEST_CHAI3D_VIRTUAL_ENVIRONMENT_SERVER_COMM_AND_MSG_H

#include <string>
#include <utility>
#include <boost/asio.hpp>

struct Endpoint_info_and_msg{
    boost::asio::ip::udp::endpoint endpoint;
    std::string msg;
    Endpoint_info_and_msg(boost::asio::ip::udp::endpoint endpoint_,std::string  msg_): endpoint(std::move(endpoint_)),msg(std::move(msg_)){};
};

#endif //TEST_CHAI3D_VIRTUAL_ENVIRONMENT_SERVER_COMM_AND_MSG_H
