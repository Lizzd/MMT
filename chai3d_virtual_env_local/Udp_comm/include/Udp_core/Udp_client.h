//
// Created by zican on 24.06.22.
//

#ifndef SIMPLE_UDP_CORE_UDP_CLIENT_H
#define SIMPLE_UDP_CORE_UDP_CLIENT_H

#include <iostream>
#include <thread>
#include <mutex>
#include <set>
#include <list>
#include <boost/asio.hpp>


#include "Udp_sender.h"
#include "Udp_receiver.h"
#include "Thread/Thread_safe_unit_msg.h"

using namespace std;

// by design this client only communicate with one server
// creating multiple Udp_client to communicate with multiple servers
class Udp_client {
private:
    struct Comm_info{
        std::string ip;
        int port;
        Comm_info():ip("127.0.0.1"),port(1234){};
        Comm_info(string ip_,const int port_):ip(std::move(ip_)),port(port_){};
    };

    Udp_receiver server_msg_receiver;
    Udp_sender udp_sender;

    Comm_info server_info;

    //for teleoperation, we do not want a queue to save all the data which are not up to date
    //so a thread safe queue of size 1 is used. It stores 1 data, and abandons old data whenever new data come.
    Thread_safe_unit_msg<string> rcv_msg_queue;
    Thread_safe_unit_msg<string> snd_msg_queue;

    bool init_done=false;
public:
    //only for local test
    Udp_client(int port_, bool has_data_processing_thread);
    Udp_client(int port_);

    Udp_client(int client_port,string server_ip,int server_port,bool has_data_processing_thread);
    Udp_client(int client_port,string server_ip,int server_port);
    //~Udp_client(){};

    [[noreturn]] static void listener_thread_task(Udp_client* udp_client);

    [[noreturn]] static void sender_thread_task(Udp_client* udp_client);

    [[noreturn]] static void data_processing_thread_task(Udp_client* udp_client);

    void add_snd_request(const std::string &msg){
        snd_msg_queue.push_back(msg);
    }

    bool get_one_msg(string &output){
        if(!rcv_msg_queue.is_empty())
        {
            output= *(rcv_msg_queue.wait_and_pop_front());
            return true;
        }
        return false;
    }

    inline bool is_init_done(){return init_done;}
};


#endif //SIMPLE_UDP_CORE_UDP_CLIENT_H
