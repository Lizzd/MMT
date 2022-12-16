//
// Created by zican on 24.06.22.
//

#ifndef SIMPLE_UDP_CORE_UDP_SERVER_H
#define SIMPLE_UDP_CORE_UDP_SERVER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <set>
#include <list>
#include <boost/asio.hpp>
#include <string>
#include <utility>


#include "Udp_sender.h"
#include "Udp_receiver.h"
#include "Thread/Thread_safe_unit_msg.h"
#include "Endpoint_info_and_msg.h"

using namespace std;


typedef boost::asio::ip::udp::endpoint Asio_endpoint;

// Udp_server send the same msg to all clients associated with this server
class Udp_server {
private:

    //for every client we allocate it with a msg rcv queue
    struct Client_msg_queue{
        const Asio_endpoint client_endpoint;
        Thread_safe_unit_msg<string> rcv_msg_queue;
        Client_msg_queue(string ip_,const int port_): client_endpoint(boost::asio::ip::address::from_string(ip_),port_){};
        explicit Client_msg_queue(Asio_endpoint client_endpoint_): client_endpoint(std::move(client_endpoint_)){};
    };

    //todo: we need a more flexible data parser

    Udp_receiver any_client_listener;
    //this is for quick check for the new client, since hashing string is much quicker than hashing objects
    set<string> current_remote_address_set;
    //we cannot check ip address and port separately, the last check must be done with the endpoint_list as a whole
    set<boost::asio::ip::udp::endpoint> current_endpoint_set;
    Udp_sender udp_sender;

    list<Asio_endpoint> client_info_list;
    list<Client_msg_queue> client_rcv_msg_queue;

    Thread_safe_unit_msg<Endpoint_info_and_msg> target_snd_msg_queue;

    Thread_safe_unit_msg<string> rcv_msg_queue;
    Thread_safe_unit_msg<string> snd_msg_queue;

    [[noreturn]] static void listener_thread_task(Udp_server* udp_server);

    [[noreturn]] static void sender_thread_task(Udp_server* udp_server);

    //todo: use strategic design pattern for this data_processing_task();
    [[noreturn]] static void data_processing_task(Udp_server* udp_server);

    bool init_done=false;

public:

    //the server start running threads after construction
    Udp_server(int port_,bool has_data_processing_thread);
    //by default has_data_processing_thread=false;
    Udp_server(int port_);
    void add_snd_request_to_all_client(const std::string & msg){snd_msg_queue.push_back(msg);}
    void add_snd_request_to_one_client(const Endpoint_info_and_msg & endpoint_and_msg){target_snd_msg_queue.push_back(endpoint_and_msg);}

    bool get_one_msg(string &output){
        if(!rcv_msg_queue.is_empty())
        {
            output= *(rcv_msg_queue.wait_and_pop_front());
            return true;
        }
        return false;
    }

    bool get_msgs_from_all_client(list<Endpoint_info_and_msg> &output_list){
        bool has_at_least_one_msg=false;
        for (auto &it : client_rcv_msg_queue)
        {

            if(!it.rcv_msg_queue.is_empty())
            {
                has_at_least_one_msg=true;
                string msg= *(it.rcv_msg_queue.wait_and_pop_front());
                output_list.emplace_back(it.client_endpoint,msg);
            }
            else this_thread::yield();
        }
        return has_at_least_one_msg;
    }

    inline bool is_init_done(){return init_done;}

    //todo: no copy constructor and copy assign

};


#endif //SIMPLE_UDP_CORE_UDP_SERVER_H
