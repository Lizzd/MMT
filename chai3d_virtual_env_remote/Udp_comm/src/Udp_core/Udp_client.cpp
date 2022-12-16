//
// Created by zican on 24.06.22.
//

#include "../../include/Udp_core/Udp_client.h"

Udp_client::Udp_client(int port, bool has_data_processing_thread): server_msg_receiver(port){
    if(has_data_processing_thread)
    {
        thread processing_thread(Udp_client::data_processing_thread_task,this);
        processing_thread.detach();
    }

    thread sender_thread(Udp_client::sender_thread_task,this);
    sender_thread.detach();
    thread listener_thread(Udp_client::listener_thread_task,this);
    listener_thread.detach();
    init_done=true;
}
Udp_client::Udp_client(int port): Udp_client(port,false){}

Udp_client::Udp_client(int client_port,string server_ip,int server_port,bool has_data_processing_thread):
        server_msg_receiver(client_port)
{
    server_info.ip=server_ip;
    server_info.port=server_port;

    if(has_data_processing_thread)
    {
        thread processing_thread(Udp_client::data_processing_thread_task, this);
        processing_thread.detach();
    }


    thread listener_thread(Udp_client::listener_thread_task,this);
    listener_thread.detach();

    thread sender_thread(Udp_client::sender_thread_task,this);
    sender_thread.detach();

    init_done=true;
}
Udp_client::Udp_client(int client_port,string server_ip,int server_port): Udp_client(client_port,server_ip,server_port,false){}

void Udp_client::listener_thread_task(Udp_client *udp_client) {

    while(true){
        //boost::asio::ip::udp::endpoint remote_endpoint;
        std::string msg=udp_client->server_msg_receiver.rcv_packet();


        //cout<<"[client] received feedback: "<<msg<<endl;
        udp_client->rcv_msg_queue.push_back(msg);
    }
}

void Udp_client::sender_thread_task(Udp_client *udp_client) {
    while(true)
    {
        try{
            if(udp_client->snd_msg_queue.is_empty())
            {
                // we want the thread to run less than 1kHz
                this_thread::sleep_for(chrono::milliseconds(1));
                continue;
            }


            shared_ptr<string> msg=udp_client->snd_msg_queue.wait_and_pop_front();

            *msg= to_string(udp_client->server_msg_receiver.listen_port)+":"+*msg;

            udp_client->udp_sender.sendPacket(*msg,udp_client->server_info.ip,udp_client->server_info.port);

            this_thread::sleep_for(chrono::milliseconds(1));
        }catch(exception e){cerr<<"[sender_thread_task]"<<e.what()<<endl;}

    }
}

void Udp_client::data_processing_thread_task(Udp_client *udp_client) {
    int cnt=0;

    while(true) {
        if(cnt>=1000)cnt=0;
        shared_ptr<string> rcv_msg=udp_client->rcv_msg_queue.wait_and_pop_front();
        cout<<"thread processing: rcv["<<cnt<<"] "<<*rcv_msg<<endl;

        this_thread::sleep_for(chrono::milliseconds(10));
        //simulate the haptic rendering and data reading
        cnt++;
    }
}
