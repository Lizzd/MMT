//
// Created by zican on 24.06.22.
//

#include "Udp_server.h"


Udp_server::Udp_server(int port_, bool has_data_processing_thread):any_client_listener(port_) {
    cout<<"starting udp server..."<<endl;
    //C++ RAII
    //constructing the thread means running the thread;

    thread sender_thread(Udp_server::sender_thread_task,this);
    sender_thread.detach();

    //for haptic rendering

    if(has_data_processing_thread)
    {
        thread data_process_thread(Udp_server::data_processing_task,this);
        data_process_thread.detach();
    }
    thread listener_thread(Udp_server::listener_thread_task,this);
    listener_thread.detach();

    init_done=true;
}

Udp_server::Udp_server(int port_):Udp_server(port_,false){}

[[noreturn]] void Udp_server::listener_thread_task(Udp_server* udp_server){
    cout<<"starting listener thread..."<<endl;

    while(true){
        boost::asio::ip::udp::endpoint remote_endpoint;
        std::string msg=udp_server->any_client_listener.rcv_packet();
        remote_endpoint=udp_server->any_client_listener.getClientEndpoint();

        size_t found=msg.find(':');
        int feedback_port=0;
        if(found!=std::string::npos)
        {
            feedback_port= stoi(msg.substr(0,found));
            msg=msg.substr(found+1,msg.size()-found);
            //cout<<"parsing result: "<<feedback_port<<"  "<<msg<<endl;
        }
        else
        {
            cerr<<"[server] the message contains no feedback port"<<endl;
        }

        Asio_endpoint client_info=remote_endpoint;
        Asio_endpoint feedback_endpoint=Asio_endpoint(client_info.address(),feedback_port);

        //if it is a new ip address
        if(udp_server->current_remote_address_set.find(client_info.address().to_string())==udp_server->current_remote_address_set.end()||
           udp_server->current_endpoint_set.find(remote_endpoint)==udp_server->current_endpoint_set.end())
        {
            cout<<__FUNCTION__ <<": new client: "<<client_info.address().to_string()<<":"<<client_info.port()<<endl;
            //new endpoint
            //emplace_back construct the new element with the parameters
            //todo: the client_info_list should be thread safe since its accessed by 2 threads
            udp_server->client_info_list.push_back(feedback_endpoint);
            udp_server->client_rcv_msg_queue.emplace_back(feedback_endpoint);
            //update current address, endpoint sets
            udp_server->current_remote_address_set.insert(remote_endpoint.address().to_string());
            udp_server->current_endpoint_set.insert(boost::asio::ip::udp::endpoint(remote_endpoint.address(),remote_endpoint.port()));
        }

        // multi-client multi-thread safe queue design
        for(auto &it:udp_server->client_rcv_msg_queue){
            if(it.client_endpoint==feedback_endpoint)
            {
                it.rcv_msg_queue.push_back(msg);
            }

        }

        //this is the shared_memory operation
        //udp_server->rcv_msg_queue.push_back(msg);
    }
}

[[noreturn]] void Udp_server::sender_thread_task(Udp_server* udp_server){
    cout<<"starting sender thread..."<<endl;
    while(true)
    {
        if(udp_server->snd_msg_queue.is_empty()&&udp_server->target_snd_msg_queue.is_empty())
        {
            // we want the thread to run less than 1kHz
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }

        if(!udp_server->snd_msg_queue.is_empty())
        {
            shared_ptr<string> msg=udp_server->snd_msg_queue.wait_and_pop_front();
            //cout<<"[server] sending: "<<*msg<<endl;
            for(const auto& it:udp_server->client_info_list)
            {
                //cout<<"[server] send target: "<<it.ip<<" "<<it.port<<endl;
                udp_server->udp_sender.sendPacket(*msg,it.address().to_string(),it.port());
            }

        }
        if(!udp_server->target_snd_msg_queue.is_empty())
        {
            shared_ptr<Endpoint_info_and_msg> endpoint_and_msg=udp_server->target_snd_msg_queue.wait_and_pop_front();
            udp_server->udp_sender.sendPacket(endpoint_and_msg->msg,endpoint_and_msg->endpoint);
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }

}

[[noreturn]] void Udp_server::data_processing_task(Udp_server* udp_server){
    int cnt=0;
    cout<<"starting data processing thread..."<<endl;
    while(true) {
        if(cnt>=1000)cnt=0;
        shared_ptr<string> rcv_msg=udp_server->rcv_msg_queue.wait_and_pop_front();
        //cout<<"[server] thread processing: rcv["<<cnt<<"] "<<*rcv_msg<<endl;

        std::string test_snd_msg=std::to_string(cnt*0.1)+","+std::to_string(cnt*0.2)+","+std::to_string(cnt*0.4)+"\n"+std::to_string(cnt*0.01);
        udp_server->snd_msg_queue.push_back(test_snd_msg);
        this_thread::sleep_for(chrono::milliseconds(10));

        //simulate the haptic rendering and data reading
        cnt++;
    }
}





