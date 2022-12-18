//
// Created by zican on 01.08.22.
//
#include "gtest/gtest.h"
#include "Udp_core/Udp_client.h"
#include "Udp_core/Udp_server.h"
#include "Thread/Thread_safe_queue.h"

TEST(MAIN_TEST,first_google_test)
{
    // both Udp_server and Udp_client start their working threads on construction
    // by default both server/client create one sender and one receiver thread

    // the listening port parameter must be provided to Ude_server on construction
    Udp_server my_server(12345);

    // Udp_clients need the server addr:port information and its own listening port to be constructed
    Udp_client my_client(12344,"127.0.0.1",12345);

    // we would like to make sure that both my_server and my_client is successfully initialized
    while(!my_client.is_init_done()||!my_server.is_init_done()){this_thread::sleep_for(chrono::milliseconds(5));}
    cout<<"[main thread]server and client init done!"<<endl;

    //both the clients and server provides a thread-safe interface to send and fetch messages
    //normally we create new threads to process the msg received or build messages to be sent

    //example: using c++ lambda expression to create new threads
    //we need to pass the server/client object reference to get access to the message queue
    //also the non-stopping loop of the thread is controlled by a bool flag ref
    bool is_server_msg_consuming_thread_running=true;
    thread server_msg_consuming_thread([&my_server,&is_server_msg_consuming_thread_running]{
        cout<<"[server] starting server msg consuming thread..."<<endl;
        while(is_server_msg_consuming_thread_running){
            string msg;
            // get_one_msg unblocking check if the Thread_safe_queue in Udp_server is empty, and returns one msg if not so
            // this method also makes the Thread_safe_queue pops the message at the front, so the thread is called a msg consumer
            if(my_server.get_one_msg(msg)){
                // you can parse the message after receiving it
                cout<<"[server] received msg: "<<msg<<endl;
            }

            this_thread::yield();
            // this controls the maximum frequency at which the thread runs
            // for test output we use lower frequency, and 1Hz over wireless network is tested on raspberryPi
            this_thread::sleep_for(chrono::milliseconds(1000));
        }
    });
    //normally the msg consuming and generating thread run permanently, so we should call detach()
    server_msg_consuming_thread.detach();

    bool is_server_msg_generating_thread_running=true;
    thread server_msg_generating_thread([&my_server,&is_server_msg_generating_thread_running]{
        cout<<"[server] starting server msg consuming thread..."<<endl;
        int cnt=0;
        while(is_server_msg_generating_thread_running)
        {
            // build the string according to your protocol here as a std::string

            if(cnt>=100000)cnt=0;
            else cnt++;

            ostringstream msg_builder_stream;
            std::time_t cur_time=chrono::system_clock::to_time_t(chrono::system_clock::now());
            msg_builder_stream<<"time:"<<std::ctime(&cur_time);
            msg_builder_stream<<"cnt:"<<cnt<<"\n";
            msg_builder_stream<<"data:"<<cnt*0.01<<","<<cnt*0.02<<","<<cnt*0.04<<"\n";

            my_server.add_snd_request_to_all_client(msg_builder_stream.str());

            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1000));
        }

    });
    server_msg_generating_thread.detach();

    bool is_client_msg_generating_thread_running=true;
    thread client_msg_generating_thread([&my_client,&is_client_msg_generating_thread_running]{
        cout<<"[client] starting server msg consuming thread..."<<endl;
        int cnt=0;
        while(is_client_msg_generating_thread_running)
        {
            // build the string according to your protocol here as a std::string

            if(cnt>=100000)cnt=0;
            else cnt++;

            ostringstream msg_builder_stream;
            std::time_t cur_time=chrono::system_clock::to_time_t(chrono::system_clock::now());
            msg_builder_stream<<"time:"<<std::ctime(&cur_time);
            msg_builder_stream<<"cnt:"<<cnt<<"\n";
            msg_builder_stream<<"data:"<<cnt*0.04<<","<<cnt*0.01<<","<<cnt*0.02<<"\n";

            my_client.add_snd_request(msg_builder_stream.str());

            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1000));
    }
    });
    client_msg_generating_thread.detach();

    bool is_client_msg_consuming_thread_running=true;
    thread client_msg_consuming_thread([&my_client,&is_client_msg_consuming_thread_running]{
        cout << "[client] starting server msg consuming thread..." << endl;
        while (is_client_msg_consuming_thread_running) {
            string msg;
            // get_one_msg unblocking check if the Thread_safe_queue in Udp_server is empty, and returns one msg if not so
            // this method also makes the Thread_safe_queue pops the message at the front, so the thread is called a msg consumer
            if (my_client.get_one_msg(msg)) {
                // you can parse the message after receiving it
                cout << "[client] received msg: " << msg << endl;

                this_thread::yield();
                // this controls the maximum frequency at which the thread runs
                // for test output we use lower frequency, and 1Hz over wireless network is tested on raspberryPi
                this_thread::sleep_for(chrono::milliseconds(1000));
            }
        }
    });
    client_msg_consuming_thread.detach();

    this_thread::sleep_for(chrono::milliseconds(10000));
    is_server_msg_consuming_thread_running=false;
    is_server_msg_generating_thread_running=false;
    is_client_msg_generating_thread_running=false;
    is_client_msg_consuming_thread_running=false;


    cout<<"successfully shutdown all server"<<endl;


}

TEST(SECOND_TEST,test_thread_safe_queue) {
    Thread_safe_queue<string> my_queue;
    int is_writer_running=true;
    thread writerA([&my_queue,&is_writer_running]{
        while(is_writer_running)
        {
            my_queue.push_back("a very very very very long message");
            this_thread::yield();
        }
    });

    thread writerB([&my_queue,&is_writer_running]{
        while(is_writer_running)
        {
            my_queue.push_back("a very very very very long message");
            this_thread::yield();
        }
    });

    thread writerC([&my_queue,&is_writer_running]{
        while(is_writer_running)
        {
            my_queue.push_back("a very very very very long message");
            this_thread::yield();
        }
    });

    thread readerA([&my_queue]{
        for(int i=0;i<10000;i++)
        {
            auto ans=my_queue.wait_and_pop_front();
            ASSERT_EQ(*ans,"a very very very very long message");
            cout<<"reader A: "<<i<<endl;
            this_thread::yield();
        }
        cout<<"reader A done"<<endl;
    });

    thread readerB([&my_queue]{
        for(int i=0;i<10000;i++)
        {
            string ans;
            my_queue.wait_and_pop_front(ans);
            ASSERT_EQ(ans,"a very very very very long message");
            cout<<"reader B: "<<i<<endl;
            this_thread::yield();
        }
        cout<<"reader B done"<<endl;
    });

    readerA.join();
    readerB.join();
    is_writer_running=false;
}
