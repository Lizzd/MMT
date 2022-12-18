//
// Created by zican on 22.09.22.
//
#include "gtest/gtest.h"
#include "PIDController.h"
#include "I_Robot.h"
#include "Falcon_robot_fd.h"
#include "Udp_core/Udp_client.h"
#include "Udp_core/Udp_server.h"
#include "Thread/Thread_safe_queue.h"
#include "StringTools/String_tools.h"

#include <chrono>

TEST(TESTCLIENT,test_with_vitural_environment){
    shared_ptr<I_Robot> client_falcon_robot_ptr= std::make_shared<Falcon_robot_fd>();

    ASSERT_TRUE(client_falcon_robot_ptr->is_device_ready()) << "please connect the device first before launching the program" << endl;

    bool init_successful_flag=false;
    Eigen::Vector3d lastPos(0.0,0.0,0.0);
    client_falcon_robot_ptr->get_cartesian_position(lastPos);
    Eigen::Vector3d currentPos(0.0,0.0,0.0);
    double eps=0.0000001;
    int diff_data_cnt=0;
    while(!init_successful_flag)
    {
        client_falcon_robot_ptr->get_cartesian_position(currentPos);
        Eigen::Vector3d diff=lastPos-currentPos;
        if(diff.norm()>eps) diff_data_cnt++;
        if (diff_data_cnt > 100) init_successful_flag = true;
    }

    bool is_simulation_running=true;

    std::cout<<"starting test client..."<<endl;
    Udp_client my_client(12344,"127.0.0.1",12306);
    Eigen::Vector3d shared_target_position(0.0,0.0,0.0);
    int incoming_command_type=0;//0-position, 1=force
    thread message_reading_thread([&my_client,&shared_target_position,&incoming_command_type,&is_simulation_running] {
        cout << "[client] starting message reading thread..." << endl;
        while (is_simulation_running) {
            string msg;
            if (my_client.get_one_msg(msg)) {
                //currently, the rcv_data is like the following
                //e.g.
                //seq:100\n
                //timestamp:23.34\n
                //data:1.2,2.3,3.4\n

                try{
                    vector<string> tokens;
                    String_tools::string_split(msg, tokens, "\n");
                    string dataline = tokens[2];
                    size_t pos = 0;

                    vector<string> data;
                    if ((pos = dataline.find(':')) != string::npos) {
                        vector<string> parts;
                        String_tools::string_split(dataline, parts, ":");
                        string tag=parts[0];
                        if(tag=="position")incoming_command_type=0;
                        if(tag=="force")incoming_command_type=1;
                        dataline.erase(0, pos + 1);

                        String_tools::string_split(dataline, data, ",");
                    }
                    shared_target_position=Eigen::Vector3d(stod(data[0]), stod(data[1]), stod(data[2]));
                }catch(exception e){cout<<"haptic update error: "<<e.what()<<endl;}
            }

            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    message_reading_thread.detach();


    thread haptic_update_thread([&shared_target_position,&is_simulation_running,&incoming_command_type,client_falcon_robot_ptr]{
        int running_cnt = 0;
        PIDController myController(100,0.00,0);

        while(is_simulation_running)
        {

            Eigen::Vector3d targetPosition=shared_target_position;

            Eigen::Vector3d currentPosition(0.0, 0.0, 0.0);
            if (!client_falcon_robot_ptr->get_cartesian_position(currentPosition)) {
                cout << "get device position failed" << endl;
            }

            Eigen::Vector3d ControlForce;
            if(incoming_command_type==0)
                ControlForce = myController.getPIDControlForce(targetPosition, currentPosition);
            else
                ControlForce=shared_target_position;

            running_cnt++;
            if (running_cnt >= 500) {
                running_cnt = 0;
                cout << "pos from server: " << targetPosition.x() << " "
                     << targetPosition.y() << " "
                     << targetPosition.z() << endl;
                cout << "my current pos:  " << currentPosition.x() << " "
                     << currentPosition.y() << " "
                     << currentPosition.z() << endl;
                cout << "force instruction: " << ControlForce.x() << " " << ControlForce.y() << " "
                     << ControlForce.z()
                     << endl;
            }

            client_falcon_robot_ptr->set_cartesian_force(ControlForce);
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    haptic_update_thread.detach();

    thread send_msg_thread([&my_client,&is_simulation_running,client_falcon_robot_ptr] {
        cout << "[client] starting msg build&send thread" << endl;
        int running_cnt = 0;
        int seq = 0;

        auto start_time=chrono::steady_clock::now();

        while (is_simulation_running) {

            if (seq > 10000)seq = 0;
            else seq++;

            auto time_duration= chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start_time);
            int timestamp=time_duration.count();

            Eigen::Vector3d currentPosition;
            client_falcon_robot_ptr->get_cartesian_position(currentPosition);
            Eigen::Vector3d currentReactionForce;
            client_falcon_robot_ptr->get_cartesian_force(currentReactionForce);

            ostringstream msg_builder_stream;
            msg_builder_stream << "seq:" << seq << "\n";
            msg_builder_stream << "timestamp:" << timestamp << "\n";
            msg_builder_stream << "data:" << currentPosition.x() << "," << currentPosition.y() << ","
                               << currentPosition.z() << "\n";

            //cout<<"[client] sending msg: "<<msg_builder_stream.str()<<endl;

            my_client.add_snd_request(msg_builder_stream.str());
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    send_msg_thread.join();

}