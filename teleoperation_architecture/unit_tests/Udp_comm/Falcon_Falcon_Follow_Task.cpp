//
// Created by zican on 21.09.22.
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

TEST(TESTFALCONFALCONFOLLOW,test_with_one_pc)
{
    // server side
    // also 3 threads
    shared_ptr<I_Robot> server_falcon_robot_ptr= std::make_shared<Falcon_robot_fd>();
    ASSERT_TRUE(server_falcon_robot_ptr->is_device_ready())<<"please connect the device first before launching the program"<<endl;

    bool init_successful_flag=false;
    Eigen::Vector3d lastPos(0.0,0.0,0.0);
    server_falcon_robot_ptr->get_cartesian_position(lastPos);
    Eigen::Vector3d currentPos(0.0,0.0,0.0);
    double eps=0.0000001;
    int diff_data_cnt=0;
    while(!init_successful_flag)
    {
        server_falcon_robot_ptr->get_cartesian_position(currentPos);
        Eigen::Vector3d diff=lastPos-currentPos;
        if(diff.norm()>eps) diff_data_cnt++;
        if (diff_data_cnt > 100) init_successful_flag = true;
    }

    bool is_simulation_running=true;
    std::cout<<"starting test server..."<<endl;

    Udp_server my_server(12345);
    Eigen::Vector3d target_reaction_pos(0.0,0.0,0.0);
    thread read_msg_thread([&my_server,&is_simulation_running,&target_reaction_pos]{
        cout<<"starting haptic rendering thread..."<<endl;
        while(is_simulation_running){

            string tmp;
            if(my_server.get_one_msg(tmp))
            {
                try{
                    //cout<<"[test]"<<tmp<<endl;
                    vector<string> tokens;
                    String_tools::string_split(tmp,tokens,"\n");
                    string dataline=tokens[2];

                    //cout<<"[test]dataline:"<<dataline<<endl;

                    size_t pos=0;

                    vector<string> data;
                    if((pos=dataline.find(':'))!=string::npos)
                    {
                        dataline.erase(0,pos+1);
                        //cout<<"[test]new dataline:"<<dataline<<endl;

                        String_tools::string_split(dataline,data,",");

                    }

                    //cout<<"[test]"<<data[0]<<" "<<data[1]<<" "<<data[2]<<endl;
                    target_reaction_pos=Eigen::Vector3d(stod(data[0]),stod(data[1]),stod(data[2]));

                    //render data
                }catch (exception e){cerr<<"parse error: "<<e.what()<<endl;}
                //now we just parse the data for the haptic demo

            }
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    read_msg_thread.detach();

    thread render_haptic_thread([&target_reaction_pos,&is_simulation_running,server_falcon_robot_ptr]{
        PIDController my_pid_controller(100, 0, 0);

        while(is_simulation_running)
        {
            //temporally us PP structure
            Eigen::Vector3d currentPos;
            server_falcon_robot_ptr->get_cartesian_position(currentPos);

            Eigen::Vector3d target_force=my_pid_controller.getPIDControlForce(target_reaction_pos,currentPos);

            //cVector3d target_reaction_force=-target_reaction_force;
            server_falcon_robot_ptr->set_cartesian_force(target_force);
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    render_haptic_thread.detach();

    thread msg_generator_thread([&my_server,server_falcon_robot_ptr,&is_simulation_running]{
        cout<<"starting msg generator thread..."<<endl;

        //in this thread we take out the msg from the server
        //and send the info (received or calculated force) to the device

        int seq=0;
        auto start_time=chrono::steady_clock::now();

        while(is_simulation_running) {
            if (seq > 10000)seq = 0;
            else seq++;

            auto time_duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_time);
            int timestamp = time_duration.count();


            Eigen::Vector3d currentPosition(0, 0, 0);
            server_falcon_robot_ptr->get_cartesian_position(currentPosition);
            //format msg
            ostringstream msg;
            msg << "seq:" << seq << "\n";
            msg << "timestamp:" << timestamp << "\n";
            msg << "data:" << currentPosition.x() << "," << currentPosition.y() << "," << currentPosition.z() << "\n";


            my_server.add_snd_request_to_all_client(msg.str());
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    msg_generator_thread.detach();


    // client side
    // 3 threads for listening, haptic updating and sending

    shared_ptr<I_Robot> client_falcon_robot_ptr= std::make_shared<Falcon_robot_fd>();

    ASSERT_TRUE(client_falcon_robot_ptr->is_device_ready()) << "please connect the device first before launching the program" << endl;

    init_successful_flag=false;
    lastPos.setZero();
    client_falcon_robot_ptr->get_cartesian_position(lastPos);
    currentPos.setZero();
    diff_data_cnt=0;
    while(!init_successful_flag)
    {
        client_falcon_robot_ptr->get_cartesian_position(currentPos);
        Eigen::Vector3d diff=lastPos-currentPos;
        if(diff.norm()>eps) diff_data_cnt++;
        if (diff_data_cnt > 100) init_successful_flag = true;
    }


    std::cout<<"starting test client..."<<endl;
    Udp_client my_client(12344,"127.0.0.1",12345);
    Eigen::Vector3d shared_target_position(0.0,0.0,0.0);
    thread message_reading_thread([&my_client,&shared_target_position,&is_simulation_running] {
        cout << "[client] starting haptic rendering thread..." << endl;
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


    thread haptic_update_thread([&shared_target_position,&is_simulation_running,client_falcon_robot_ptr]{
        int running_cnt = 0;
        PIDController myController(100,0.00,0);

        while(is_simulation_running)
        {

            Eigen::Vector3d targetPosition=shared_target_position;

            Eigen::Vector3d currentPosition(0.0, 0.0, 0.0);
            if (!client_falcon_robot_ptr->get_cartesian_position(currentPosition)) {
                cout << "get device position failed" << endl;
            }

            Eigen::Vector3d ControlForce = myController.getPIDControlForce(targetPosition, currentPosition);

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