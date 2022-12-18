//
// Created by zican on 23.09.22.
//

#include "PIDController.h"
#include "I_Robot.h"
#include "Falcon_robot_fd.h"
#include "Udp_core/Udp_client.h"
#include "Udp_core/Udp_server.h"
#include "Thread/Thread_safe_queue.h"
#include "StringTools/String_tools.h"

#include <chrono>

int main(){
    // main robot side
    // need both client and server
    shared_ptr<I_Robot> server_falcon_robot_ptr= std::make_shared<Falcon_robot_fd>();

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
//    Udp_client main_robot_ve_client(19991,"192.168.1.122",12306);
    Udp_client main_robot_ve_client(19991,"127.0.0.1",12306);
    Eigen::Vector3d target_reaction_force(0.0,0.0,0.0);
    thread read_ve_force_thread([&main_robot_ve_client,&is_simulation_running,&target_reaction_force]{
        cout<<"starting haptic rendering thread..."<<endl;
        while(is_simulation_running){

            string tmp;
            if(main_robot_ve_client.get_one_msg(tmp))
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
                    target_reaction_force=Eigen::Vector3d(stod(data[0]),stod(data[1]),stod(data[2]));

                    //render data
                }catch (exception e){cerr<<"parse error: "<<e.what()<<endl;}
                //now we just parse the data for the haptic demo

            }
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    read_ve_force_thread.detach();

    thread render_haptic_thread([&target_reaction_force,&is_simulation_running,server_falcon_robot_ptr]{
        PIDController my_pid_controller(300, 0.1, 10);

        while(is_simulation_running)
        {
            //temporally us PP structure
            Eigen::Vector3d currentPos;
            server_falcon_robot_ptr->get_cartesian_position(currentPos);

            //Eigen::Vector3d target_force=my_pid_controller.getPIDControlForce(target_reaction_pos,currentPos);

            //cVector3d target_reaction_force=-target_reaction_force;
            server_falcon_robot_ptr->set_cartesian_force(target_reaction_force);
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    render_haptic_thread.detach();

    thread msg_generator_thread([&my_server,&main_robot_ve_client,server_falcon_robot_ptr,&is_simulation_running]{
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

//            my_server.add_snd_request_to_all_client(msg.str());
            main_robot_ve_client.add_snd_request(msg.str());
            this_thread::yield();
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
    msg_generator_thread.join();

}
