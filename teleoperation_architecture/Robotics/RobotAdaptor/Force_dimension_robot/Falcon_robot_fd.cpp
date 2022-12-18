//
// Created by zican on 08.09.22.
//

#include "Falcon_robot_fd.h"
#include <iostream>
#include <Eigen/Dense>

#include "dhdc.h"

using namespace std;

//array initialization
bool Falcon_robot_fd::local_connected_falcon_id[]{};

Falcon_robot_fd::Falcon_robot_fd(): I_Robot(3) {
    // check the newest un-occupied device id
    // we suppose there are no more than 10 falcon connected to one device (which is far more than enough)
    int usable_id=0;
    for(int i=0;i<10;i++)
    {
        if(!Falcon_robot_fd::local_connected_falcon_id[i])
        {
            usable_id=i;
            Falcon_robot_fd::local_connected_falcon_id[i]=true;
            break;
        }
    }
    char new_dhd_id=dhdOpenID(usable_id);
    if (new_dhd_id< 0)
    {
        std::cout << "error: cannot open device" << std::endl;
        std::cout<<"try all other possible ID"<<std::endl;
        Falcon_robot_fd::local_connected_falcon_id[usable_id]=false;
        for (int i=usable_id;i<10;i++)
        {
            char try_dhd_id=dhdOpenID(i);
            if(try_dhd_id>=usable_id) {
                dhd_id=new_dhd_id;
                is_ready=true;
                cout<<"new falcon id: "<<usable_id<<endl;
                break;
            }
        }
    }
    else
    {
        dhd_id=new_dhd_id;
        is_ready=true;
        cout<<"new falcon id: "<<usable_id<<endl;
    }
}

bool Falcon_robot_fd::get_cartesian_force(Eigen::Vector3d &output_force) {
    if(!is_ready)return false;
    double px,py,pz;
    dhdGetForce(&px, &py, &pz,dhd_id);
    output_force[0]=px;
    output_force[0]=py;
    output_force[0]=pz;
    return true;
}

bool Falcon_robot_fd::get_cartesian_position(Eigen::Vector3d &output_position)
{
    if(!is_ready)return false;
    double px,py,pz;
    dhdGetPosition(&px, &py, &pz,dhd_id);
    output_position[0]=px;
    output_position[1]=py;
    output_position[2]=pz;
    return true;
}

bool Falcon_robot_fd::set_cartesian_force(const Eigen::Vector3d &input_force) {
    if(!is_ready)return false;
    dhdSetForce(input_force[0], input_force[1], input_force[2],dhd_id);
    return true;

}