//
// Created by zican on 08.09.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_FD_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_FD_H

#include "I_Robot.h"

class Falcon_robot_fd:public I_Robot {
private:
    char dhd_id=-1;
    bool is_ready=false;
    static bool local_connected_falcon_id[10];

public:
// for robot running in the chai3d env., we must pass the argc and argv argument to initialize the glut env.
    Falcon_robot_fd();

    bool get_cartesian_position(Eigen::Vector3d& output_position) override;
    bool try_get_cartesian_position(Eigen::Vector3d& output_position)override{ print_under_construction_message(__FUNCTION__ );return false;} //non-block read
    bool get_cartesian_force(Eigen::Vector3d& output_force)override;
    bool try_get_cartesian_force(Eigen::Vector3d& output_force)override{ print_under_construction_message(__FUNCTION__ );return false;} //non_block read
    int get_number_of_joints()override{ print_under_construction_message(__FUNCTION__ );return -1;}
    bool is_joint_controllable()override{ print_under_construction_message(__FUNCTION__ );return false;}


    //write methods
    //return false if the robot does not support this writing function
    //all write methods set the target methods of the robot but do not guarantee that the robot reach the target at the end of the call
    //so all write methods are naturally non-blocking
    //the implement of the interface has the versatility of deciding if the target is reached and the function should be blocked
    bool set_joint_position(const Eigen::Vector3d& input_position,int joint_number)override{ print_under_construction_message(__FUNCTION__ );return false;}
    bool set_joint_speed(const Eigen::Vector3d& input_speed,int joint_number)override{ print_under_construction_message(__FUNCTION__ );return false;}
    //set cartesian position may be achieved by calling the internal controller of the robot,
    //or by a position_tracking controller defined by the user/project
    bool set_cartesian_position(const Eigen::Vector3d& input_position)override{ print_under_construction_message(__FUNCTION__ );return false;}
    bool set_cartesian_speed(const Eigen::Vector3d& input_speed)override{ print_under_construction_message(__FUNCTION__ );return false;}
    bool set_cartesian_acceleration(const Eigen::Vector3d& input_accel)override{ print_under_construction_message(__FUNCTION__ );return false;}
    bool set_cartesian_force(const Eigen::Vector3d& input_force)override;

    bool is_device_ready(){return is_ready;}
    bool set_device_id(char _dhd_id){dhd_id=_dhd_id;return true;};
    char get_device_id(){return dhd_id;}

private:

    static void print_under_construction_message(std::string function_name){std::cout<<function_name<<" under construction"<<std::endl;}

    int check_all_limits() override{ print_under_construction_message(__FUNCTION__ );return -1;}
    //this method update all available information from the robot if possible
    bool update_from_robot()override{ print_under_construction_message(__FUNCTION__ );return false;}

    bool emergency_halt()override{ print_under_construction_message(__FUNCTION__ );return false;}

};


#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_FD_H
