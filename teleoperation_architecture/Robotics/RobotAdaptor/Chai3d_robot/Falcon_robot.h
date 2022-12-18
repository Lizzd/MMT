//
// Created by zican on 22.08.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_H

#include "chai3d.h"

#include "I_Robot.h"
#include <string>

#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif

using namespace chai3d;


class Falcon_robot:public I_Robot {
private:

    static void print_under_construction_message(std::string function_name){std::cout<<function_name<<" under construction"<<std::endl;}

// chai3d need the glut environment to run
// these functions deal with the glut and chai3d initialization
    void initialize_glut_interface(int argc, char* argv[]);
    void initialize_chai3d_world(int argc, char* argv[]);
    static void updateGraphics(){/*do nothing*/};
    cWorld* world;

    //chai3d specific robot interface
    cGenericHapticDevicePtr hapticMasterDevice;

    static Eigen::Vector3d chai3d_vec_to_eigen(const chai3d::cVector3d & input){
        return Eigen::Vector3d{input.x(),input.y(),input.z()};
    }

    int check_all_limits() override{ print_under_construction_message(__FUNCTION__ );return -1;}
    //this method update all available information from the robot if possible
    bool update_from_robot()override{ print_under_construction_message(__FUNCTION__ );return false;}

    bool emergency_halt()override{ print_under_construction_message(__FUNCTION__ );return false;}

public:
// for robot running in the chai3d env., we must pass the argc and argv argument to initialize the glut env.
    Falcon_robot(int argc, char* argv[]);

    bool get_cartesian_position(Eigen::Vector3d& output_position) override;
    bool try_get_cartesian_position(Eigen::Vector3d& output_position)override{ print_under_construction_message(__FUNCTION__ );return false;} //non-block read
    bool get_cartesian_force(Eigen::Vector3d& output_force)override{ print_under_construction_message(__FUNCTION__ );return false;}
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
    bool set_cartesian_force(const Eigen::Vector3d& input_force)override{ print_under_construction_message(__FUNCTION__ );return false;}

};


#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_FALCON_ROBOT_H
