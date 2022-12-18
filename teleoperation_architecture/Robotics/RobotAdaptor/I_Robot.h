//
// Created by zican on 08.08.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_I_ROBOT_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_I_ROBOT_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

// interface of all robot
class I_Robot{
protected:
    int number_of_joints=0;
    struct Limits{
        double upper_limit=0.0;
        double lower_limit=-0.0;
        Limits(double _upper,double _lower):upper_limit(_upper),lower_limit(_lower){
            if(upper_limit<lower_limit)
            {
                std::cerr<<"Limits Error: upper_limit_lower_limit";
                throw std::exception();
            }
        }
        Limits()=default;
    };
    std::vector<Limits> joint_position_limit_vec;
    std::vector<Limits> joint_speed_limit_vec;
    std::vector<Limits> joint_acceleration_limit_vec;
    std::vector<Limits> cartesian_position_limit_vec;
    std::vector<Limits> cartesian_speed_limit_vec;
    std::vector<Limits> cartesian_acceleration_limit_vec;

    std::vector<double> joint_positions;
    std::vector<double> joint_speed;
    std::vector<double> joint_acceleration;
    Eigen::Vector3d cartesian_position{0.0,0.0,0.0};
    Eigen::Vector3d cartesian_speed{0.0,0.0,0.0};
    Eigen::Vector3d cartesian_acceleration{0.0,0.0,0.0};
    Eigen::Vector3d cartesian_force{0.0,0.0,0.0};


    virtual int check_all_limits()=0;
    //this method update all available information from the robot if possible
    virtual bool update_from_robot()=0;

    virtual bool emergency_halt()=0;

public:
    I_Robot()=default;

    I_Robot(int _number_of_joints):
            number_of_joints(_number_of_joints),
            joint_position_limit_vec(_number_of_joints),
            joint_speed_limit_vec(_number_of_joints),
            joint_acceleration_limit_vec(_number_of_joints),
            cartesian_acceleration_limit_vec(3),
            cartesian_speed_limit_vec(3),
            cartesian_position_limit_vec(3),
            joint_positions(_number_of_joints),
            joint_speed(_number_of_joints),
            joint_acceleration(_number_of_joints)
    {};

    //read methods
    //return false if this information is not available (which is common)
    virtual bool get_cartesian_position(Eigen::Vector3d& output_position)=0;
    virtual bool try_get_cartesian_position(Eigen::Vector3d& output_position)=0; //non-block read
    virtual bool get_cartesian_force(Eigen::Vector3d& output_force)=0;
    virtual bool try_get_cartesian_force(Eigen::Vector3d& output_force)=0; //non_block read
    virtual int get_number_of_joints()=0;
    virtual bool is_joint_controllable()=0;


    //write methods
    //return false if the robot does not support this writing function
    //all write methods set the target methods of the robot but do not guarantee that the robot reach the target at the end of the call
    //so all write methods are naturally non-blocking
    //the implement of the interface has the versatility of deciding if the target is reached and the function should be blocked
    virtual bool set_joint_position(const Eigen::Vector3d& input_position,int joint_number)=0;
    virtual bool set_joint_speed(const Eigen::Vector3d& input_speed,int joint_number)=0;
    //set cartesian position may be achieved by calling the internal controller of the robot,
    //or by a position_tracking controller defined by the user/project
    virtual bool set_cartesian_position(const Eigen::Vector3d& input_position)=0;
    virtual bool set_cartesian_speed(const Eigen::Vector3d& input_speed)=0;
    virtual bool set_cartesian_acceleration(const Eigen::Vector3d& input_accel)=0;
    virtual bool set_cartesian_force(const Eigen::Vector3d& input_force)=0;

    virtual bool is_device_ready()=0;

    virtual ~I_Robot()=default;

};

#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_I_ROBOT_H
