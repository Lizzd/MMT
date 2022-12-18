//
// Created by zican on 05.08.22.
//

#include "gtest/gtest.h"
#include "PIDController.h"
#include "I_Robot.h"
#include "Falcon_robot_fd.h"


TEST(TESTPIDCONTROLLER,numberic_test)
{

    PIDController myController(100,0.01,-10);
//shared_ptr is used to manage the multi-modality of C++
    shared_ptr<I_Robot> falcon_robot_ptr= std::make_shared<Falcon_robot_fd>();

    ASSERT_TRUE(falcon_robot_ptr->is_device_ready())<<"please connect the device first before launching the program"<<endl;


    for(int i=0;i<100000;i++)
    {
        Eigen::Vector3d pos;
        falcon_robot_ptr->get_cartesian_position(pos);
        std::cout<<pos[0]<<","<<pos[1]<<","<<pos[2]<<std::endl;
    }
}