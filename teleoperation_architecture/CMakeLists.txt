cmake_minimum_required(VERSION 3.0)
project(Teleoperation_architecture_AhtalKa)

find_package (Eigen3 3.3 REQUIRED)

# robot interface variables
SET(ROBOT_INTERFACE ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/I_Robot.h)
SET(ROBOT_INTERFACE_DIR ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor)

SET(FALCON_ROBOT ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot/Falcon_robot.cpp
        ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot/Falcon_robot.h
        )

SET(FALCON_ROBOT_DIR ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot)

SET(FALCON_ROBOT_FD ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp
        ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.h)

SET(FALCON_ROBOT_FD_DIR ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Force_dimension_robot)



include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

#build the libraries before the projects
add_subdirectory(Udp_comm)
add_subdirectory(Robotics)

#add_subdirectory(external/Google_test/lib)
#add_subdirectory(external/Google_test)
#add_subdirectory(main)
#add_subdirectory(unit_tests)
add_subdirectory(examples/01_two_falcon_follow_and_one_ve)



