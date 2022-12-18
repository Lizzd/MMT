project(my_test_robotics_control)

#######################################
# The following interface is defined in the parent CMakeLists.txt
#   SET(ROBOT_INTERFACE ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/I_Robot.h)
#   SET(ROBOT_INTERFACE_DIR ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/)
#   SET(FALCON_ROBOT ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot/Falcon_robot.cpp
#        ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot/Falcon_robot.h
#        )
#   SET(FALCON_ROBOT_DIR ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor/Chai3d_robot/)
########################################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread" )
set(THREADS_PREFER_PTHREAD_FLAG ON)

#set(TEST_CHAI3D TRUE)
set(TEST_FORCE_DIMENSION TRUE)

SET(ROBOTICS_CONTROL_PID_CONTROLLER
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID/PIDController.cpp
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID/PIDController.h
        )

SET(ROBOTICS_CONTROL_TEST_SOURCE
        PIDController_test.cpp
        TDPAController_test.cpp
        )

#################################### TEST FOR FORCE DIMENSION INTERFACE ################################################
if(TEST_FORCE_DIMENSION)
    find_package (Eigen3 3.3 REQUIRED)

    set(FORCE_DIMENSION_SDK_PATH /home/zican/zican/libraries/force_dimension/sdk-3.14.0-linux-x86_64-gcc/sdk-3.14.0)
    set(FORCE_DIMENSION_SDK_LIB_PATH ${FORCE_DIMENSION_SDK_PATH}/lib/release/lin-x86_64-gcc)
    set(FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)

    include_directories(${FORCE_DIMENSION_SDK_PATH}/include)

    link_directories(${FORCE_DIMENSION_SDK_LIB_PATH})

    add_executable(test_robotics_control ${ROBOTICS_CONTROL_PID_CONTROLLER} ${ROBOTICS_CONTROL_TEST_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD})

    target_include_directories(test_robotics_control
            PUBLIC
            ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
            ${ROBOT_INTERFACE_DIR}
            ${FALCON_ROBOT_FD_DIR}
            )

    target_link_libraries(test_robotics_control PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
    target_link_libraries(test_robotics_control PRIVATE Eigen3::Eigen)
    target_link_libraries(test_robotics_control PRIVATE gtest gtest_main)
endif()

########################################## TEST FOR CHAI3D INTERFACE ###################################################
if(TEST_CHAI3D)
    find_package(CHAI3D REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(Threads REQUIRED)
    find_package(CHAI3D-ODE REQUIRED)
    find_package(Boost REQUIRED)
    find_package(Boost COMPONENTS system REQUIRED)

    if(CHAI3D_FOUND)
        message(STATUS "Found CHAI3D: ${CHAI3D_SOURCE_DIR}")
    endif()

    if(CHAI3D-ODE_FOUND)
        message(STATUS "Found CHAI3D-ODE: ${CHAI3D-ODE_INCLUDE_DIRS}")
    endif()

    SET(CHAI3D_DIR
            ${CHAI3D_INCLUDE_DIRS} ${CHAI3D-ODE_INCLUDE_DIRS}
            ${CHAI3D_PATH}/modules/ODE/src
            ${CHAI3D_PATH}/modules/ODE/external/ODE/include
            ${CHAI3D_PATH}/modules/ODE/external/ODE/ode/src
            ${CHAI3D_PATH}/modules/ODE/external/ODE/ode/src/joints
            ${CHAI3D_PATH}/modules/ODE/external/ODE/OPCODE
            ${CHAI3D_PATH}/modules/ODE/external/ODE/OPCODE/Ice
            )

    set(CHAI3D_VERSION 3.2.0)
    set(CHAI3D_PATH /home/zican/zican/fox/00_LMT/02_materials/2021_10_raspberry/20211114_CHL_backup/chai3d-3.2.0)
    link_directories(${CHAI3D_LIBRARY_DIRS})

    add_executable(test_robotics_control ${ROBOTICS_CONTROL_TEST_SOURCE} ${ROBOTICS_CONTROL_PID_CONTROLLER}
            ${ROBOT_INTERFACE} ${FALCON_ROBOT}
            )

    target_include_directories(test_robotics_control
            PRIVATE
            ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
            ${ROBOT_INTERFACE_DIR}
            ${FALCON_ROBOT_DIR}
            ${CHAI3D_DIR}
            )

    message(STATUS ${FALCON_ROBOT_DIR})

    target_link_libraries(test_robotics_control PRIVATE gtest gtest_main)
    target_link_libraries(test_robotics_control PRIVATE Eigen3::Eigen)
    target_link_libraries(test_robotics_control PRIVATE ${CHAI3D_LIBRARIES} ${GLUT_LIBRARY} ${CHAI3D-ODE_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
endif()