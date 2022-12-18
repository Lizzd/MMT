cmake_minimum_required(VERSION 3.21)
project(test_Chai3d_Virtual_Environment_remote)

set(CMAKE_CXX_STANDARD 14)

set(CHAI3D-ODE_DIR ../chai3d-3.2.0)

find_package(CHAI3D REQUIRED)
find_package(GLUT REQUIRED)
find_package(GSL REQUIRED)
find_package(CHAI3D-ODE REQUIRED)
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

if(CHAI3D_FOUND)
    message(STATUS "Found CHAI3D: ${CHAI3D_SOURCE_DIR}")
endif()

set(GLUT_LIBRARY "/usr/lib/x86_64-linux-gnu/libglut.so;/usr/lib/x86_64-linux-gnu/libXi.so")

if(GLUT_FOUND)
    message(STATUS "Found GLUT: ${GLUT_LIBRARY}")
endif()

set(CHAI3D_VERSION 3.2.0)
set(CHAI3D_PATH /home/li/internet/chai3d-3.2.0)
SET(MY_TOOlS
        ${CMAKE_SOURCE_DIR}/Tools/StringTools/String_tools.cpp)

add_subdirectory(Udp_comm)

link_directories(${CHAI3D_LIBRARY_DIRS})

add_executable(test_Chai3d_Virtual_Environment_Server main.cpp ${MY_TOOLS})

target_include_directories(test_Chai3d_Virtual_Environment_Server PRIVATE ${CHAI3D_INCLUDE_DIRS} ${CHAI3D-ODE_INCLUDE_DIRS}
        ${CHAI3D_PATH}/modules/ODE/src
        ${CHAI3D_PATH}/modules/ODE/external/ODE/include
        ${CHAI3D_PATH}/modules/ODE/external/ODE/ode/src
        ${CHAI3D_PATH}/modules/ODE/external/ODE/ode/src/joints
        ${CHAI3D_PATH}/modules/ODE/external/ODE/OPCODE
        ${CHAI3D_PATH}/modules/ODE/external/ODE/OPCODE/Ice
        Udp_comm/include)
# target_link_directories(task_9_server_solution PRIVATE ${CHAI3D_LIBRARY_DIRS}) Use
# this if 3.14 is available
target_link_libraries(test_Chai3d_Virtual_Environment_Server PRIVATE ${CHAI3D_LIBRARIES} ${GLUT_LIBRARY} ${CHAI3D-ODE_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
target_link_libraries(test_Chai3d_Virtual_Environment_Server PRIVATE udp_communication)
