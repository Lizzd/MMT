project(01_two_falcon_follow_and_one_ve)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread" )
set(THREADS_PREFER_PTHREAD_FLAG ON)

SET(MAIN_ROBOT_SOURCE
        Main_robot/main_robot.cpp
        )

SET(SECONDARY_ROBOT_SOURCE
        Secondary_robot/secondary_robot.cpp)

SET(MY_TOOlS
        ${CMAKE_SOURCE_DIR}/Tools/StringTools/String_tools.cpp)

find_package (Eigen3 3.3 REQUIRED)

set(FORCE_DIMENSION_SDK_PATH /home/li/internet/force_dimension/sdk-3.14.0-linux-x86_64-gcc/sdk-3.14.0)
set(FORCE_DIMENSION_SDK_LIB_PATH ${FORCE_DIMENSION_SDK_PATH}/lib/release/lin-x86_64-gcc)
set(FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)

include_directories(${FORCE_DIMENSION_SDK_PATH}/include)

link_directories(${FORCE_DIMENSION_SDK_LIB_PATH})

add_executable(01_main_robot ${MAIN_ROBOT_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD} ${MY_TOOLS})
add_executable(01_secondary_robot ${SECONDARY_ROBOT_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD} ${MY_TOOLS})

target_include_directories(01_main_robot
        PUBLIC
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
        ${ROBOT_INTERFACE_DIR}
        ${FALCON_ROBOT_FD_DIR}
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        ${CMAKE_SOURCE_DIR}/Tools
        )

target_link_libraries(01_main_robot PRIVATE Eigen3::Eigen)
target_link_libraries(01_main_robot PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
target_link_libraries(01_main_robot PRIVATE udp_communication)
target_link_libraries(01_main_robot PRIVATE robot_lib)

target_include_directories(01_secondary_robot
        PUBLIC
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
        ${ROBOT_INTERFACE_DIR}
        ${FALCON_ROBOT_FD_DIR}
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        ${CMAKE_SOURCE_DIR}/Tools
        )

target_link_libraries(01_secondary_robot PRIVATE Eigen3::Eigen)
target_link_libraries(01_secondary_robot PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
target_link_libraries(01_secondary_robot PRIVATE udp_communication)
target_link_libraries(01_secondary_robot PRIVATE robot_lib)
