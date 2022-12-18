project(falcon_falcon_follow_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread" )
set(THREADS_PREFER_PTHREAD_FLAG ON)

SET(FALCON_FOLLOW_TASK_SOURCE
        Falcon_Falcon_Follow_Task.cpp
        )

SET(FALCON_CLIENT_TEST_SOURCE
        falcon_client_test.cpp)

SET(TWO_FALCON_ONE_VE_SOURCE
        two_falcon_one_ve_test.cpp)

SET(MY_TOOlS
        ${CMAKE_SOURCE_DIR}/Tools/StringTools/String_tools.cpp)

find_package (Eigen3 3.3 REQUIRED)

set(FORCE_DIMENSION_SDK_PATH /home/zican/zican/libraries/force_dimension/sdk-3.14.0-linux-x86_64-gcc/sdk-3.14.0)
set(FORCE_DIMENSION_SDK_LIB_PATH ${FORCE_DIMENSION_SDK_PATH}/lib/release/lin-x86_64-gcc)
set(FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)

include_directories(${FORCE_DIMENSION_SDK_PATH}/include)

link_directories(${FORCE_DIMENSION_SDK_LIB_PATH})

add_executable(test_falcon_falcon_follow ${FALCON_FOLLOW_TASK_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD} ${MY_TOOLS})
add_executable(test_falcon_client ${FALCON_CLIENT_TEST_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD} ${MY_TOOLS})
add_executable(test_two_falcon_with_one_ve ${TWO_FALCON_ONE_VE_SOURCE} ${ROBOT_INTERFACE} ${FALCON_ROBOT_FD} ${MY_TOOLS})

target_include_directories(test_falcon_falcon_follow
        PUBLIC
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
        ${ROBOT_INTERFACE_DIR}
        ${FALCON_ROBOT_FD_DIR}
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        ${CMAKE_SOURCE_DIR}/Tools
        )

target_link_libraries(test_falcon_falcon_follow PRIVATE Eigen3::Eigen)
target_link_libraries(test_falcon_falcon_follow PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
target_link_libraries(test_falcon_falcon_follow PRIVATE gtest gtest_main)
target_link_libraries(test_falcon_falcon_follow PRIVATE udp_communication)
target_link_libraries(test_falcon_falcon_follow PRIVATE robot_lib)

target_include_directories(test_falcon_client
        PUBLIC
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
        ${ROBOT_INTERFACE_DIR}
        ${FALCON_ROBOT_FD_DIR}
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        ${CMAKE_SOURCE_DIR}/Tools
        )

target_link_libraries(test_falcon_client PRIVATE Eigen3::Eigen)
target_link_libraries(test_falcon_client PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
target_link_libraries(test_falcon_client PRIVATE gtest gtest_main)
target_link_libraries(test_falcon_client PRIVATE udp_communication)
target_link_libraries(test_falcon_client PRIVATE robot_lib)

target_include_directories(test_two_falcon_with_one_ve
        PUBLIC
        ${CMAKE_SOURCE_DIR}/Robotics/Control/PID
        ${ROBOT_INTERFACE_DIR}
        ${FALCON_ROBOT_FD_DIR}
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        ${CMAKE_SOURCE_DIR}/Tools
        )

target_link_libraries(test_two_falcon_with_one_ve PRIVATE Eigen3::Eigen)
target_link_libraries(test_two_falcon_with_one_ve PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
target_link_libraries(test_two_falcon_with_one_ve PRIVATE gtest gtest_main)
target_link_libraries(test_two_falcon_with_one_ve PRIVATE udp_communication)
target_link_libraries(test_two_falcon_with_one_ve PRIVATE robot_lib)