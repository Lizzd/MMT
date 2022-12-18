find_package (Eigen3 3.3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread" )
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CONTROL_ALGORITHMS_FILES
        Control/PID/PIDController.h
        Control/PID/PIDController.cpp)

set(ROBOT_ADAPTOR_FILES
        RobotAdaptor/I_Robot.h
        RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.cpp
        RobotAdaptor/Force_dimension_robot/Falcon_robot_fd.h)

set(FORCE_DIMENSION_SDK_PATH ${CMAKE_SOURCE_DIR}/external/force_dimension/sdk-3.14.0-linux-x86_64-gcc/sdk-3.14.0)
set(FORCE_DIMENSION_SDK_LIB_PATH ${FORCE_DIMENSION_SDK_PATH}/lib/release/lin-x86_64-gcc)
set(FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)

include_directories(${FORCE_DIMENSION_SDK_PATH}/include)
link_directories(${FORCE_DIMENSION_SDK_LIB_PATH})

add_library(robot_lib STATIC ${CONTROL_ALGORITHMS_FILES} ${ROBOT_ADAPTOR_FILES})

target_include_directories(robot_lib PRIVATE ${CMAKE_SOURCE_DIR}/Robotics/RobotAdaptor)

target_link_libraries(robot_lib PRIVATE Eigen3::Eigen)
target_link_libraries(robot_lib PRIVATE dhd ${FORCE_DIMENSION_SDK_DEPENDENCY_LIBRARIES})
