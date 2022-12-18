set(ROBOTICS_SOURCE
        ../Robotics/Control/PID/PIDController.cpp
        )

set(ROBOTICS_INCLUDE
        ../Robotics/Control/PID/PIDController.h
        ../unit_tests/Robotics/Control/PIDController_test.cpp
        ../unit_tests/Robotics/Control/TDPAController_test.cpp
        ../Robotics/RobotAdaptor/I_Robot.h)


add_executable(AhtalKa_main
        main.cpp
        ${ROBOTICS_SOURCE}
        ${ROBOTICS_INCLUDE}
        )

target_link_libraries(AhtalKa_main
        Eigen3::Eigen
        )