project(test_main_function)

add_executable(test_main test_main.cpp)

target_include_directories(test_main
        PRIVATE
        ${CMAKE_SOURCE_DIR}/Udp_comm/include
        )

target_link_libraries(test_main udp_communication)

target_link_libraries(test_main gtest gtest_main)