find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread" )
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(UDP_COMM_HEADER_FILES
        include/Udp_core/Udp_client.h
        include/Udp_core/Udp_server.h
        include/Udp_core/Udp_receiver.h
        include/Udp_core/Udp_sender.h
        )

set(UDP_COMM_HEADER_FILES
        src/Udp_core/Udp_client.cpp
        src/Udp_core/Udp_server.cpp
        src/Udp_core/Udp_receiver.cpp
        src/Udp_core/Udp_sender.cpp
        )

add_library(udp_communication STATIC ${UDP_COMM_HEADER_FILES} ${UDP_COMM_HEADER_FILES})

target_include_directories(udp_communication
        PRIVATE
        ./include/Udp_core
        ./include
        )

target_link_libraries(udp_communication
        PRIVATE
        ${Boost_LIBRARIES}
        ${Boost_SYSTEM_LIBRARY}
        )

