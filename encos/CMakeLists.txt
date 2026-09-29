cmake_minimum_required(VERSION 3.5)
project(MasterStack)

add_subdirectory(thirdparty)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(
        thirdparty/canfd/include
)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES motor_control.c
            motor_control.h
            math_ops.c
            math_ops.h
            transmit.cpp
            transmit.h
            transmit_fd.cpp
            transmit_fd.h
            config.h
            queue.h command.cpp command.h
            pace_stamp.cpp pace_stamp.h)
LINK_LIBRARIES(m)
add_library(motor ${SOURCES})
target_link_libraries(motor keenon_canfd)
