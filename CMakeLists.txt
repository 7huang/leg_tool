cmake_minimum_required(VERSION 3.0.2)
project(leg_tool)

find_package(Threads REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

add_subdirectory(encos)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  encos
  encos/thirdparty/canfd/include
)

set(SOURCES encos/motor_control.c
            encos/motor_control.h
            encos/math_ops.c
            encos/math_ops.h
            encos/transmit.cpp
            encos/transmit.h
            encos/transmit_fd.cpp
            encos/config.h
            encos/queue.h
            encos/command.cpp 
            encos/command.h
            encos/pace_stamp.cpp
            encos/pace_stamp.h
            include/ec_api_without_ros.hpp
            src/ec_api_without_ros.cpp
            include/test_func.hpp
            src/test_func.cpp
            include/close_chain_mapping.h
            src/close_chain_mapping.cpp
            include/parallelmechanism.h
            src/parallelmechanism.cpp
            )
            
LINK_LIBRARIES(m)
add_library(encos_motor ${SOURCES})
target_link_libraries(encos_motor keenon_canfd)

add_library(keenon_lf1 SHARED ${SOURCES})
target_link_libraries(keenon_lf1 keenon_canfd)

## Declare a C++ executable
add_executable(leg_tool src/ec_api_without_ros.cpp src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leg_tool
  ${catkin_LIBRARIES}
  encos_motor
  Threads::Threads
)

## PACE 最小实验: 单关节 chirp 激励 + 指令/反馈时间戳对齐记录
add_executable(pace_single_joint src/pace_single_joint.cpp)
target_link_libraries(pace_single_joint encos_motor Threads::Threads)

