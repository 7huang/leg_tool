cmake_minimum_required(VERSION 3.10.2)
project(CanFD)

add_compile_options(-std=c++11)
find_package(Threads REQUIRED)

# ========== 添加头文件搜索路径 ==========
include_directories(include)

# 添加 encos 目录（command.h 所在位置）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_library(keenon_canfd SHARED src/can_hw.cpp src/can_util.cpp src/encos.cpp)