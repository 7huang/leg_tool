#pragma once

#include "time.h"
#include <iostream>
#include <chrono>
#include <cstdio>
#include <cstring>  
#include <thread>   
#include <iomanip> 

#include "ec_api_without_ros.hpp"

//正弦运动
static double angle_rad; //下发弧度
static float duration_sec = 10; //总时长
static float amplitude_rad = 0.4; //振幅
static float sin_frequency_hz = 0.2; //频率

//关节逐个旋转
static int rate = 500; //比例
static int time_factor = rate * 8; //4000：4秒一个关节

//指令
static MotorCmd cmd_test;

/*电机id号初始化*/
void test_id_init();

/*打印测试*/
void test_print();

/*电机逐个旋转测试*/
void test_pos_one_by_one();

/*电机一起旋转测试*/
void test_sin_wave_all();

/*pd控制测试*/
void test_stand_pd();

/*设置*/
void setting_tool();

void test_ankel_solver();

void test_waist_solver();
