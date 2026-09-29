// can_hw.h
#pragma once
#include <stdio.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <stdbool.h> 
#include "math_ops.hpp"

extern int raw_sockets[6];

//电机IDs

// 控制参数范围

#define PI 3.14159265359f

// 控制数据结构体
// typedef struct {
//     float pos_des_;
//     float vel_des_;
//     float ff_;
//     float kp_;
//     float kd_;
// } YKSMotorData;



// 函数声明
// void sendCommand(int s_id, int numOfActuator, YKSMotorData *mot_data, int sock);
// void sendCanCommand(YKSMotorData *mot_data);
bool init_can();
void close_canDevice();
void sendFixedCommandAllMotors();
void send_fixed_command(int sock, int motor_id) ;
