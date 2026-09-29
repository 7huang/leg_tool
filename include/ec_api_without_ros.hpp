#pragma once

#include <cstdio>
#include <boost/bind.hpp>
#include "command.h"
#include "time.h"
#include "queue.h"
#include "motor_control.h"
#include "transmit_fd.h"
#include "config.hpp"
#include "can_hw.hpp"
#include <queue>
#include "parallelmechanism.h"
#include "close_chain_mapping.h"
extern "C" {
//左腿，右腿，腰，左臂，右臂
const float lf1_zero_offset_deg[29] = {0,-90,0,-8,0,0,
                            -0,90,0,8,0,0,
                            0,0,0,
                            164,-95,0,122,0,0,0,
                            -164,95,0,-122,0,0,0
};

const float lf1_zero_offset_rad[29] = {0,-1.5707,0,-0.1396,0,0,
                            -0,1.5707,0,0.1396,0,0,
                            0,0,0,
                            2.8623,-1.658,0,2.1292,0,0,0,
                            -2.8623,1.658,0,-2.1292,0,0,0
};
const int can_usleep = 1150; //报文下发间隔
const int motor_number_ = 29;
extern bool show_debug_info;
extern bool safe_torque_mode;

typedef struct{
    int8_t motor_id[motor_number_];
    float q_des[motor_number_];
    float qd_des[motor_number_];
    float kp[motor_number_];
    float kd[motor_number_];
    float kd_ff[2];
    float tff[motor_number_];
    int8_t mode[1]; //1 = position, 2 = force-position
    uint32_t msg_cmd;
} MotorCmd;
#pragma pack(push, 1)  
typedef struct{
    int8_t motor_id[motor_number_];
    float q[motor_number_];
    float qd[motor_number_];
    float tau[motor_number_];
    uint32_t msg_state;
} MotorState;
#pragma pack(pop)
extern MotorCmd motor_cmd;
extern MotorState motor_state_output;

/*
cmdCallback()：回调函数
    每次收到电机控制指令，就将控制指令封装，并存储到无锁队列A中，以便发送线程读取。
    然后读取接收线程的消息队列B，解包获得电机信息。
*/ 
    void cmdCallback(MotorCmd* motor_cmd);

/*
check_safe_pos()：检测电机位置是否到达限位
    return：true 当位置安全时
    return：false 当超限时
*/
    bool check_safe_pos(float current_pos, MotorRealLimits motor_real_limits);

/*
check_safe_torque()：检测单个电机力矩是否到达限位
    return：true 当力矩安全时
    return：false 当超限时
*/
    bool check_safe_torque(float current_torque, MotorRealLimits motor_real_limits);

/*
check_safe_torque_multi()：检测motor_number个电机力矩是否到达限位
    内部调用了check_safe_torque()
    return：true 当所有力矩安全时
    return：false 当任意力矩超限时
*/
    bool check_safe_torque_multi();

/*
safe_torque_break()：检测motor_number个电机力矩是否到达限位，任意电机超限，则所有电机刹车。
    内部调用了check_safe_torque_multi()
*/
    void safe_torque_break();

/*
motorBreak()：电机刹车
*/
    void motorBreak();

/*
motorBackToZero()：使用位置模式，将所有电机回零
*/
    void motorBackToZero();

/*
motorSetZero()：将当前电机位置设置为零点
    在命令行中输入ID号（1-motor_number），对单个电机标零
*/
    void motorSetZero();

/*
idCheck()：检查电机ID1-motor_number
    return：true 当所有id找到时
    return：false 当任意id没有找到时
*/
    bool idCheck();

/*
posCheck()：检查motor_number个关节的位置角度
*/
    void posCheck();

/*
posCheckTest()：一个自定的测试单关节读取的函数
*/
    void posCheckTest();

/*
idSet()：设置单个关节id
*/
    bool idSet();

/*
sendToEC()：将封好的报文入队，供EC读取
*/
    void sendToEC();

/*
getFromEC()：从EC取出报文，并解包
*/
    void getFromEC();

/*
sendMotorCmd()：下发/motor_cmd指令
*/
    void sendMotorCmd(MotorCmd* motor_cmd);

    void sendMotorCmd_();

/*
motorState_()：获取电机信息
    使用方法：参考sendMotorCmd()
    从EC队列中取出数据，并解包，发布。
    loop:因为接收motor_number个电机需要2次EC循环，所以第一次循环（loop=0）只是填充数组（6个电机数据），只有在第二次（loop=1）时才发送motor_number个电机的消息。
    只在应答模式使用，发出一帧控制信息，才能得到一帧状态信息。一帧6个电机。
*/
    void motorState_();

    void sendToFD();
    void getFromFD(int channel);
    void motorState_fd();

    void get_pvt_kp(uint16_t id);
    void get_pvt_kd(uint16_t id);
    void get_motor_pvt_params(uint16_t id);
    void set_motor_pvt_params(void);
//API FOR SIM2REAL
//override ZQ's API

    int motor_init();

    MotorCmd get_motor_cmd(void);

    void get_motor_data(MotorState* state);

    void convert_motor_data();

    int set_motor_cmd(MotorCmd *cmd);

    int set_motor_enable_cmd(int enable, int index);
// #endif
}