#pragma once
#include <stdbool.h>  
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include "can_hw.hpp"
#include <boost/lockfree/spsc_queue.hpp>
#include <atomic>
#include <csignal>

#define ENCOS_CAN_INTERVAL_US 1000
#define LF1_LEG_L 0
#define LF1_LEG_R 1
#define LF1_WAIST 2
#define LF1_ARM_L 3
#define LF1_ARM_R 4

typedef struct {
    float position;
    float speed;
    float current;
} joint_data_t;

struct Motor_Msg_can
{
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
};

typedef struct
{
    uint8_t motor_num;
    uint8_t can_ide;
    struct Motor_Msg_can motor[6];

} CANFD_Msg;

extern CANFD_Msg Rx_Message_fd_[1];
// #define BYTE unsigned char
// #define UINT unsigned int
// typedef  struct  _VCI_CAN_OBJ{
// 	UINT	ID;
// 	UINT	TimeStamp;
// 	BYTE	TimeFlag;
// 	BYTE	SendType;
// 	BYTE	RemoteFlag;//是否是远程帧
// 	BYTE	ExternFlag;//是否是扩展帧
// 	BYTE	DataLen;
// 	BYTE	Data[8];
// 	BYTE	Reserved[3];
// }VCI_CAN_OBJ;

int encos_send_frames(uint32_t id, uint8_t *data, uint8_t len, uint8_t channel);

void encos_start_recv_thread();

void signal_handler(int signal);

#include <signal.h>

void signal_handler_sa(int signal); //避免多次ctrl+c时失效
