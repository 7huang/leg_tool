#include "queue.h"
#include "command.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include "time.h"
#include <iostream>
#include <chrono>
#include "transmit_fd.h"
#include "pace_stamp.h"
#include <vector>
#include <algorithm>

extern "C" {
#include "motor_control.h"
}
int tiktak = 0;
std::vector<int> group_0 = {1,5,6,7,11,12,16,17,23,24,41,42,43,2047};
std::vector<int> group_1 = {2,5,6,8,11,12,18,19,25,26,41,42,43,2047};
std::vector<int> group_2 = {3,5,6,9,11,12,20,27,41,42,43,2047};
std::vector<int> group_3 = {4,5,6,10,11,12,21,22,28,29,41,42,43,2047};

spsc_queue<EtherCAT_Msg_ptr, capacity<1>> messages_fd_tx[CHANNEL_NUMBER];
spsc_queue<EtherCAT_Msg_ptr, capacity<2>> messages_fd_rx[CHANNEL_NUMBER];

EtherCAT_Msg Rx_Message_fd[CHANNEL_NUMBER]; //only [0] is used
EtherCAT_Msg Tx_Message_fd[CHANNEL_NUMBER]; 

std::atomic<bool> running{false};

void Encos_CANFD_Data_Get()
{
    for (int channel = 0; channel < 6; ++channel)
        {
            Rx_Message_fd[0].motor[channel].id = Rx_Message_fd_[0].motor[channel].id;
            Rx_Message_fd[0].motor[channel].dlc = Rx_Message_fd_[0].motor[channel].dlc;
            memcpy(Rx_Message_fd[0].motor[channel].data, Rx_Message_fd_[0].motor[channel].data, sizeof(Rx_Message_fd_[0].motor[channel].data));
                // printf("idx = %d", Rx_Message_fd[0].motor[i].id);
            EtherCAT_Msg_ptr can_msg_rx = std::make_shared<EtherCAT_Msg>(Rx_Message_fd[0]);
            sendToQueue_fd_rx(0, can_msg_rx);
        }
}

// 打包线程(sendMotorCmd_)与本线程各自以约 1kHz 自由运行, 中间只隔一个容量为 1 的队列,
// 两者相位漂移时本线程会取到空队列。tiktak 照常前进, 轮到的那组电机这一拍就没有报文,
// 而且可能连续多轮都落在同一组上(实测单个电机最长约 50ms 收不到指令)。
// 队列里的报文是整机指令快照, 队列为空时重发上一份快照与本该发送的内容只差 1~2ms;
// 只重发不超过 kReuseMaxAgeNs 的快照, 打包线程真正停止时仍会停发, 保留电机的 CAN 超时保护。
static const int64_t kReuseMaxAgeNs = 3000000;
static EtherCAT_Msg_ptr last_msg_tx[CHANNEL_NUMBER];
static int64_t last_msg_tx_ns[CHANNEL_NUMBER] = {0};

void Encos_CANFD_Command_Set()
{
    int channel_data[6] = {0,0,0,0,0,0};
    EtherCAT_Msg_ptr msg_tx[6] = {};
    const int64_t now_ns = pace_now_ns();

    // 寻找数据的TX总线
    for (int channel = 0; channel < 6; ++channel)
    {
        // 检测无锁队列是否为空
        if (messages_fd_tx[channel].empty()) {
            const bool reuse = last_msg_tx[channel] && now_ns - last_msg_tx_ns[channel] <= kReuseMaxAgeNs;
            pace_count_tx_queue_empty(channel, reuse);
            if (reuse){
                channel_data[channel] = 1;
                msg_tx[channel] = last_msg_tx[channel];
            }
        }
        else{
            channel_data[channel] = 1;
            msg_tx[channel] = messages_fd_tx[channel].front();
            messages_fd_tx[channel].pop();
            last_msg_tx[channel] = msg_tx[channel];
            last_msg_tx_ns[channel] = now_ns;
        }
    }
    // 根据顺序向TX总线发送
    for (int passage = 0; passage < 7; ++passage)
    {
        for (int channel = 0; channel < 6; ++channel)
        {
            if (channel_data[channel] == 1)
            {
                EtherCAT_Msg &can_msg_tx = *msg_tx[channel];
                if (tiktak == 0 && std::find(group_0.begin(), group_0.end(), can_msg_tx.motor[passage].id) != group_0.end()){
                    encos_send_frames(can_msg_tx.motor[passage].id, can_msg_tx.motor[passage].data, can_msg_tx.motor[passage].dlc, channel);
                    pace_record_tx(can_msg_tx.motor[passage].id);
                }
                else if (tiktak == 1 && std::find(group_1.begin(), group_1.end(), can_msg_tx.motor[passage].id) != group_1.end()){
                    encos_send_frames(can_msg_tx.motor[passage].id, can_msg_tx.motor[passage].data, can_msg_tx.motor[passage].dlc, channel);
                    pace_record_tx(can_msg_tx.motor[passage].id);
                }
                else if (tiktak == 2 && std::find(group_2.begin(), group_2.end(), can_msg_tx.motor[passage].id) != group_2.end()){
                    encos_send_frames(can_msg_tx.motor[passage].id, can_msg_tx.motor[passage].data, can_msg_tx.motor[passage].dlc, channel);
                    pace_record_tx(can_msg_tx.motor[passage].id);
                }
                else if (tiktak == 3 && std::find(group_3.begin(), group_3.end(), can_msg_tx.motor[passage].id) != group_3.end()){
                    encos_send_frames(can_msg_tx.motor[passage].id, can_msg_tx.motor[passage].data, can_msg_tx.motor[passage].dlc, channel);
                    pace_record_tx(can_msg_tx.motor[passage].id);
                }
            }
        }
    }
    tiktak += 1;
    if (tiktak >= 4){
        tiktak = 0;
    }
}

/**
 * @description: customized Encos_runImpl() for encos
 * @return {*}
 * @author: yq qiao
 */

void Encos_CANFD_tx() {
    while (running)
    {
        // Encos_EtherCAT_Run();

        struct timespec start, end;
        struct timespec remaining;
        clock_gettime(CLOCK_MONOTONIC, &start);

        Encos_CANFD_Command_Set();
        
        clock_gettime(CLOCK_MONOTONIC, &end);
        long elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000L 
                        + (end.tv_nsec - start.tv_nsec);
        const long TARGET_NS = 1 * 1000000L; // 1ms = 1,000,000 ns
        if (elapsed_ns < TARGET_NS) {
            long sleep_ns = TARGET_NS - elapsed_ns;
            remaining.tv_sec = sleep_ns / 1000000000L;
            remaining.tv_nsec = sleep_ns % 1000000000L;
            nanosleep(&remaining, nullptr);
        }
    }
}

void Encos_CANFD_rx() {
    while (running)
    {

        Encos_CANFD_Data_Get();
        
        usleep(330);//

    }
}

/**
 * @description: customized startRun() for encos
 * @return {*}
 * @author: yq qiao
 */

std::thread runThread_fd_tx;
std::thread runThread_fd_rx;

void Encos_CANFD_startRun() {
    running = true;
    runThread_fd_tx = std::thread(Encos_CANFD_tx);
    //runThread_fd_rx = std::thread(Encos_CANFD_rx);
}

