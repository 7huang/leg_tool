#pragma once
#include <stdint.h>

/*
 * PACE 采集用的逐关节收发时间戳(CLOCK_MONOTONIC, ns)。
 *   rx: motorState_fd 解包出一帧"应答帧1"(位置/速度/力矩)时记录,
 *       q/qd/tau 与 motor_state_output 同坐标系(电机角 - lf1_zero_offset_rad)。
 *   tx: Encos_CANFD_Command_Set 把该电机的报文 write() 进 socket 时记录。
 * rx_count / tx_count 每帧 +1, 用于区分"新反馈"与"重复读到的旧反馈"。
 */
typedef struct {
    float q;
    float qd;
    float tau;
    uint8_t temperature;
    uint8_t error;
    int64_t rx_ns;
    uint32_t rx_count;
    int64_t tx_ns;
    uint32_t tx_count;
} PaceJointSample;

#ifdef __cplusplus
extern "C" {
#endif

int64_t pace_now_ns(void);

void pace_record_rx(int index, float q, float qd, float tau, uint8_t temperature, uint8_t error);

void pace_record_tx(uint32_t motor_id);

/* 返回 0 成功, -1 表示 index 无效 */
int pace_get_joint_sample(int index, PaceJointSample *out);

#ifdef __cplusplus
}
#endif
