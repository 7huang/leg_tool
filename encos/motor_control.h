/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <inttypes.h>
#include <string.h>
#include "config.h"
#include "math_ops.h"

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

/* 电机参数配置指令(写)配置代码: 参考 ENCOS 技术文档 V1.14 第 9.2 节 */
#define param_set_acc     0x01  /* 系统加速度 */
#define param_set_kt      0x04  /* 扭矩系数 */
#define param_set_kp      0x05  /* 力位混控 KP 范围 */
#define param_set_kd      0x06  /* 力位混控 KD 范围 */
#define param_set_pos     0x07  /* 力位混控位置范围 */
#define param_set_spd     0x08  /* 力位混控速度范围 */
#define param_set_tor     0x09  /* 力位混控扭矩范围 */
#define param_set_cur     0x0a  /* 力位混控电流范围 */
#define param_set_timeout 0x0b  /* CAN 超时时间 */
#define param_set_cur_pi  0x0c  /* 电流环 PI */
#define param_set_spd_pi  0x0d  /* 速度环 PI */
#define param_set_pos_pd  0x0e  /* 位置环 PD */

#define comm_ack 0x00
#define comm_auto 0x01

typedef struct
{
    uint16_t motor_id;
    uint8_t INS_code;  // instruction code.
    uint8_t motor_fbd; // motor CAN communication feedback.
} MotorCommFbd;

typedef struct
{
    uint16_t angle_actual_int;
    uint16_t angle_desired_int;
    int16_t speed_actual_int;
    int16_t speed_desired_int;
    int16_t current_actual_int;
    int16_t current_desired_int;
    float speed_actual_rad;
    float speed_desired_rad;
    float angle_actual_rad;
    float angle_desired_rad;
    uint16_t motor_id;
    uint8_t temperature;
    uint8_t error;
    float angle_actual_float;
    float speed_actual_float;
    float current_actual_float;
    float angle_desired_float;
    float speed_desired_float;
    float current_desired_float;
    float power;
    uint16_t acceleration;
    uint16_t linkage_KP;
    uint16_t speed_KI;
    uint16_t feedback_KP;
    uint16_t feedback_KD;
    float torque_float;
    uint16_t pvt_kp;
    uint16_t pvt_kd;
} OD_Motor_Msg;

typedef struct{
    float torque_min;
    float torque_max;
    float current_min;
    float current_max;
    float k_d_min;
    float k_d_max;
    float k_t;
} MotorParaLimits;

/*
 * 电机力位混控(PVT)模式参数,由"电机控制参数查询指令"(查询代码 23~34)返回。
 * 比例关系参考 ENCOS 技术文档 9.3 节:
 *   pvt_kp/pvt_kd        : 比例 1
 *   pos/spd               : 原始 int16 比例 100 (rad / rad/s)
 *   tor/cur               : 原始 int16 比例 10  (Nm / A)
 *   cur_loop/spd_loop/pos_loop : 原始值比例 10000/100000 等
 */
typedef struct{
    uint16_t motor_id;
    uint16_t pvt_kp_min;      // 查询代码 23:力位混控 KP MIN
    uint16_t pvt_kp_max;      // 查询代码 23:力位混控 KP MAX
    uint16_t pvt_kd_min;      // 查询代码 24:力位混控 KD MIN
    uint16_t pvt_kd_max;      // 查询代码 24:力位混控 KD MAX
    float pos_min;            // 查询代码 25:力位混控 POS MIN (rad)
    float pos_max;            // 查询代码 25:力位混控 POS MAX (rad)
    float spd_min;            // 查询代码 26:力位混控 SPD MIN (rad/s)
    float spd_max;            // 查询代码 26:力位混控 SPD MAX (rad/s)
    float tor_min;            // 查询代码 27:力位混控 TOR MIN (Nm)
    float tor_max;            // 查询代码 27:力位混控 TOR MAX (Nm)
    float cur_min;            // 查询代码 28:力位混控 CUR MIN (A)
    float cur_max;            // 查询代码 28:力位混控 CUR MAX (A)
    uint16_t can_timeout_ms;  // 查询代码 31:CAN 超时保护时间 (ms)
    float cur_loop_kp;        // 查询代码 32:电流环 KP
    float cur_loop_ki;        // 查询代码 32:电流环 KI
    float spd_loop_kp;        // 查询代码 33:速度环 KP
    float spd_loop_ki;        // 查询代码 33:速度环 KI
    float pos_loop_kp;        // 查询代码 34:位置环 KP
    float pos_loop_kd;        // 查询代码 34:位置环 KD
} MotorPVTParams;

typedef struct{
    int motor_id;
    float real_pos_min;
    float real_pos_max;
    float real_torque_min;
    float real_torque_max;
    MotorParaLimits motor_para;
} MotorRealLimits;

typedef struct{
    MotorRealLimits motor_real_limits;
} Limits12Motors;

extern OD_Motor_Msg rv_motor_msg[7];
extern uint16_t motor_id_check;
extern MotorPVTParams motor_pvt_params;

void MotorIDReset(EtherCAT_Msg *TxMessage);
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new);
void MotorIDSettingMulti(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t motor_id_new);
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd);
void MotorSetting_multi(EtherCAT_Msg *TxMessage, uint16_t passage, uint16_t motor_id, uint8_t cmd);
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id);
void MotorCommModeReading_multi(EtherCAT_Msg *TxMessage, uint16_t passage, uint16_t motor_id);
void MotorIDReading(EtherCAT_Msg *TxMessage);

void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float cur);
void send_motor_ctrl_cmd_multi(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float cur);
void set_motor_position(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
void set_motor_speed(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
void set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
void set_motor_acceleration(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t acc, uint8_t ack_status);
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
void set_motor_feedbackKP(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t fdbKP, uint8_t ack_status);
void get_motor_parameter(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint8_t param_cmd);
void set_motor_parameter(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint8_t cfg_code, uint8_t ack_status, uint16_t val1, uint16_t val2, uint8_t dlc);

uint8_t RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id);
uint8_t RV_can_data_repack_multi(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id);
void Rv_Message_Print(uint8_t ack_status);

int float_to_uint(float x, float x_min, float x_max, int bits);

//lf1 fd
void send_motor_ctrl_cmd_fd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
uint8_t RV_fd_data_repack_multi(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t channel);

//电机错误帧解析:每个回传帧的Byte0[0:4]为电机错误信息(ENCOS技术文档错误信息表),
//error_code非0即为错误帧,按id做边沿检测,每个错误出现时打印一次 id + 错误类型
const char *RV_motor_error_str(uint8_t err_code);
void RV_motor_error_report(uint16_t motor_id, uint8_t err_code);

//C侧独立电机日志(与python解耦): motor_init()拿到CAN锁后调用 mc_log_open_default()
//创建 ./log/log_motor_control_*.log; mc_log() 写入带毫秒时间戳的一条记录;
//未打开时 mc_log 静默。声明带 extern "C", 供 C++(ec_api_without_ros.cpp)调用。
#ifdef __cplusplus
extern "C" {
#endif
void mc_log_open(const char *path);
void mc_log_open_default(void);
void mc_log_close(void);
void mc_log(const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#endif
