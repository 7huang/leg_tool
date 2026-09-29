#include "motor_control.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdlib.h>

/* ── 电机控制独立日志(C 侧自管理, 与 python 解耦) ─────────────────────
 * 生命周期: motor_init() 拿到 CAN 锁后调用 mc_log_open_default() 创建
 *   ./log/log_motor_control_YYYYMMDD_HHMMSS.log(与 python 主日志同目录、
 *   同 log_ 前缀, 因此会被 python Logger 的 log_*.log 轮转一并清理)。
 * 打开失败时 mc_log() 静默跳过, 不影响任何控制逻辑。
 * 线程安全: RX 线程(电机报错)与 TX 1kHz 线程(力矩限幅)都会调用,
 *   整行先拼进缓冲区再单次 fprintf 写出(stdio 内部锁保证行不交错)。 */
static FILE *mc_log_fp = NULL;

/* 自定义路径打开(可选, 一般用默认的 mc_log_open_default) */
void mc_log_open(const char *path)
{
    if (mc_log_fp != NULL){
        fclose(mc_log_fp);
    }
    mc_log_fp = fopen(path, "a");
    if (mc_log_fp != NULL){
        setvbuf(mc_log_fp, NULL, _IOLBF, 0);
    }
}

/* 滚动清理: 保留最新的 MC_LOG_KEEP 个 log_motor_control_*.log。
 * 由 mc_log_open_default() 在创建新日志前调用(motor_init 上下文, 单线程, 安全)。
 * 文件名含定长时间戳, 字典序 == 时间序, 用 alphasort 排序后删最旧即可, 无需 stat。 */
#define MC_LOG_KEEP 10

static int mc_log_filter(const struct dirent *d)
{
    const char *n = d->d_name;
    size_t len = strlen(n);
    return len > sizeof("log_motor_control_") - 1 &&
           strncmp(n, "log_motor_control_", sizeof("log_motor_control_") - 1) == 0 &&
           len > 4 && strcmp(n + len - 4, ".log") == 0;
}

static void mc_log_rotate(void)
{
    struct dirent **list;
    int n = scandir("./log", &list, mc_log_filter, alphasort);
    if (n <= 0)
        return;
    /* 删最旧的, 使新建后总数不超过 MC_LOG_KEEP */
    int del = n - (MC_LOG_KEEP - 1);
    for (int k = 0; k < del; ++k){
        char path[192];
        snprintf(path, sizeof(path), "./log/%s", list[k]->d_name);
        remove(path);
    }
    for (int k = 0; k < n; ++k){
        free(list[k]);
    }
    free(list);
}

/* 默认路径: 当前工作目录 ./log/log_motor_control_年月日_时分秒.log */
void mc_log_open_default(void)
{
    mkdir("./log", 0777); /* 已存在时报错, 忽略即可 */
    //mc_log_rotate();     /* 先删最旧日志, 再创建新的[推理层会删除] */
    char path[96];
    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(path, sizeof(path), "./log/log_motor_control_%Y%m%d_%H%M%S.log", &tmv);
    mc_log_open(path);
}

void mc_log_close(void)
{
    if (mc_log_fp != NULL){
        fclose(mc_log_fp);
        mc_log_fp = NULL;
    }
}

void mc_log(const char *fmt, ...)
{
    if (mc_log_fp == NULL)
        return;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tmv;
    localtime_r(&ts.tv_sec, &tmv);
    char line[512];
    int n = snprintf(line, sizeof(line), "[%04d-%02d-%02d %02d:%02d:%02d.%03ld] ",
                     tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                     tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ts.tv_nsec / 1000000L);
    if (n < 0 || (size_t)n >= sizeof(line))
        return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + n, sizeof(line) - n, fmt, ap);
    va_end(ap);
    fprintf(mc_log_fp, "%s\n", line); /* 单次写出, 保证整行原子性 */
    fflush(mc_log_fp);                /* 低频日志, 立即落盘, 崩溃不丢 */
}

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

int motor_id_safe_check[29];
float motor_pos_safe_check[29];
float motor_set_zero_position[29]; //TODO：标零数据回传
MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[7];
MotorPVTParams motor_pvt_params;
const MotorParaLimits A10020P2 = {-300.0f, 300.0f, -140.0f, 140.0f, 0.0f, 50.0f, 2.6f};
const MotorParaLimits A10020P1 = {-150.0f, 150.0f, -70.0f, 70.0f, 0.0f, 50.0f, 2.5f};
const MotorParaLimits A8112P1 = {-90.0f, 90.0f, -60.0f, 60.0f, 0.0f, 50.0f, 2.1f};
const MotorParaLimits A6408P2 = {-60.0f, 60.0f, -60.0f, 60.0f, 0.0f, 50.0f, 2.35f};
//lf1
const MotorParaLimits A6416H = {-120.0f, 120.0f, -60.0f, 60.0f, 0.0f, 50.0f, 2.65f};
const MotorParaLimits A6408H16 = {-45.0f, 45.0f, -30.0f, 30.0f, 0.0f, 50.0f, 2.15f};
const MotorParaLimits A6408H30 = {-60.0f, 60.0f, -60.0f, 60.0f, 0.0f, 50.0f, 2.45f};
const MotorParaLimits A4315 = {-70.0f, 70.0f, -30.0f, 30.0f, 0.0f, 50.0f, 2.8f};
const MotorParaLimits A4310H = {-30.0f, 30.0f, -30.0f, 30.0f, 0.0f, 50.0f, 1.4f};
const MotorParaLimits A2806 = {-12.0f, 12.0f, -10.0f, 10.0f, 0.0f, 50.0f, 1.35f};

int hip_motor_type = 2;

/*
 * 电机错误信息表(参考 ENCOS 技术文档 V1.14 第10章"问答模式反馈报文"):
 * 每个回传/应答帧的 Byte0[0:4](uint5) 为电机错误信息,错误码含义如下。
 */
const char *RV_motor_error_str(uint8_t err_code){
    switch (err_code){
        case 0: return "无错误";
        case 1: return "电机过热";
        case 2: return "电机过流";
        case 3: return "电机电压过高";
        case 4: return "电机电压过低";
        case 5: return "电机编码器错误";
        case 6: return "电机刹车电压过高";
        case 7: return "DRV驱动错误";
        default: return "未知错误";
    }
}

// 电机错误帧解析。问答模式下每个回传帧都携带错误信息(错误码非0即为错误帧),
// 电机报错期间每帧都带同一错误码(约1kHz)。为避免刷屏,按电机CAN id做边沿检测:
// 仅在错误码从0变为非0、或错误码发生变化时打印一次 id + 错误类型。
static uint8_t last_motor_err_code[256]; // 按CAN id记录上次打印的错误码

void RV_motor_error_report(uint16_t motor_id, uint8_t err_code){
    if (motor_id == 0 || motor_id > 255)
        return;
    if (err_code == 0){
        last_motor_err_code[motor_id] = 0; // 错误清除,复位记录,下次再报错可再次打印
        return;
    }
    if (last_motor_err_code[motor_id] == err_code)
        return; // 同一错误持续上报,不重复打印
    last_motor_err_code[motor_id] = err_code;
    printf("[MOTOR-ERROR] motor_id = %d, error_code = %d, error_type = %s\n",
           motor_id, err_code, RV_motor_error_str(err_code));
    mc_log("[MOTOR-ERROR] motor_id = %d, error_code = %d, error_type = %s",
           motor_id, err_code, RV_motor_error_str(err_code));
}

// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd)
{
    // EtherCAT_Msg TxMessage;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].dlc = 4;

    if (cmd == 0)
        return;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = cmd;
}

void MotorSetting_multi(EtherCAT_Msg *TxMessage, uint16_t passage, uint16_t motor_id, uint8_t cmd)
{
    // EtherCAT_Msg TxMessage;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].id = 0x7FF;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].dlc = 4;

    if (cmd == 0)
        return;

    TxMessage->motor[passage-1].data[0] = motor_id >> 8;
    TxMessage->motor[passage-1].data[1] = motor_id & 0xff;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = cmd;
}

// Reset Motor ID
void MotorIDReset(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = 0x7F;
    TxMessage->motor[0].data[1] = 0x7F;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x05;
    TxMessage->motor[0].data[4] = 0x7F;
    TxMessage->motor[0].data[5] = 0x7F;
}
// set motor new ID
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xff;
}
// set motors new IDs
void MotorIDSettingMulti(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t motor_id_new)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].id = 0x7FF;
    TxMessage->motor[passage - 1].dlc = 6;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].data[0] = motor_id >> 8;
    TxMessage->motor[passage - 1].data[1] = motor_id & 0xff;
    TxMessage->motor[passage - 1].data[2] = 0x00;
    TxMessage->motor[passage - 1].data[3] = 0x04;
    TxMessage->motor[passage - 1].data[4] = motor_id_new >> 8;
    TxMessage->motor[passage - 1].data[5] = motor_id_new & 0xff;
}
// read motor communication mode
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x81;
}
// read multiple motor communication mode
void MotorCommModeReading_multi(EtherCAT_Msg *TxMessage, uint16_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = 0x7FF;
    TxMessage->motor[passage-1].dlc = 4;

    TxMessage->motor[passage-1].data[0] = motor_id >> 8;
    TxMessage->motor[passage-1].data[1] = motor_id & 0xff;
    TxMessage->motor[passage-1].data[2] = 0x00;
    TxMessage->motor[passage-1].data[3] = 0x81;
}

// read motor ID
void MotorIDReading(EtherCAT_Msg *TxMessage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (tor > T_MAX)
        tor = T_MAX;
    else if (tor < T_MIN)
        tor = T_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    TxMessage->motor[passage-1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage-1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage-1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage-1].data[3] = pos_int >> 8;
    TxMessage->motor[passage-1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage-1].data[5] = spd_int >> 4;
    TxMessage->motor[passage-1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage-1].data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
passage:1-6
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd_multi(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    switch(motor_id){
        // TODO：确认电机的ID号
        // 根据ID号判断电机的类型
        // 电机之间不同的限制
        case 3:
        case 4:
        case 9:
        case 10:
            if (tor > A10020P2.torque_max)
                tor = A10020P2.torque_max;
            else if (tor < A10020P2.torque_min)
                tor = A10020P2.torque_min;
            if (kd > A10020P2.k_d_max)
                kd = A10020P2.k_d_max;
            else if (kd < A10020P2.k_d_min)
                kd = A10020P2.k_d_min;
            
            kd_int = float_to_uint(kd, A10020P2.k_d_min, A10020P2.k_d_max, 9);
            tor_int = float_to_uint(tor, A10020P2.torque_min, A10020P2.torque_max, 12);
            break;
        case 1:
        case 7:
            if (hip_motor_type == 1){
                if (tor > A10020P1.torque_max)
                tor = A10020P1.torque_max;
                else if (tor < A10020P1.torque_min)
                    tor = A10020P1.torque_min;
                if (kd > A10020P1.k_d_max)
                    kd = A10020P1.k_d_max;
                else if (kd < A10020P1.k_d_min)
                    kd = A10020P1.k_d_min;
                
                kd_int = float_to_uint(kd, A10020P1.k_d_min, A10020P1.k_d_max, 9);
                tor_int = float_to_uint(tor, A10020P1.torque_min, A10020P1.torque_max, 12);
            }
            if (hip_motor_type == 2){
                if (tor > A10020P2.torque_max)
                tor = A10020P2.torque_max;
                else if (tor < A10020P2.torque_min)
                    tor = A10020P2.torque_min;
                if (kd > A10020P2.k_d_max)
                    kd = A10020P2.k_d_max;
                else if (kd < A10020P2.k_d_min)
                    kd = A10020P2.k_d_min;
                
                kd_int = float_to_uint(kd, A10020P2.k_d_min, A10020P2.k_d_max, 9);
                tor_int = float_to_uint(tor, A10020P2.torque_min, A10020P2.torque_max, 12);
            }
                
            break;
        case 2:
        case 8:
            if (tor > A8112P1.torque_max)
                tor = A8112P1.torque_max;
            else if (tor < A8112P1.torque_min)
                tor = A8112P1.torque_min;
            if (kd > A8112P1.k_d_max)
                kd = A8112P1.k_d_max;
            else if (kd < A8112P1.k_d_min)
                kd = A8112P1.k_d_min;
            
            kd_int = float_to_uint(kd, A8112P1.k_d_min, A8112P1.k_d_max, 9);
            tor_int = float_to_uint(tor, A8112P1.torque_min, A8112P1.torque_max, 12);
            break;
        case 5:
        case 6:
        case 11:
        case 12:
            // printf("A6408P2 check point 0\n");
            if (tor > A6408P2.torque_max)
                tor = A6408P2.torque_max;
            else if (tor < A6408P2.torque_min)
                tor = A6408P2.torque_min;
            if (kd > A6408P2.k_d_max)
                kd = A6408P2.k_d_max;
            else if (kd < A6408P2.k_d_min)
                kd = A6408P2.k_d_min;
            
            kd_int = float_to_uint(kd, A6408P2.k_d_min, A6408P2.k_d_max, 9);
            tor_int = float_to_uint(tor, A6408P2.torque_min, A6408P2.torque_max, 12);
            break;
    }

    // 电机共有的限制
    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    

    TxMessage->motor[passage-1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage-1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage-1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage-1].data[3] = pos_int >> 8;
    TxMessage->motor[passage-1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage-1].data[5] = spd_int >> 4;
    TxMessage->motor[passage-1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage-1].data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_position(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    TxMessage->motor[passage-1].data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    TxMessage->motor[passage-1].data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    TxMessage->motor[passage-1].data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    TxMessage->motor[passage-1].data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    TxMessage->motor[passage-1].data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    TxMessage->motor[passage-1].data[5] = (spd & 0x3FC) >> 2;
    TxMessage->motor[passage-1].data[6] = (spd & 0x03) << 6 | (cur >> 6);
    TxMessage->motor[passage-1].data[7] = (cur & 0x3F) << 2 | ack_status;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_speed(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 7;

    rv_type_convert.to_float = spd;
    TxMessage->motor[passage-1].data[0] = 0x40 | ack_status;
    TxMessage->motor[passage-1].data[1] = rv_type_convert.buf[3];
    TxMessage->motor[passage-1].data[2] = rv_type_convert.buf[2];
    TxMessage->motor[passage-1].data[3] = rv_type_convert.buf[1];
    TxMessage->motor[passage-1].data[4] = rv_type_convert.buf[0];
    TxMessage->motor[passage-1].data[5] = cur >> 8;
    TxMessage->motor[passage-1].data[6] = cur & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
cur:-3000~3000
ctrl_status:
    0:current control
    1:torque control
    2:variable damping brake control(also call full brake)
    3:dynamic brake control
    4:regenerative brake control
    5:NON
    6:NON
    7:NON
ack_status:0~3
*/
void set_motor_cur_tor(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 3;

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    TxMessage->motor[passage-1].data[0] = 0x60 | ctrl_status << 2 | ack_status;
    TxMessage->motor[passage-1].data[1] = cur_tor >> 8;
    TxMessage->motor[passage-1].data[2] = cur_tor & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void set_motor_acceleration(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t acc, uint8_t ack_status)
{


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 4;

    if (ack_status > 2)
        return;
    if (acc > 2000)
        acc = 2000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x01;
    TxMessage->motor[passage-1].data[2] = acc >> 8;
    TxMessage->motor[passage-1].data[3] = acc & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 6;

    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x02;
    TxMessage->motor[passage-1].data[2] = linkage >> 8;
    TxMessage->motor[passage-1].data[3] = linkage & 0xff;
    TxMessage->motor[passage-1].data[4] = speedKI >> 8;
    TxMessage->motor[passage-1].data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void set_motor_feedbackKP_KD(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status)
{

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 6;

    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    TxMessage->motor[passage-1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage-1].data[1] = 0x03;
    TxMessage->motor[passage-1].data[2] = fdbKP >> 8;
    TxMessage->motor[passage-1].data[3] = fdbKP & 0xff;
    TxMessage->motor[passage-1].data[4] = fdbKD >> 8;
    TxMessage->motor[passage-1].data[5] = fdbKD & 0xff;
}
// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void get_motor_parameter(EtherCAT_Msg *TxMessage,uint8_t passage, uint16_t motor_id, uint8_t param_cmd)
{


    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 2;

    TxMessage->motor[passage-1].data[0] = 0xE0;
    TxMessage->motor[passage-1].data[1] = param_cmd;
}

/*
 * 电机参数配置指令(写): 参考 ENCOS 技术文档 V1.14 第 9.2 节。
 * 帧格式: Byte0 = 0xC0 | ack_status(即 0x06<<5 | 返回状态), Byte1 = 配置代码,
 *         Byte2~5 = 配置数值(高位在前/大端)。
 * cfg_code : 见 param_set_* 宏(0x01加速度/0x04扭矩系数/0x05 KP/0x06 KD/0x07 POS/0x08 SPD/
 *            0x09 TOR/0x0a CUR/0x0b CAN超时/0x0c电流环PI/0x0d速度环PI/0x0e位置环PD)
 * ack_status: 0不返回 1返回报文类型4(配置状态) 2/3不返回且设置失败
 * val1/val2 : 已按协议比例转换后的原始数值(uint16 位模式, 有符号参数传入其补码)
 * dlc       : 4(单数值) 或 6(双数值)
 */
void set_motor_parameter(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id,
                         uint8_t cfg_code, uint8_t ack_status,
                         uint16_t val1, uint16_t val2, uint8_t dlc)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = dlc;

    TxMessage->motor[passage-1].data[0] = 0xC0 | (ack_status & 0x03); // 电机模式0x06<<5 | 返回状态
    TxMessage->motor[passage-1].data[1] = cfg_code;
    TxMessage->motor[passage-1].data[2] = (uint8_t)((val1 >> 8) & 0xFF);
    TxMessage->motor[passage-1].data[3] = (uint8_t)(val1 & 0xFF);
    if (dlc >= 6) {
        TxMessage->motor[passage-1].data[4] = (uint8_t)((val2 >> 8) & 0xFF);
        TxMessage->motor[passage-1].data[5] = (uint8_t)(val2 & 0xFF);
    }
}

void Rv_Message_Print(uint8_t ack_status)
{
    if (ack_status >= 100)
    {
        printf("motor%d message:\n", ack_status - 100);
        if (motor_comm_fbd.motor_fbd == 0x01)
        {
            printf("自动模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x02)
        {
            printf("问答模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x03)
        {
            printf("零点设置成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x04)
        {
            printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x05)
        {
            printf("重置Id成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x06)
        {
            printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x80)
        {
            printf("查询失败.\n");
        }
    }
    else
    {
        for (int i = 0; i < 6; ++i)
        {
            if (rv_motor_msg[i].motor_id == 0)
            {
                continue;
            }
            switch (ack_status)
            {
            case 1:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
                printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                break;
            case 2:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                break;
            case 3:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                break;
            case 4:
                if (motor_comm_fbd.motor_fbd == 0)
                {
                    printf("配置成功.\n");
                }
                else
                {
                    printf("配置失败.\n");
                }
                break;
            case 5:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                if (motor_comm_fbd.INS_code == 1)
                {
                    printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 2)
                {
                    printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 3)
                {
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 4)
                {
                    printf("power: %f\n", rv_motor_msg[i].power);
                }
                else if (motor_comm_fbd.INS_code == 5)
                {
                    printf("acceleration: %d\n", rv_motor_msg[i].acceleration);
                }
                else if (motor_comm_fbd.INS_code == 6)
                {
                    printf("linkage_KP: %d\n", rv_motor_msg[i].linkage_KP);
                }
                else if (motor_comm_fbd.INS_code == 7)
                {
                    printf("speed_KI: %d\n", rv_motor_msg[i].speed_KI);
                }
                else if (motor_comm_fbd.INS_code == 8)
                {
                    printf("feedback_KP: %d\n", rv_motor_msg[i].feedback_KP);
                }
                else if (motor_comm_fbd.INS_code == 9)
                {
                    printf("feedback_KD: %d\n", rv_motor_msg[i].feedback_KD);
                }
                break;
            case 6:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
                printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                break;
            default:
                break;
            }
        }
    }
}

uint16_t motor_id_check = 0;
uint8_t RV_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    for (int i = 0; i < 6; ++i)
    {
        if (RxMessage->motor[i].dlc == 0)
            continue;
        if (RxMessage->motor[i].id == 0x7FF)
        {
            if (RxMessage->motor[i].data[2] != 0x01) // determine whether it is a motor feedback instruction
                return 255;                              // it is not a motor feedback instruction
            if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) // inquire failed
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) // reset ID succeed
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            return 100 + i;
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // Response mode
        {
            // printf("id = %d\n"i);
            ack_status = RxMessage->motor[i].data[0] >> 5;
            motor_id_t = RxMessage->motor[i].id - 1;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].motor_id = motor_id_check;
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[0] & 0x1F;
            if (ack_status == 1) // response frame 1
            {
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];

                rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[6] - 50) / 2;
            }
            else if (ack_status == 2) // response frame 2
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) // response frame 3
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) // response frame 4
            {
                if (RxMessage->motor[i].dlc != 3)
                    return 255;
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (ack_status == 5) // response frame 5
            {
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) // get position
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) // get speed
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) // get current
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) // get power
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) // get acceleration
                {
                    rv_motor_msg[motor_id_t].acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) // get linkage_KP
                {
                    rv_motor_msg[motor_id_t].linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) // get speed_KI
                {
                    rv_motor_msg[motor_id_t].speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) // get feedback_KP
                {
                    rv_motor_msg[motor_id_t].feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) // get feedback_KD
                {
                    rv_motor_msg[motor_id_t].feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
            }

            return ack_status;
        }
        else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) // automatic feedback mode
        {
            motor_id_t = RxMessage->motor[i].id - 0x205;
            rv_motor_msg[motor_id_t].motor_id = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
            rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
            rv_motor_msg[motor_id_t].current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
            rv_motor_msg[motor_id_t].temperature = RxMessage->motor[i].data[6];
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[7];
            return 6;
        }
    }
}
// RV_can_data_repack() in multiple motor mode
uint8_t RV_can_data_repack_multi(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    for (int i = 0; i < 6; ++i)
    {
        if (RxMessage->motor[i].dlc == 0)
            continue;
        if (RxMessage->motor[i].id == 0x7FF)
        {
            if (RxMessage->motor[i].data[2] != 0x01) // determine whether it is a motor feedback instruction
                return 255;                              // it is not a motor feedback instruction
            if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) // inquire failed
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) // reset ID succeed
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            return 100 + i;
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // Response mode
        {
            // printf("id = %d\n"i);
            ack_status = RxMessage->motor[i].data[0] >> 5;
            motor_id_t = RxMessage->motor[i].id - 1;
            if (motor_id_t > 5 && motor_id_t < 9)
                motor_id_t -= 3;
            if (motor_id_t > 8 && motor_id_t < 12)
                motor_id_t -= 9;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].motor_id = motor_id_check;
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[0] & 0x1F;
            if (ack_status == 1) // response frame 1
            {
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];

                rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[6] - 50) / 2;
                switch (motor_id_check){
                    case 3:
                    case 4:
                    case 9:
                    case 10:
                        rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, A10020P2.current_min, A10020P2.current_max, 12);
                        rv_motor_msg[motor_id_t].torque_float = rv_motor_msg[motor_id_t].current_actual_float * A10020P2.k_t;
                        break;
                    case 1:
                    case 7:
                        if (hip_motor_type == 1){
                            rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, A10020P1.current_min, A10020P1.current_max, 12);
                            rv_motor_msg[motor_id_t].torque_float = rv_motor_msg[motor_id_t].current_actual_float * A10020P1.k_t;
                        }
                        if (hip_motor_type == 2){
                            rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, A10020P2.current_min, A10020P2.current_max, 12);
                            rv_motor_msg[motor_id_t].torque_float = rv_motor_msg[motor_id_t].current_actual_float * A10020P2.k_t;
                        }
                        
                        break;
                    case 2:
                    case 8:
                        rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, A8112P1.current_min, A8112P1.current_max, 12);
                        rv_motor_msg[motor_id_t].torque_float = rv_motor_msg[motor_id_t].current_actual_float * A8112P1.k_t;
                        break;
                    // case 1:
                    case 5:
                    case 6:
                    case 11:
                    case 12:
                        rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, A6408P2.current_min, A6408P2.current_max, 12);
                        rv_motor_msg[motor_id_t].torque_float = rv_motor_msg[motor_id_t].current_actual_float * A6408P2.k_t;
                        break;
                    default:
                        break;
                }
                // return ack_status; //注意：多电机不能return
                // printf("motor_id = %d \n",rv_motor_msg[motor_id_t].motor_id);
                
            }
            else if (ack_status == 2) // response frame 2
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) // response frame 3
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id_t].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) // response frame 4
            {
                if (RxMessage->motor[i].dlc != 3)
                    return 255;
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (ack_status == 5) // response frame 5
            {
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) // get position
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                    motor_id_safe_check[RxMessage->motor[i].id - 1] = RxMessage->motor[i].id; //用于ID检查
                    motor_pos_safe_check[RxMessage->motor[i].id - 1] = rv_type_convert.to_float;
                    // motor_set_zero_position[RxMessage->motor[i].id - 1] = rv_motor_msg[motor_id_t].angle_actual_float;
                    // printf("RxMessage->motor[i].id=%d \n",RxMessage->motor[i].id);
                    // printf("motor_set_zero_position=%f \n",motor_set_zero_position[RxMessage->motor[i].id - 1]);
                    // printf("angle_actual_float=%f \n",rv_motor_msg[motor_id_t].angle_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) // get speed
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) // get current
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) // get power
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) // get acceleration
                {
                    rv_motor_msg[motor_id_t].acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) // get linkage_KP
                {
                    rv_motor_msg[motor_id_t].linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) // get speed_KI
                {
                    rv_motor_msg[motor_id_t].speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) // get feedback_KP
                {
                    rv_motor_msg[motor_id_t].feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) // get feedback_KD
                {
                    rv_motor_msg[motor_id_t].feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
            }

            // return ack_status;
        }
        else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) // automatic feedback mode
        {
            motor_id_t = RxMessage->motor[i].id - 0x205;
            rv_motor_msg[motor_id_t].motor_id = RxMessage->motor[i].id;
            rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
            rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
            rv_motor_msg[motor_id_t].current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
            rv_motor_msg[motor_id_t].temperature = RxMessage->motor[i].data[6];
            rv_motor_msg[motor_id_t].error = RxMessage->motor[i].data[7];
            return 6;
        }
    }
    
    // printf("id = %d\n",rv_motor_msg[0].motor_id);
    // printf("id = %d\n",rv_motor_msg[1].motor_id);
    // printf("id = %d\n",rv_motor_msg[2].motor_id);
    // printf("id = %d\n",rv_motor_msg[3].motor_id);
    // printf("id = %d\n",rv_motor_msg[4].motor_id);
    // printf("id = %d\n",rv_motor_msg[5].motor_id);
}

void send_motor_ctrl_cmd_fd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

    switch(motor_id){
        // TODO：确认电机的ID号
        // 根据ID号判断电机的类型
        // 电机之间不同的限制
        case 1:
        case 2:
        case 4:
        case 7:
        case 8:
        case 10:
            if (tor > A6416H.torque_max)
                tor = A6416H.torque_max;
            else if (tor < A6416H.torque_min)
                tor = A6416H.torque_min;
            if (kd > A6416H.k_d_max)
                kd = A6416H.k_d_max;
            else if (kd < A6416H.k_d_min)
                kd = A6416H.k_d_min;
            
            kd_int = float_to_uint(kd, A6416H.k_d_min, A6416H.k_d_max, 9);
            tor_int = float_to_uint(tor, A6416H.torque_min, A6416H.torque_max, 12);
            break;
        case 3:
        case 9:
            if (tor > A6408H30.torque_max)
            tor = A6408H30.torque_max;
            else if (tor < A6408H30.torque_min)
                tor = A6408H30.torque_min;
            if (kd > A6408H30.k_d_max)
                kd = A6408H30.k_d_max;
            else if (kd < A6408H30.k_d_min)
                kd = A6408H30.k_d_min;
            
            kd_int = float_to_uint(kd, A6408H30.k_d_min, A6408H30.k_d_max, 9);
            tor_int = float_to_uint(tor, A6408H30.torque_min, A6408H30.torque_max, 12); 
            break;
        case 41:
                if (tor > A6408H16.torque_max)
                tor = A6408H16.torque_max;
                else if (tor < A6408H16.torque_min)
                    tor = A6408H16.torque_min;
                if (kd > A6408H16.k_d_max)
                    kd = A6408H16.k_d_max;
                else if (kd < A6408H16.k_d_min)
                    kd = A6408H16.k_d_min;
                
                kd_int = float_to_uint(kd, A6408H16.k_d_min, A6408H16.k_d_max, 9);
                tor_int = float_to_uint(tor, A6408H16.torque_min, A6408H16.torque_max, 12);            
            break;
        case 42:
        case 43:
            // printf("A6408P2 check point 0\n");
            if (tor > A4315.torque_max)
                tor = A4315.torque_max;
            else if (tor < A4315.torque_min)
                tor = A4315.torque_min;
            if (kd > A4315.k_d_max)
                kd = A4315.k_d_max;
            else if (kd < A4315.k_d_min)
                kd = A4315.k_d_min;
            
            kd_int = float_to_uint(kd, A4315.k_d_min, A4315.k_d_max, 9);
            tor_int = float_to_uint(tor, A4315.torque_min, A4315.torque_max, 12);
            break;
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 23:
        case 24:
        case 25:
        case 26:
        case 27:
        case 5:
        case 6:
	    case 11:
	    case 12:
            if (tor > A4310H.torque_max)
                tor = A4310H.torque_max;
            else if (tor < A4310H.torque_min)
                tor = A4310H.torque_min;
            if (kd > A4310H.k_d_max)
                kd = A4310H.k_d_max;
            else if (kd < A4310H.k_d_min)
                kd = A4310H.k_d_min;
            
            kd_int = float_to_uint(kd, A4310H.k_d_min, A4310H.k_d_max, 9);
            tor_int = float_to_uint(tor, A4310H.torque_min, A4310H.torque_max, 12);
            break;
        case 21:
        case 22:
        case 28:
        case 29:
            if (tor > A2806.torque_max)
                tor = A2806.torque_max;
            else if (tor < A2806.torque_min)
                tor = A2806.torque_min;
            if (kd > A2806.k_d_max)
                kd = A2806.k_d_max;
            else if (kd < A2806.k_d_min)
                kd = A2806.k_d_min;
            
            kd_int = float_to_uint(kd, A2806.k_d_min, A2806.k_d_max, 9);
            tor_int = float_to_uint(tor, A2806.torque_min, A2806.torque_max, 12);
            break;
        default:
            break;
    }

    // 电机共有的限制
    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);

    TxMessage->motor[passage-1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage-1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage-1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage-1].data[3] = pos_int >> 8;
    TxMessage->motor[passage-1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage-1].data[5] = spd_int >> 4;
    TxMessage->motor[passage-1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage-1].data[7] = tor_int & 0xff;
}

uint8_t RV_fd_data_repack_multi(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t channel)
{
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    // printf("REPACK\n");
    for (int i = 0; i < 7; ++i)
    {
        if (RxMessage->motor[i].dlc == 0)
            continue;
        if (RxMessage->motor[i].id == 0x7FF)
        {
            if (RxMessage->motor[i].data[2] != 0x01) // determine whether it is a motor feedback instruction
                return 255;                              // it is not a motor feedback instruction
            if ((RxMessage->motor[i].data[0] == 0xff) && (RxMessage->motor[i].data[1] == 0xFF))
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if ((RxMessage->motor[i].data[0] == 0x80) && (RxMessage->motor[i].data[1] == 0x80)) // inquire failed
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if ((RxMessage->motor[i].data[0] == 0x7F) && (RxMessage->motor[i].data[1] == 0x7F)) // reset ID succeed
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            return 100 + i;
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // Response mode
        {
            // printf("id = %d\n", RxMessage->motor[i].ids);
            ack_status = RxMessage->motor[i].data[0] >> 5;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[i].motor_id = motor_id_check;
            rv_motor_msg[i].error = RxMessage->motor[i].data[0] & 0x1F;
            RV_motor_error_report(RxMessage->motor[i].id, rv_motor_msg[i].error); // 错误帧解析:非0即打印id+错误类型
            if (ack_status == 1) // response frame 1
            {
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];

                rv_motor_msg[i].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[i].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[i].temperature = (RxMessage->motor[i].data[6] - 50) / 2;
                switch (motor_id_check){
                    case 1:
                    case 2:
                    case 4:
                    case 7:
                    case 8:
                    case 10:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A6416H.current_min, A6416H.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A6416H.k_t;
                        break;
                    case 3:
                    case 9:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A6408H30.current_min, A6408H30.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A6408H30.k_t;                     
                        break;
                    case 41:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A6408H16.current_min, A6408H16.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A6408H16.k_t;                     
                        break;
                    case 42:
                    case 43:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A4315.current_min, A4315.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A4315.k_t;
                        break;
                    case 16:
                    case 17:
                    case 18:
                    case 19:
                    case 20:
                    case 23:
                    case 24:
                    case 25:
                    case 26:
                    case 27:
                    case 5:
                    case 6:
	                case 11:
	                case 12:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A4310H.current_min, A4310H.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A4310H.k_t;
                        break;
                    case 21:
                    case 22:
                    case 28:
                    case 29:
                        rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, A2806.current_min, A2806.current_max, 12);
                        rv_motor_msg[i].torque_float = rv_motor_msg[i].current_actual_float * A2806.k_t;
                        break;
                    default:
                        break;
                }
                // return ack_status; //注意：多电机不能return
                // printf("motor_id = %d \n",rv_motor_msg[i].motor_id);
                
            }
            else if (ack_status == 2) // response frame 2
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[i].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[i].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[i].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[i].current_actual_float = rv_motor_msg[i].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) // response frame 3
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[i].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[i].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[i].temperature = (RxMessage->motor[i].data[7] - 50) / 2;
                rv_motor_msg[i].current_actual_float = rv_motor_msg[i].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) // response frame 4
            {
                if (RxMessage->motor[i].dlc != 3)
                    return 255;
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (ack_status == 5) // response frame 5
            {
                motor_comm_fbd.INS_code = RxMessage->motor[i].data[1];
                if (motor_comm_fbd.INS_code == 1 && RxMessage->motor[i].dlc == 6) // get position
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[i].angle_actual_float = rv_type_convert.to_float;
                    int index = fd_id_2_index(RxMessage->motor[i].id);
                    if (index == -1){
                        continue;
                    }
                    motor_id_safe_check[index] = RxMessage->motor[i].id; //用于ID检查
                    motor_pos_safe_check[index] = rv_type_convert.to_float;
                    // motor_set_zero_position[RxMessage->motor[i].id - 1] = rv_motor_msg[i].angle_actual_float;
                    // printf("RxMessage->motor[i].id=%d \n",RxMessage->motor[i].id);
                    // printf("motor_set_zero_position=%f \n",motor_set_zero_position[RxMessage->motor[i].id - 1]);
                    // printf("angle_actual_float=%f \n",rv_motor_msg[i].angle_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 2 && RxMessage->motor[i].dlc == 6) // get speed
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[i].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && RxMessage->motor[i].dlc == 6) // get current
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[i].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && RxMessage->motor[i].dlc == 6) // get power
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[i].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && RxMessage->motor[i].dlc == 4) // get acceleration
                {
                    rv_motor_msg[i].acceleration = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && RxMessage->motor[i].dlc == 4) // get linkage_KP
                {
                    rv_motor_msg[i].linkage_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && RxMessage->motor[i].dlc == 4) // get speed_KI
                {
                    rv_motor_msg[i].speed_KI = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && RxMessage->motor[i].dlc == 4) // get feedback_KP
                {
                    rv_motor_msg[i].feedback_KP = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && RxMessage->motor[i].dlc == 4) // get feedback_KD
                {
                    rv_motor_msg[i].feedback_KD = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                else if (motor_comm_fbd.INS_code == 23 && RxMessage->motor[i].dlc == 6) // get PVT KP
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    uint16_t kp_min = rv_type_convert2.to_uint16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    uint16_t kp_max = rv_type_convert2.to_uint16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.pvt_kp_min = kp_min;
                    motor_pvt_params.pvt_kp_max = kp_max;
                    printf("motor %d 's PVT kp min = %d, kp max = %d\n", RxMessage->motor[i].id, kp_min, kp_max);
                }
                else if (motor_comm_fbd.INS_code == 24 && RxMessage->motor[i].dlc == 6) // get PVT KD
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    uint16_t kd_min = rv_type_convert2.to_uint16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    uint16_t kd_max = rv_type_convert2.to_uint16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.pvt_kd_min = kd_min;
                    motor_pvt_params.pvt_kd_max = kd_max;
                    printf("motor %d 's PVT kd min = %d, kd max = %d\n", RxMessage->motor[i].id, kd_min, kd_max);
                }
                else if (motor_comm_fbd.INS_code == 25 && RxMessage->motor[i].dlc == 6) // get PVT POS range
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    int16_t pos_min_raw = rv_type_convert2.to_int16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    int16_t pos_max_raw = rv_type_convert2.to_int16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.pos_min = pos_min_raw / 100.0f;
                    motor_pvt_params.pos_max = pos_max_raw / 100.0f;
                    printf("motor %d 's PVT pos min = %.2f rad, pos max = %.2f rad\n",
                        RxMessage->motor[i].id, motor_pvt_params.pos_min, motor_pvt_params.pos_max);
                }
                else if (motor_comm_fbd.INS_code == 26 && RxMessage->motor[i].dlc == 6) // get PVT SPD range
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    int16_t spd_min_raw = rv_type_convert2.to_int16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    int16_t spd_max_raw = rv_type_convert2.to_int16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.spd_min = spd_min_raw / 100.0f;
                    motor_pvt_params.spd_max = spd_max_raw / 100.0f;
                    printf("motor %d 's PVT spd min = %.2f rad/s, spd max = %.2f rad/s\n",
                        RxMessage->motor[i].id, motor_pvt_params.spd_min, motor_pvt_params.spd_max);
                }
                else if (motor_comm_fbd.INS_code == 27 && RxMessage->motor[i].dlc == 6) // get PVT TOR range
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    int16_t tor_min_raw = rv_type_convert2.to_int16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    int16_t tor_max_raw = rv_type_convert2.to_int16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.tor_min = tor_min_raw / 10.0f;
                    motor_pvt_params.tor_max = tor_max_raw / 10.0f;
                    printf("motor %d 's PVT tor min = %.1f Nm, tor max = %.1f Nm\n",
                        RxMessage->motor[i].id, motor_pvt_params.tor_min, motor_pvt_params.tor_max);
                }
                else if (motor_comm_fbd.INS_code == 28 && RxMessage->motor[i].dlc == 6) // get PVT CUR range
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    int16_t cur_min_raw = rv_type_convert2.to_int16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    int16_t cur_max_raw = rv_type_convert2.to_int16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.cur_min = cur_min_raw / 10.0f;
                    motor_pvt_params.cur_max = cur_max_raw / 10.0f;
                    printf("motor %d 's PVT cur min = %.1f A, cur max = %.1f A\n",
                        RxMessage->motor[i].id, motor_pvt_params.cur_min, motor_pvt_params.cur_max);
                }
                else if (motor_comm_fbd.INS_code == 31 && RxMessage->motor[i].dlc == 4) // get CAN timeout
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.can_timeout_ms = rv_type_convert2.to_uint16;
                    printf("motor %d 's CAN timeout = %d ms\n",
                        RxMessage->motor[i].id, motor_pvt_params.can_timeout_ms);
                }
                else if (motor_comm_fbd.INS_code == 32 && RxMessage->motor[i].dlc == 6) // get current loop PI
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    uint16_t cur_kp_raw = rv_type_convert2.to_uint16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    uint16_t cur_ki_raw = rv_type_convert2.to_uint16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.cur_loop_kp = cur_kp_raw / 10000.0f;
                    motor_pvt_params.cur_loop_ki = cur_ki_raw / 10.0f;
                    printf("motor %d 's current loop KP = %.4f, KI = %.1f\n",
                        RxMessage->motor[i].id, motor_pvt_params.cur_loop_kp, motor_pvt_params.cur_loop_ki);
                }
                else if (motor_comm_fbd.INS_code == 33 && RxMessage->motor[i].dlc == 6) // get speed loop PI
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    uint16_t spd_kp_raw = rv_type_convert2.to_uint16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    uint16_t spd_ki_raw = rv_type_convert2.to_uint16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.spd_loop_kp = spd_kp_raw / 100000.0f;
                    motor_pvt_params.spd_loop_ki = spd_ki_raw / 100000.0f;
                    printf("motor %d 's speed loop KP = %.5f, KI = %.5f\n",
                        RxMessage->motor[i].id, motor_pvt_params.spd_loop_kp, motor_pvt_params.spd_loop_ki);
                }
                else if (motor_comm_fbd.INS_code == 34 && RxMessage->motor[i].dlc == 6) // get position loop PD
                {
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[3];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[2];
                    uint16_t pos_kp_raw = rv_type_convert2.to_uint16;
                    rv_type_convert2.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert2.buf[1] = RxMessage->motor[i].data[4];
                    uint16_t pos_kd_raw = rv_type_convert2.to_uint16;
                    motor_pvt_params.motor_id = RxMessage->motor[i].id;
                    motor_pvt_params.pos_loop_kp = pos_kp_raw / 100000.0f;
                    motor_pvt_params.pos_loop_kd = pos_kd_raw / 100000.0f;
                    printf("motor %d 's position loop KP = %.5f, KD = %.5f\n",
                        RxMessage->motor[i].id, motor_pvt_params.pos_loop_kp, motor_pvt_params.pos_loop_kd);
                }
            }

            // return ack_status;
        }
    }
}
