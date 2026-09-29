#include "ec_api_without_ros.hpp"
#include "pace_stamp.h"
#include <cmath>
#include <mutex>
#include <thread>
#include <future>
#include <fcntl.h>      // open()
#include <sys/file.h>   // flock()
#include <unistd.h>     // getpid()/write()/close()
#include <cerrno>       // errno
#include <cstring>      // strerror()
#include <sys/stat.h>  // fchmod, chmod, fstat 等

std::mutex output_lock; 

int motor_number = 6; //固定发送或回传6个电机的消息

extern int motor_id_safe_check[29]; //零位检查数组
extern float motor_pos_safe_check[29];
extern float motor_set_zero_position[29]; //未部署-标零反馈
double last_parallel_vel[6];
bool show_debug_info = false;
bool safe_torque_mode = false;
bool local_ankel_solver1 = false;
bool local_ankel_solver2 = true;
bool local_waist_solver2 = true;

//内部参数传递，不要直接赋值
extern EtherCAT_Msg Tx_Message_fd[CHANNEL_NUMBER];
extern EtherCAT_Msg Rx_Message_fd[CHANNEL_NUMBER];
extern OD_Motor_Msg rv_motor_msg[7];
extern MotorCommFbd motor_comm_fbd;

//电机报文参数-不同电机参数略有不同，用于CAN消息封包解包 //硬件限制
extern MotorParaLimits A10020P2;
extern MotorParaLimits A10020P1;
extern MotorParaLimits A8112P1;
extern MotorParaLimits A6408P2;

//关节参数 //软件限制 //需要调用api启用
const MotorRealLimits l_hip_roll=   {1 , -1.57, 1.57, -30, 30, A10020P1};
const MotorRealLimits l_hip_yaw=    {2 , -1.57, 1.57, -30, 30, A8112P1};
const MotorRealLimits l_hip_pitch=  {3 , -1.57, 1.57, -50, 50, A10020P2};
const MotorRealLimits l_knee_pitch= {4 , -1.57, 1.57, -50, 50, A10020P2};
const MotorRealLimits l_ankle_pitch={5 , -1.57, 1.57, -20, 20, A6408P2};
const MotorRealLimits l_ankle_row=  {6 , -1.57, 1.57, -20, 20, A6408P2};

const MotorRealLimits r_hip_roll=   {7 , -1.57, 1.57, -30, 30, A10020P1};
const MotorRealLimits r_hip_yaw=    {8 , -1.57, 1.57, -30, 30, A8112P1};
const MotorRealLimits r_hip_pitch=  {9 , -1.57, 1.57, -50, 50, A10020P2};
const MotorRealLimits r_knee_pitch= {10, -1.57, 1.57, -50, 50, A10020P2};
const MotorRealLimits r_ankle_pitch={11, -1.57, 1.57, -20, 20, A6408P2};
const MotorRealLimits r_ankle_row=  {12, -1.57, 1.57, -20, 20, A6408P2};

Limits12Motors limit_of_motor[12]={l_hip_roll,l_hip_yaw,l_hip_pitch,l_knee_pitch,l_ankle_pitch,l_ankle_row,
                                            r_hip_roll,r_hip_yaw,r_hip_pitch,r_knee_pitch,r_ankle_pitch,r_ankle_row};

MotorCmd motor_cmd_upper;
MotorState motor_state_output; //原始电机数据
MotorState motor_state_converted; //并联解算后的电机数据

// double r_l = 0.683;
// double r_l_w = 0.626;
// FastAnkelSolver solver(r_l);

ParallelMechanism solver_tx;
ParallelMechanism solver_rx;

void cmdCallback(MotorCmd* motor_cmd) {
    // struct timespec start, end;
    // clock_gettime(CLOCK_MONOTONIC, &start);

    sendMotorCmd(motor_cmd);

    // clock_gettime(CLOCK_MONOTONIC, &end);
    // long delta_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
    // printf("cmdCallback: %ldμs\n", delta_ns / 1000);

    // for (int i = 0; i < 12; ++i){
    //     motor_state_output.motor_id[i] = 0;
    // }
}
void sendMotorCmd(MotorCmd* motor_cmd_){
    if (motor_cmd_ != nullptr) {  // 安全检查
        motor_cmd_upper = *motor_cmd_;
    }
}

// ── 力矩限幅日志(边沿触发) ──────────────────────────────────────────
// sendMotorCmd_ 运行在 1kHz 实时线程, 限幅可能长时间持续触发;
// 仅在"进入饱和"与"退出饱和"时各记一条 mc_log, 避免刷log/阻塞实时循环。
// 状态按关节记录: 0=未饱和, +1=正向饱和, -1=负向饱和。
static int8_t tff_sat_state[29] = {0};

static inline void tff_clamp_log(int i, float& val, float limit)
{
    if (val > limit){
        if (tff_sat_state[i] != 1){
            tff_sat_state[i] = 1;
            mc_log("[TFF-SAT] joint %d saturated: %.2f clamped to +%.1f", i, val, limit);
        }
        val = limit;
    }
    else if (val < -limit){
        if (tff_sat_state[i] != -1){
            tff_sat_state[i] = -1;
            mc_log("[TFF-SAT] joint %d saturated: %.2f clamped to -%.1f", i, val, limit);
        }
        val = -limit;
    }
    else if (tff_sat_state[i] != 0){
        tff_sat_state[i] = 0;
        mc_log("[TFF-SAT] joint %d recovered: %.2f within +/-%.1f", i, val, limit);
    }
}

void sendMotorCmd_(){
    struct timespec start, end;
    struct timespec remaining;
    while(true){

    clock_gettime(CLOCK_MONOTONIC, &start);

    MotorCmd motor_cmd;
    motor_cmd = motor_cmd_upper;

    if(HUMANOID_TYPE == 2){
        if (motor_cmd.mode[0] == 1){ //pos = 度， spd = rpm * 10， cur_limit = realcur * 10
            printf("POSITION MODE! \n");
            for (int index = 0; index < 12; ++index){
                int channel = fd_index_2_channel(index);
                if (channel == -1){
                    continue;
                }
                int passage = index % 7 + 1; //连续六个数模6的值必不相等
                float q_des = motor_cmd.q_des[index] + lf1_zero_offset_deg[index];
                set_motor_position(&Tx_Message_fd[channel], passage, motor_cmd.motor_id[index], q_des, motor_cmd.qd_des[index], 100, 1);
            }
            // set_motor_position(&Tx_Message_fd[0], 1, motor_cmd.motor_id[0], motor_cmd.q_des[0], motor_cmd.qd_des[0], 100, 1);
            sendToFD();
        }

        else if (motor_cmd.mode[0] == 2 or motor_cmd.mode[0] == 3){
            if (motor_cmd.mode[0] == 2){
                local_ankel_solver1 = false;
                local_ankel_solver2 = false;
                local_waist_solver2 = false;
            }

                        //pd control
            int dir_cmd[15] = {0,0,0,0,1,1,
                            0,0,0,0,1,1,
                            0,1,1};
	        int dir_obs[15] = {0,0,0,0,1,1,
                            0,0,0,0,1,1,
                            0,1,1};
            for (int i = 4; i < 15; ++i){
                if(i == 4 or i == 5 or i == 10 or i == 11 or i == 13 or i == 14){
                    motor_cmd.tff[i] = motor_cmd.kp[i] * (motor_cmd.q_des[i] * dir_cmd[i] - motor_state_converted.q[i] * dir_obs[i]) 
                    - motor_cmd.kd[i] * motor_state_converted.qd[i] * dir_obs[i];
                    motor_cmd.kp[i] = 0;
                    motor_cmd.kd[i] = 0;
                    // 并联关节(逆解输入)关节力矩限幅: i=4,10 与 i=13,14 为 [-60,60]; i=5,11 为 [-20,20]
                    float para_limit = 60.0f;
                    if (i == 5 or i == 11){
                        para_limit = 20.0f;
                    }
                    if (motor_cmd.tff[i] > para_limit){
                        motor_cmd.tff[i] = para_limit;
                    }
                    if (motor_cmd.tff[i] < -para_limit){
                        motor_cmd.tff[i] = -para_limit;
                    }
                }
            }

            size_t array_size = sizeof(motor_cmd.q_des) / sizeof(motor_cmd.q_des[0]);
            std::vector<double> q_d(motor_state_converted.q, motor_state_converted.q + array_size);
            std::vector<double> v_d(motor_cmd.qd_des, motor_cmd.qd_des + array_size);
            std::vector<double> tq_d(motor_cmd.tff, motor_cmd.tff + array_size);

            if (local_ankel_solver2 == true){
                // 逆向解算：三个解算相互独立（分别读写不同的全局 last_motor_* 状态）。
                // 单个解算已优化到微秒级，相比每个周期新建 3 个线程（创建+调度约几十
                // 微秒）的开销，同步执行延迟更低、时序更确定，且避免 1kHz 实时循环
                // 中反复创建线程导致的不确定性。
                double alpha4, alpha5, v4, v5, tq4, tq5;
                std::tie(alpha4, alpha5, v4, v5, tq4, tq5) = solver_tx.jointToMotorLeft(
                        q_d[5], q_d[4], v_d[5], v_d[4], tq_d[5], tq_d[4]);

                double alpha10, alpha11, v10, v11, tq10, tq11;
                std::tie(alpha10, alpha11, v10, v11, tq10, tq11) = solver_tx.jointToMotorRight(
                    q_d[11], q_d[10], v_d[11], v_d[10], tq_d[11], tq_d[10]);

                double alpha13, alpha14, v13, v14, tq13, tq14;
                std::tie(alpha13, alpha14, v13, v14, tq13, tq14) = solver_tx.jointToMotorW(
                    q_d[13], q_d[14], v_d[13], v_d[14], tq_d[13], tq_d[14]);

                double vel[6] = {v4,v5,v10,v11,v13,v14};
                double alpha = 0.5;
                for (int i = 0; i < 6; ++i){
                    vel[i] = vel[i] * alpha + last_parallel_vel[i] * (1 - alpha);
                    last_parallel_vel[i] = vel[i];
                }

                q_d[4] = alpha4;
                q_d[5] = alpha5;
                v_d[4] = vel[0];
                v_d[5] = vel[1];
                tq_d[4] = tq4;
                tq_d[5] = tq5;
                
                q_d[10] = alpha10;
                q_d[11] = alpha11;
                v_d[10] = vel[2];
                v_d[11] = vel[3];
                tq_d[10] = tq10;
                tq_d[11] = tq11;

                q_d[13] = alpha13;
                q_d[14] = alpha14;
                v_d[13] = vel[4];
                v_d[14] = vel[5];
                tq_d[13] = tq13;
                tq_d[14] = tq14;         

            }
            for (int i = 4; i <12; ++i){
                if(i == 4 or i == 5 or i == 10 or i == 11){
                    motor_cmd.q_des[i] = 0.0;
                    motor_cmd.qd_des[i] = 0.0;
                    motor_cmd.kd[i] = motor_cmd.kd_ff[0];
                    motor_cmd.tff[i] = static_cast<float>(tq_d[i]);
                    tff_clamp_log(i, motor_cmd.tff[i], 25.0f); // 限幅±25, 饱和/恢复时记log
                }
            }
            for (int i = 13; i <15; ++i){
                motor_cmd.q_des[i] = 0.0;
                motor_cmd.qd_des[i] = 0.0;
                motor_cmd.kd[i] = motor_cmd.kd_ff[1];
                motor_cmd.tff[i] = static_cast<float>(tq_d[i]);
                tff_clamp_log(i, motor_cmd.tff[i], 50.0f);    // 限幅±50, 饱和/恢复时记log
            }
                //size_t array_size = sizeof(motor_cmd.q_des) / sizeof(motor_cmd.q_des[0]);
                //std::vector<double> q_d(motor_cmd.q_des, motor_cmd.q_des + array_size);
                //std::vector<double> v_d(motor_cmd.qd_des, motor_cmd.qd_des + array_size);
                //std::vector<double> tq_d(motor_cmd.tff, motor_cmd.tff + array_size);
                //solver_w_tx.jointToMotorAll(q_d, v_d, tq_d);
                // for (int i = 13; i <15; ++i){
                //     motor_cmd.q_des[i] = 0.0;
                //     motor_cmd.qd_des[i] = 0.0;
                //     motor_cmd.kp[i] = 100.0;
                //     motor_cmd.kd[i] = 20.0;
                //     motor_cmd.tff[i] = 0;
                // }

            motor_state_output.msg_state = motor_cmd.msg_cmd;
            // printf("motor_state_output.msg_state = %d", motor_state_output.msg_state);
            for (int index = 0; index < 29; ++index){
                int channel = fd_index_2_channel(index);
                if (channel == -1){
                    continue;
                }
                int passage = index % 7 + 1; 
                float q_des = motor_cmd.q_des[index] + lf1_zero_offset_rad[index];
                send_motor_ctrl_cmd_fd(&Tx_Message_fd[channel],passage,motor_cmd.motor_id[index],motor_cmd.kp[index],motor_cmd.kd[index],
                    q_des,motor_cmd.qd_des[index],motor_cmd.tff[index]);
            }
            sendToFD();
        }

        else{
            sendToFD(); //用于leg_tool功能函数发送
        }

        if (show_debug_info == true || motor_cmd.msg_cmd == 1){
                for (int i = 0; i < 15; ++i){
                    printf("data: id = %d, kp = %.1f, kd = %.1f, q_des = %.3f, tff = %.3f, q = %.3f, qd = %.3f, tau = %.3f\n",motor_state_output.motor_id[i],
                        motor_cmd.kp[i],motor_cmd.kd[i],motor_cmd.q_des[i],motor_cmd.tff[i],motor_state_output.q[i],motor_state_output.qd[i],motor_state_output.tau[i]);
                }
                printf("\n");
            }
        // struct timespec now_;
        // clock_gettime(CLOCK_MONOTONIC, &now_);
        // long delta_ns = (now_.tv_sec) * 1e9 + (now_.tv_nsec);
        // printf("cmdCallback: %ldμs\n", delta_ns / 1000);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);

    long elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000L 
                    + (end.tv_nsec - start.tv_nsec);
    const long TARGET_NS = 1 * 1000000L; // 2ms = 2,000,000 ns
    
    if (elapsed_ns < TARGET_NS) {
        long sleep_ns = TARGET_NS - elapsed_ns;
        remaining.tv_sec = sleep_ns / 1000000000L;
        remaining.tv_nsec = sleep_ns % 1000000000L;
        nanosleep(&remaining, nullptr);
    }
}
}
int rx_loss = 0;
static uint32_t last_rx_state_id = 0; //getFromFD 刚解包的"应答帧1"的电机id, 非状态帧为0, 供 PACE 打时间戳


void motorState_fd(){
    while(true){
        for (int channel = 0; channel < 6; ++channel){
            while (messages_fd_rx[channel].read_available()>0){//回传电机数据
                rx_loss = 0;
                //printf("test-time read available \n");
                getFromFD(channel);
                //printf("test-time 解包 \n");

                for (int i = 0; i < 7; ++i){
                    if (rv_motor_msg[i].motor_id == 0){
                        //printf("rv_motor_msg[i].motor_id = %d", i);
                        continue;
                    }
                    //使用电机ID填充到状态信息数组的指定位置。比如1号电机的数组下标是[0]。
                    // printf("rv_motor_msg[i].motor_id = %d", rv_motor_msg[i].motor_id);
                    int index = fd_id_2_index(rv_motor_msg[i].motor_id);
                    if (index == -1){
                        continue;
                    }
                    float rad_mapped = rv_motor_msg[i].angle_actual_rad - lf1_zero_offset_rad[index];
                    // output_lock.lock();
                    motor_state_output.motor_id[index] = rv_motor_msg[i].motor_id;
                    motor_state_output.q[index] = rad_mapped;
                    motor_state_output.qd[index] = rv_motor_msg[i].speed_actual_rad;
                    motor_state_output.tau[index] = rv_motor_msg[i].torque_float;
                    // output_lock.unlock();
                    if (rv_motor_msg[i].motor_id == last_rx_state_id){
                        pace_record_rx(index, rad_mapped, rv_motor_msg[i].speed_actual_rad, rv_motor_msg[i].torque_float,
                                       rv_motor_msg[i].temperature, rv_motor_msg[i].error);
                    }
                }
            }
        }
        usleep(330);
    }
}

bool check_safe_pos(float current_pos, MotorRealLimits motor_real_limits){
    if (current_pos > motor_real_limits.real_pos_max){
        return false;
    }
    if (current_pos < motor_real_limits.real_pos_min){
        return false;
    }
    return true;
}

bool check_safe_torque(float current_torque, MotorRealLimits motor_real_limits){
    if (current_torque > motor_real_limits.real_torque_max){
        return false;
    }
    if (current_torque < motor_real_limits.real_torque_min){
        return false;
    }
    return true;
}

bool check_safe_torque_multi(){
    bool safe_flag = true;
    for (int i = 0; i < 12; ++i){
        safe_flag = safe_flag && check_safe_torque(motor_state_output.tau[i], limit_of_motor[i].motor_real_limits);
    }
    return safe_flag;
}

void safe_torque_break(){
    if (check_safe_torque_multi() == false){
        motorBreak();
    }
}

void motorBreak(){

    if (HUMANOID_TYPE == 2){
        for (int index = 0; index < 29; ++index){
                int channel = fd_index_2_channel(index);
                int passage = index % 7 + 1; //连续六个数模6的值必不相等
                int id = fd_index_2_id(index);
                set_motor_cur_tor(&Tx_Message_fd[channel], passage, id, 0, 2, 1);
            }
    }
    
}

void motorBackToZero(){

    if (HUMANOID_TYPE == 2){
        for (int index = 0; index < 6; ++index){
            int id = fd_index_2_id(index);
            set_motor_position(&Tx_Message_fd[LF1_LEG_L], index+1, id, lf1_zero_offset_deg[index], 3, 200, 1);
        }
        for (int index = 6; index < 12; ++index){
            int id = fd_index_2_id(index);
            set_motor_position(&Tx_Message_fd[LF1_LEG_R], index-5, id, lf1_zero_offset_deg[index], 3, 200, 1);
        }
        for (int index = 12; index < 15; ++index){
            int id = fd_index_2_id(index);
            set_motor_position(&Tx_Message_fd[LF1_WAIST], index-11, id, lf1_zero_offset_deg[index], 3, 200, 1);
        }
        for (int index = 15; index < 22; ++index){
            int id = fd_index_2_id(index);
            set_motor_position(&Tx_Message_fd[LF1_ARM_L], index-14, id, lf1_zero_offset_deg[index], 3, 200, 1);
        }
        for (int index = 22; index < 29; ++index){
            int id = fd_index_2_id(index);
            set_motor_position(&Tx_Message_fd[LF1_ARM_R], index-21, id, lf1_zero_offset_deg[index], 3, 200, 1);
        }
    }
}

//清空所有通道的Tx_Message_fd(将电机can_id全部置0)。
//sendMotorCmd_()线程会以约1kHz的周期不断把Tx_Message_fd里的报文重复下发,
//debug功能(标零/改ID/查询参数)把报文写进Tx_Message_fd后若不清理,
//该报文就会被持续高频重复发送,电机应答也会被持续回传打印。
//所以每个debug功能函数结尾统一:间隔2ms(保证报文已下发1~2次)后调用本函数清空,
//can_id为0的报文会被底层(tiktak分组/encos_send_frames)直接丢弃,不再上总线。
void clearTxMessageFd(){
    for (int channel = 0; channel < CHANNEL_NUMBER; ++channel){
        for (int i = 0; i < 7; ++i){
            Tx_Message_fd[channel].motor[i].id = 0;  //can_id置0,底层不再发送
            Tx_Message_fd[channel].motor[i].dlc = 0;
            Tx_Message_fd[channel].motor[i].rtr = 0;
        }
        Tx_Message_fd[channel].motor_num = 0;
    }
}

void motorSetZero(){

    if (HUMANOID_TYPE == 2){
        int input_id;
        std::cout << "输入一个电机ID: ";
        std::cin >> input_id; // 阻塞等待回车
        std::cout << "对电机标零，ID: " << input_id << std::endl;
        switch (input_id){
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
                MotorSetting_multi(&Tx_Message_fd[LF1_LEG_L], 1, input_id, 0x03);//必须指定CAN总线
                break;
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
                MotorSetting_multi(&Tx_Message_fd[LF1_LEG_R], 1, input_id, 0x03);//必须指定CAN总线
                break;
            case 41:
            case 42:
            case 43:
                MotorSetting_multi(&Tx_Message_fd[LF1_WAIST], 1, input_id, 0x03);//必须指定CAN总线
                break;
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
                MotorSetting_multi(&Tx_Message_fd[LF1_ARM_R], 1, input_id, 0x03);//必须指定CAN总线
                break;
            case 23:
            case 24:
            case 25:
            case 26:
            case 27:
            case 28:
            case 29:
                MotorSetting_multi(&Tx_Message_fd[LF1_ARM_L], 1, input_id, 0x03);//必须指定CAN总线
                break;
            default:
                break;
        }
        usleep(10000);
        clearTxMessageFd();    //清空Tx_Message_fd(can_id置0),停止0x03指令高频重复下发
    }
    

}

void getFromFD(int channel){
    last_rx_state_id = 0;
    if (!messages_fd_rx[channel].empty()) {
        EtherCAT_Msg_ptr msg_rx = messages_fd_rx[channel].front();
        messages_fd_rx[channel].pop();
        EtherCAT_Msg &can_msg_rx = *msg_rx;
        //receive_loop 总是把帧放在 motor[0]; Byte0 高3位 = 1 才是携带位置/速度/力矩的"应答帧1"
        const Motor_Msg &frame = can_msg_rx.motor[0];
        if (frame.dlc != 0 && frame.id != 0x7FF && (frame.data[0] >> 5) == 1){
            last_rx_state_id = frame.id;
        }
        RV_fd_data_repack_multi(&can_msg_rx, comm_ack, 0);
    }
}

void sendToFD(){
    EtherCAT_Msg_ptr can_msg_tx[6] = {};
    for (int channel = 0; channel < 6; ++channel){
        can_msg_tx[channel] = std::make_shared<EtherCAT_Msg>(Tx_Message_fd[channel]);
        sendToQueue_fd_tx(channel, can_msg_tx[channel]); //发送到无锁队列
    }
}

void posCheck(){

    if (HUMANOID_TYPE == 2){
        for (int index = 0; index < 29; ++index){ //
            int id;
            id = fd_index_2_id(index);
            get_motor_parameter(&Tx_Message_fd[0], 1, id, 1); //虽然是channel0，但是实际channel由底层根据id号决定。
            usleep(6000);
            if (true){//回传电机数据
                printf("ID = %d, Pos = %f\n", motor_id_safe_check[index], motor_pos_safe_check[index]);
            }
        }
        usleep(5000);          //间隔2ms,确保最后一条查询报文已下发
        clearTxMessageFd();    //清空查询报文,停止1kHz高频重复查询
    }
}

bool idSet(){

    if (HUMANOID_TYPE == 2){
        printf("输入CAN总线！输入值范围0-5\n");
        int channel;
        std::cin >> channel;
        if (channel > 5 or channel < 0){
            printf("无效总线:%d！程序终止！", channel);
            return false;
        }
        printf("输入原id号！输入值范围1-255\n");
        int motor_id;
        std::cin >> motor_id;
        if (motor_id > 255 or motor_id < 1){
            printf("无效原id:%d！程序终止！", motor_id);
            return false;
        }
        printf("输入新id号！输入值范围1-255\n");
        int motor_id_new;
        std::cin >> motor_id_new;
        if (motor_id_new > 255 or motor_id_new < 1){
            printf("无效新id%d！程序终止！", motor_id_new);
            return false;
        }
        printf("您想要将总线 %d 的 %d 号电机修改为 %d 号电机\n", channel, motor_id, motor_id_new);

        char confirmation;
        std::cout << "确认修改吗？(y/n): ";
        std::cin >> confirmation;
        if (std::tolower(confirmation) != 'y') {
            printf("\n终止修改！\n");
            return false;
        }
        printf("\n开始修改！\n");
        MotorIDSettingMulti(&Tx_Message_fd[channel], 1, motor_id, motor_id_new);
        usleep(5000);

        clearTxMessageFd();    //清空改ID报文,停止1kHz高频重复下发
        return true;
    }
}

void get_pvt_kp(uint16_t id){
    get_motor_parameter(&Tx_Message_fd[0], 1, id, 23);
    usleep(5000);          //间隔2ms,确保查询报文已下发
    clearTxMessageFd();    //清空查询报文,避免持续重复查询导致应答反复打印
}

void get_pvt_kd(uint16_t id){
    get_motor_parameter(&Tx_Message_fd[0], 1, id, 24);
    usleep(5000);          //间隔2ms,确保查询报文已下发
    clearTxMessageFd();    //清空查询报文,避免持续重复查询导致应答反复打印
}

/*
 * 读取指定电机的力位混控(PVT)模式参数并打印。
 * 依次发送"电机控制参数查询指令"(ENCOS 文档 9.3 节),等待返回后打印:
 *   23: PVT KP 范围      24: PVT KD 范围      25: PVT POS 范围
 *   26: PVT SPD 范围     27: PVT TOR 范围     28: PVT CUR 范围
 *   31: CAN 超时时间     32: 电流环 PI        33: 速度环 PI
 *   34: 位置环 PD
 * 返回报文由 motorState_fd 线程解包(motor_control.c 的 RV_fd_data_repack_multi),
 * 结果打印并存入 motor_pvt_params。
 */
void get_motor_pvt_params(uint16_t id){
    printf("开始查询电机 %d 的力位混控(PVT)参数...\n", id);
    uint8_t param_cmds[] = {23, 24, 25, 26, 27, 28, 31, 32, 33, 34};
    for (uint8_t cmd : param_cmds){
        get_motor_parameter(&Tx_Message_fd[0], 1, id, cmd);
        usleep(5000);          //间隔2ms,确保本条查询报文经1ms线程下发1~2次
        clearTxMessageFd();    //立即清空,避免同一条查询被1kHz持续重复下发、应答被反复打印
        usleep(48 * 1000);     //剩余时间等待电机应答(回传由motorState_fd线程解包打印一次)
    }
    usleep(4000);
    clearTxMessageFd();        //收尾清空,停止最后一条查询报文的高频重复下发
    printf("电机 %d 力位混控参数查询完成,详见上方输出。\n", id);
}

/*
 * 发送一条"电机参数配置指令"(ENCOS 文档 9.2 节)并确认返回。
 * ack_status=1: 要求电机返回报文类型4(配置代码+配置状态)。
 * 返回报文由 motorState_fd 线程解包(motor_control.c 的 RV_fd_data_repack_multi),
 * 结果写入全局 motor_comm_fbd(INS_code=配置代码, motor_fbd=配置状态 0失败/1成功)。
 */
static void set_motor_param_once(uint16_t id, uint8_t cfg_code, uint16_t val1, uint16_t val2, uint8_t dlc){
    set_motor_parameter(&Tx_Message_fd[0], 1, id, cfg_code, 1, val1, val2, dlc);
    usleep(5000);          //间隔2ms,确保配置报文经1ms线程下发
    clearTxMessageFd();    //立即清空,避免配置报文被1kHz持续重复下发
    usleep(50 * 1000);     //等待电机返回配置状态(motorState_fd线程解包)
    if (motor_comm_fbd.INS_code == cfg_code){
        printf("配置%s: 代码 0x%02X\n", motor_comm_fbd.motor_fbd == 1 ? "成功" : "失败", cfg_code);
    } else {
        printf("未收到配置返回: 代码 0x%02X\n", cfg_code);
    }
}

/*
 * 交互式修改电机的力位混控(PVT)及控制参数(仿照 get_motor_pvt_params)。
 * 对应 ENCOS 技术文档 V1.14 第 9.2 节"电机控制参数配置指令":
 *   0x01 系统加速度  0x04 扭矩系数  0x05 PVT KP  0x06 PVT KD
 *   0x07 PVT POS     0x08 PVT SPD   0x09 PVT TOR 0x0a PVT CUR
 *   0x0b CAN超时     0x0c 电流环PI  0x0d 速度环PI 0x0e 位置环PD
 */
void set_motor_pvt_params(void){
    uint16_t id;
    printf("开始设置电机力位混控(PVT)参数\n");
    printf("输入电机ID: ");
    std::cin >> id;

    printf("\n选择要设置的参数:\n");
    printf("  1: PVT KP 范围 (min, max)\n");
    printf("  2: PVT KD 范围 (min, max)\n");
    printf("  3: PVT 位置范围 (min, max, rad)\n");
    printf("  4: PVT 速度范围 (min, max, rad/s)\n");
    printf("  5: PVT 扭矩范围 (min, max, Nm)\n");
    printf("  6: PVT 电流范围 (min, max, A)\n");
    printf("  7: CAN 超时时间 (ms, 0=关闭)\n");
    printf("  8: 电流环 PI (kp, ki)\n");
    printf("  9: 速度环 PI (kp, ki)\n");
    printf(" 10: 位置环 PD (kp, kd)\n");
    printf(" 11: 系统加速度 (rad/s^2)\n");
    printf(" 12: 扭矩系数\n");
    printf("选择: ");
    int sel;
    std::cin >> sel;

    double v1 = 0.0, v2 = 0.0;
    uint8_t cfg_code = 0, dlc = 6;
    uint16_t raw1 = 0, raw2 = 0;
    bool two_val = true;

    switch (sel){
        case 1:  cfg_code = 0x05; // PVT KP: uint16 比例1
            printf("输入 KP MIN: "); std::cin >> v1;
            printf("输入 KP MAX: "); std::cin >> v2;
            raw1 = (uint16_t)std::lround(v1);
            raw2 = (uint16_t)std::lround(v2);
            break;
        case 2:  cfg_code = 0x06; // PVT KD: uint16 比例1
            printf("输入 KD MIN: "); std::cin >> v1;
            printf("输入 KD MAX: "); std::cin >> v2;
            raw1 = (uint16_t)std::lround(v1);
            raw2 = (uint16_t)std::lround(v2);
            break;
        case 3:  cfg_code = 0x07; // PVT POS: int16 比例100
            printf("输入 POS MIN (rad): "); std::cin >> v1;
            printf("输入 POS MAX (rad): "); std::cin >> v2;
            raw1 = (uint16_t)(int16_t)std::lround(v1 * 100.0);
            raw2 = (uint16_t)(int16_t)std::lround(v2 * 100.0);
            break;
        case 4:  cfg_code = 0x08; // PVT SPD: int16 比例100
            printf("输入 SPD MIN (rad/s): "); std::cin >> v1;
            printf("输入 SPD MAX (rad/s): "); std::cin >> v2;
            raw1 = (uint16_t)(int16_t)std::lround(v1 * 100.0);
            raw2 = (uint16_t)(int16_t)std::lround(v2 * 100.0);
            break;
        case 5:  cfg_code = 0x09; // PVT TOR: int16 比例10
            printf("输入 TOR MIN (Nm): "); std::cin >> v1;
            printf("输入 TOR MAX (Nm): "); std::cin >> v2;
            raw1 = (uint16_t)(int16_t)std::lround(v1 * 10.0);
            raw2 = (uint16_t)(int16_t)std::lround(v2 * 10.0);
            break;
        case 6:  cfg_code = 0x0a; // PVT CUR: int16 比例10
            printf("输入 CUR MIN (A): "); std::cin >> v1;
            printf("输入 CUR MAX (A): "); std::cin >> v2;
            raw1 = (uint16_t)(int16_t)std::lround(v1 * 10.0);
            raw2 = (uint16_t)(int16_t)std::lround(v2 * 10.0);
            break;
        case 7:  cfg_code = 0x0b; dlc = 4; two_val = false; // CAN超时: uint16 比例1
            printf("输入超时时间 (ms, 0=关闭): "); std::cin >> v1;
            raw1 = (uint16_t)std::lround(v1);
            break;
        case 8:  cfg_code = 0x0c; // 电流环 PI: KP*10000, KI*10
            printf("输入 Current KP: "); std::cin >> v1;
            printf("输入 Current KI: "); std::cin >> v2;
            raw1 = (uint16_t)std::lround(v1 * 10000.0);
            raw2 = (uint16_t)std::lround(v2 * 10.0);
            break;
        case 9:  cfg_code = 0x0d; // 速度环 PI: KP*100000, KI*100000
            printf("输入 Speed KP: "); std::cin >> v1;
            printf("输入 Speed KI: "); std::cin >> v2;
            raw1 = (uint16_t)std::lround(v1 * 100000.0);
            raw2 = (uint16_t)std::lround(v2 * 100000.0);
            break;
        case 10: cfg_code = 0x0e; // 位置环 PD: KP*100000, KD*100000
            printf("输入 Position KP: "); std::cin >> v1;
            printf("输入 Position KD: "); std::cin >> v2;
            raw1 = (uint16_t)std::lround(v1 * 100000.0);
            raw2 = (uint16_t)std::lround(v2 * 100000.0);
            break;
        case 11: cfg_code = 0x01; dlc = 4; two_val = false; // 系统加速度: uint16 比例100
            printf("输入加速度 (rad/s^2): "); std::cin >> v1;
            raw1 = (uint16_t)std::lround(v1 * 100.0);
            break;
        case 12: cfg_code = 0x04; dlc = 4; two_val = false; // 扭矩系数: uint16 比例100
            printf("输入扭矩系数: "); std::cin >> v1;
            raw1 = (uint16_t)std::lround(v1 * 100.0);
            break;
        default:
            printf("无效选择\n");
            return;
    }

    set_motor_param_once(id, cfg_code, raw1, raw2, two_val ? 6 : 4);
}
//
//API FOR SIM2REAL

int motor_init(){
    // ── 防双开:进程级 flock 锁,持有到进程退出(崩溃/被杀时内核自动释放) ──
    static int s_can_lock_fd = -1;                  // fd 必须活到进程结束:fd 关闭即释放锁
    if (s_can_lock_fd >= 0){
        // 同进程重入:锁已由本进程持有,拒绝再次初始化
        printf("[CAN-BUS] motor_init 已在本进程调用过,拒绝重复初始化。\n");
        return -1;
    }
    s_can_lock_fd = open("/tmp/leg_can_owner.lock", O_CREAT | O_RDWR, 0666);
    if (s_can_lock_fd < 0) {
        // open 失败 ≠ 锁被占用。最常见:上次以 sudo 运行的进程创建了 root 属主、
        // 实际权限 0644(0666 被 umask 削减)的锁文件,普通用户无法 O_RDWR 打开。
        printf("[CAN-BUS] 打开锁文件 /tmp/leg_can_owner.lock 失败: %s\n", strerror(errno));
        printf("[CAN-BUS] 若为权限问题(Permission denied),执行 sudo rm /tmp/leg_can_owner.lock 后重试。\n");
        return -1;
    }
    fchmod(s_can_lock_fd, 0666);                      // 抵消 umask,避免 root 创建的锁文件阻塞普通用户
    // 有界重试:上个实例 Ctrl+C 退出时,信号处理函数里的 close_canDevice()
    // (6 次 system("sudo ip link down"))需 1~2 秒才走到 exit,此窗口内锁仍被持有。
    // 最多等 5 秒,抹掉这个退出竞态;真有实例在跑时最终仍会失败退出。
    bool locked = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (flock(s_can_lock_fd, LOCK_EX | LOCK_NB) == 0) {
            locked = true;
            break;
        }
        if (attempt == 0) {
            printf("[CAN-BUS] 锁暂被占用,等待持有者退出(最多 5 秒)...\n");
        }
        usleep(250 * 1000);
    }
    if (!locked) {
        printf("[CAN-BUS] CAN 总线已被其它进程占用(motor_init 已在运行),本实例直接退出。\n");
        printf("[CAN-BUS] 占用进程 pid 见锁文件: cat /tmp/leg_can_owner.lock\n");
        close(s_can_lock_fd);
        s_can_lock_fd = -1;                           // 复位,避免残留已关闭的 fd
        return -1;
    }
    // 在锁文件里记录本进程 pid,方便排查占用者
    ftruncate(s_can_lock_fd, 0);                    // 清掉上个 pid 位数更多时的残留
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "pid=%d\n", getpid());
    if (write(s_can_lock_fd, buf, n) != n){
        printf("[CAN-BUS] 警告: 写入 pid 到锁文件失败(不影响锁功能)。\n");
    }

    // ── C侧电机日志(与python解耦): 拿到CAN锁后才创建,
    //    加锁失败的实例不会留下空日志文件 ──
    mc_log_open_default();
    mc_log("[INIT] motor_init success, can bus locked");

    struct sched_param param = { .sched_priority = 40 }; // 最高优先级
    sched_setscheduler(0, SCHED_RR, &param);

    if (HUMANOID_TYPE == 2){
        init_can();
        Encos_CANFD_startRun();
        encos_start_recv_thread();
        std::thread thread_convert(convert_motor_data);
        std::thread thread_tx(sendMotorCmd_);
        std::thread thread_rx(motorState_fd);
        thread_rx.detach();
        thread_tx.detach();
        thread_convert.detach();
        std::signal(SIGINT, signal_handler);

        // struct sigaction sa;
        // sa.sa_handler = signal_handler_sa;
        // sigemptyset(&sa.sa_mask);
        // sa.sa_flags = SA_RESTART;  // 重启被中断的系统调用
        // sigaction(SIGINT, &sa, nullptr);
    }
    return 0;
}

MotorCmd get_motor_cmd(void){
    MotorCmd cmd;
    return cmd;
}

void get_motor_data(MotorState* motor_state_output_){
    if (motor_state_output_ != nullptr) {  // 安全检查
        *motor_state_output_ = motor_state_converted;  // 解引用后赋值
    }
}

void convert_motor_data(){
    struct timespec start, end;
    struct timespec remaining;
    while(true){
    clock_gettime(CLOCK_MONOTONIC, &start);
    if (local_ankel_solver2 == true && local_waist_solver2 == true){
        // 正向解算：三个解算相互独立（分别读写不同的全局 last_* 状态），
        // 且单个解算已优化到微秒级。相比每个周期新建 3 个线程（约几十微秒
        // 的创建与调度开销），同步执行延迟更低、时序更确定，同时消除对
        // motor_state_output 的并发读竞争。
        double tr5, tp4, vr5, vp4, tq_r5, tq_p4;
        std::tie(tr5, tp4, vr5, vp4, tq_r5, tq_p4) = solver_rx.motorToJointLeft(
            -motor_state_output.q[4], -motor_state_output.q[5],
            -motor_state_output.qd[4], -motor_state_output.qd[5],
            -motor_state_output.tau[4], -motor_state_output.tau[5]
        );

        double tr11, tp10, vr11, vp10, tq_r11, tq_p10;
        std::tie(tr11, tp10, vr11, vp10, tq_r11, tq_p10) = solver_rx.motorToJointRight(
            -motor_state_output.q[10], -motor_state_output.q[11],
            -motor_state_output.qd[10], -motor_state_output.qd[11],
            -motor_state_output.tau[10], -motor_state_output.tau[11]
        );

        double tr14, tp13, vr14, vp13, tq_r14, tq_p13;
        // 注意:腰的两个电机角是交叉传入的(与 jointToMotorW 的索引对应关系保持一致)
        std::tie(tr14, tp13, vr14, vp13, tq_r14, tq_p13) = solver_rx.motorToJointW(
            -motor_state_output.q[14], -motor_state_output.q[13],
            -motor_state_output.qd[14], -motor_state_output.qd[13],
            -motor_state_output.tau[14], -motor_state_output.tau[13]
        );

        motor_state_converted.q[4] = tp4; // pitch urdf
        motor_state_converted.qd[4] = vp4;
        motor_state_converted.tau[4] = tq_p4;
        motor_state_converted.q[5] = tr5; // roll urdf
        motor_state_converted.qd[5] = vr5;
        motor_state_converted.tau[5] = tq_r5;

        motor_state_converted.q[10] = tp10;
        motor_state_converted.qd[10] = vp10;
        motor_state_converted.tau[10] = tq_p10;
        motor_state_converted.q[11] = tr11;
        motor_state_converted.qd[11] = vr11;
        motor_state_converted.tau[11] = tq_r11;

        motor_state_converted.q[14] = tp13; // roll urdf
        motor_state_converted.qd[14] = vp13;
        motor_state_converted.tau[14] = tq_p13;
        motor_state_converted.q[13] = tr14; // pitch urdf
        motor_state_converted.qd[13] = vr14;
        motor_state_converted.tau[13] = tq_r14;

        for (int i = 0; i < 29; ++i){
            if (i == 4 or i == 5 or i == 10 or i == 11 or i == 13 or i == 14){
                
            }
            else{
                motor_state_converted.q[i] = motor_state_output.q[i];
                motor_state_converted.qd[i] = motor_state_output.qd[i];
                motor_state_converted.tau[i] = motor_state_output.tau[i];
            }
        }
    }
    if (local_ankel_solver2 == false and local_waist_solver2 == false){
        for (int i = 0; i < 29; ++i){
            motor_state_converted.q[i] = motor_state_output.q[i];
            motor_state_converted.qd[i] = motor_state_output.qd[i];
            motor_state_converted.tau[i] = motor_state_output.tau[i];
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &end);

    long elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000L 
                    + (end.tv_nsec - start.tv_nsec);
    const long TARGET_NS = 1 * 1000000L; // 2ms = 2,000,000 ns
    
    if (elapsed_ns < TARGET_NS) {
        long sleep_ns = TARGET_NS - elapsed_ns;
        remaining.tv_sec = sleep_ns / 1000000000L;
        remaining.tv_nsec = sleep_ns % 1000000000L;
        nanosleep(&remaining, nullptr);
    }
}
}

int set_motor_cmd(MotorCmd *cmd){
    cmdCallback(cmd);
}

int set_motor_enable_cmd(int enable, int index){
    if(enable == false){
        motorBreak();
        return 0;
    }
    else if (enable == true)
    {
        return 0;
    }
}
