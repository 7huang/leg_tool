#include "encos.hpp"
#include <thread>

static joint_data_t g_joint_data[50] = {{0,0,0}};
CANFD_Msg Rx_Message_fd_[1];

int encos_send_frames(uint32_t id, uint8_t *data, uint8_t len, uint8_t channel)
{
    //定义CAN帧
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    
    //定义CANFD帧
    struct canfd_frame frame_fd;
    frame_fd.can_id = id;
    frame_fd.len = len;
    frame_fd.flags = CANFD_BRS;
    for (int cnt = 0; cnt < len; cnt++){
        frame.data[cnt] = data[cnt];
    }
    // 设置模式时必须通过channel指定can总线
    if (id == 0x7FF){
        if (write(raw_sockets[channel], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
    }
    if ((id >= 16 && id <= 22)){
        if (write(raw_sockets[LF1_ARM_R], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
        // if (write(raw_sockets[0], &frame_fd, sizeof(frame_fd)) != sizeof(frame_fd)) {
        //     printf("Failed to send fixed command to motor ID %d \n", id);
        // }

    }
    else if ((id >= 23 && id <= 29)){
        if (write(raw_sockets[LF1_ARM_L], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
    }

    else if ((id >= 41 && id <= 43)){
        if (write(raw_sockets[LF1_WAIST], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
    }

    else if ((id >= 1 && id <= 6)){
        if (write(raw_sockets[LF1_LEG_L], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
    }

    else if ((id >= 7 && id <= 12)){
        if (write(raw_sockets[LF1_LEG_R], &frame, sizeof(frame)) != sizeof(frame)) {
            // printf("Failed to send fixed command to motor ID %d \n", id);
        }
    }

    // usleep(ENCOS_CAN_INTERVAL_US);
    return 0;
}

using boost::lockfree::spsc_queue;
using boost::lockfree::capacity;
typedef std::shared_ptr<can_frame> can_frame_ptr;
extern struct spsc_queue<can_frame_ptr, capacity<2>> can_frames_queue[6];

int encos_receive()
{
    while (1) {
        for (int i = 0; i < 6; ++i){
            if (can_frames_queue[i].empty()==true){
                Rx_Message_fd_[0].motor[i].id = 0;
                Rx_Message_fd_[0].motor[i].dlc = 0;
                continue;
            }
            struct can_frame frame;
            can_frame_ptr frame_ptr = can_frames_queue[i].front();
            frame = *frame_ptr;
            // &frame = *frame_ptr;
            can_frames_queue[i].pop();
            Rx_Message_fd_[0].motor[i].id = frame.can_id;
            Rx_Message_fd_[0].motor[i].dlc = frame.can_dlc;
            memcpy(Rx_Message_fd_[0].motor[i].data, frame.data, sizeof(frame.data));
            // printf("id = %d\n", Rx_Message_fd_[0].motor[i].id);
        }
        usleep(330);
    }

    return 0;
}

void *encos_rx_thread(void *param)
{
    printf("start rx_thread \n");
    encos_receive();
    return 0;
}

pthread_t tid_rx;
void encos_start_recv_thread(){
    //pthread_create(&tid_rx, NULL, encos_rx_thread, NULL);
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        close_canDevice();
        std::cout << "\n接收到Ctrl+C信号，程序正在退出..." << std::endl;
        usleep(10000);
        exit(1);
    }
}

volatile sig_atomic_t g_cleanup_in_progress = 0;
volatile sig_atomic_t g_should_exit = 0;

void signal_handler_sa(int sig) {
    if (g_cleanup_in_progress) {
        // 第二次收到信号，忽略或记录
        std::cout << "\n退出中，请稍候..." << std::endl;
        return;
    }
    
    g_cleanup_in_progress = 1;
    std::cout << "\n开始退出..." << std::endl;
    
    // 执行清理操作
    close_canDevice();
    // sleep(3);  // 模拟清理
    
    std::cout << "退出完成" << std::endl;
    g_should_exit = 1;
    sleep(1);
    exit(1);
}