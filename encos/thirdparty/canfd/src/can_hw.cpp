#include "can_hw.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>
#include <fcntl.h>
#include <unordered_map>
#include <array>

#include <boost/lockfree/spsc_queue.hpp>
#include <atomic>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "command.h"
#include "config.h"

extern EtherCAT_Msg Rx_Message_fd[6];

// Global variables
int bcm_sockets[6] = {-1, -1, -1, -1, -1, -1};  // BCM sockets for can0 to can3
int raw_sockets[6] = {-1, -1, -1, -1, -1, -1};  // RAW sockets for receiving on can0 to can3

// Per-interface data: current send data for each motor ID
std::vector<std::unordered_map<canid_t, std::vector<uint8_t>>> current_data(6);  // One per can0-3
std::mutex recv_mutex;  // Mutex for updating rv_motor_msg
std::atomic<bool> stop_receive(false);
std::vector<std::thread> receive_threads;

// Mapping: can_index (0-3) to motor IDs (1-15, grouped by 3 per bus except possibly last)
std::vector<std::vector<canid_t>> id_groups = {
    {20, 21, 22},     // can0
    {20, 21, 22},     // can1
    {20, 21, 22},     // can2
    {20, 21, 22},   // can3
    {20, 21, 22}
    // If can4 for 13-15, add {13,14,15} and extend arrays
};

long send_interval_us = 1000;

bool check_can_interface(const std::string& interface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        printf("Error creating socket for checking %s", interface.c_str());
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        printf("Error getting flags for %s", interface.c_str());
        close(sock);
        return false;
    }
    close(sock);

    if (ifr.ifr_flags & IFF_UP) {
        printf("%s is UP  ", interface.c_str());
        return true;
    } else {
        printf("%s is DOWN or not properly configured", interface.c_str());
        return false;
    }
}

int init_socketcan(const std::string& interface, int bitrate, int max_retries) {
    for (int attempt = 0; attempt < max_retries; ++attempt) {
        //printf("Attempt %d/%d: Setting up %s with bitrate %d", attempt + 1, max_retries, interface.c_str(), bitrate);

        system(("sudo ip link set " + interface + " down").c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // system("sudo ip link set can0 type can bitrate 1000000 berr-reporting off");
        int ret = system(("sudo ip link set " + interface + " up type can bitrate " + std::to_string(bitrate)).c_str());
        // int ret = system(("sudo ip link set " + interface + " up type can bitrate 1000000 sample-point 0.875 prop-seg 29 phase-seg1 30 phase-seg2 20 sjw 1 restart-ms 0 berr-reporting off").c_str());
        
        // int ret = system(("sudo ip link set " + interface + " up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.7 fd on").c_str());
        if (ret != 0) {
            printf("Attempt %d failed to set up %s", attempt + 1, interface.c_str());
            if (attempt == max_retries - 1) {
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        ret = system(("sudo ip link set " + interface + " txqueuelen 1000").c_str());
        if (ret != 0) {
            printf("Failed to set txqueuelen for %s", interface.c_str());
        }

        if (check_can_interface(interface)) {
            return 0;
        } else {
            printf("Attempt %d failed: %s is still DOWN", attempt + 1, interface.c_str());
            if (attempt == max_retries - 1) {
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    return -1;
}

void setup_bcm_periodic_send(int bcm_sock, canid_t can_id, const std::vector<uint8_t>& data) {
    struct bcm_msg_head msg_head = {};
    msg_head.opcode = TX_SETUP;
    msg_head.can_id = can_id;
    msg_head.flags = SETTIMER | STARTTIMER;
    msg_head.nframes = 1;
    msg_head.ival2.tv_sec = send_interval_us / 1000000;
    msg_head.ival2.tv_usec = send_interval_us % 1000000;

    struct can_frame cf = {};
    cf.can_id = can_id;
    cf.can_dlc = data.size();
    std::memcpy(cf.data, data.data(), data.size());

    char buf[sizeof(struct bcm_msg_head) + sizeof(struct can_frame)];
    std::memcpy(buf, &msg_head, sizeof(struct bcm_msg_head));
    std::memcpy(buf + sizeof(struct bcm_msg_head), &cf, sizeof(struct can_frame));

    if (write(bcm_sock, buf, sizeof(buf)) < 0) {
        printf("Error setting up BCM TX for ID %d: %s", can_id, strerror(errno));
    }
}

void update_bcm_periodic_data(int bcm_sock, canid_t can_id, const std::vector<uint8_t>& new_data) {
    // Delete existing TX setup if it exists
    struct bcm_msg_head delete_head = {};
    delete_head.opcode = TX_DELETE;
    delete_head.can_id = can_id;
    if (write(bcm_sock, &delete_head, sizeof(delete_head)) < 0) {
        // Ignore error if not exists, but log for debugging
        printf("Warning deleting BCM TX for ID %d: %s", can_id, strerror(errno));
    }

    // Setup new periodic send
    setup_bcm_periodic_send(bcm_sock, can_id, new_data);
}


using boost::lockfree::spsc_queue;
using boost::lockfree::capacity;
typedef std::shared_ptr<can_frame> can_frame_ptr;
spsc_queue<can_frame_ptr, capacity<2>> can_frames_queue[6];
// void receive_loop(int* raw_sock) {
//     struct can_frame frame[6];
//     while (!stop_receive) {
//         for (int channel = 0; channel < 6; ++channel){
//             int nbytes = recv(raw_sock[channel], &frame[channel], sizeof(frame[channel]), 0);
//             if (nbytes < 0) {
//                 if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
//                     // std::this_thread::sleep_for(std::chrono::microseconds(100));
//                     continue;
//                 }
//                 printf("Error receiving frame on can%d: %s", channel, strerror(errno));
//                 continue;
//             } 
//             else if (nbytes == sizeof(frame[channel])) {
//                 Rx_Message_fd[0].motor[channel].id = frame[channel].can_id;
//                 Rx_Message_fd[0].motor[channel].dlc = frame[channel].can_dlc;
//                 memcpy(Rx_Message_fd[0].motor[channel].data, frame[channel].data, sizeof(frame[channel].data));
//             }
//         }
//         EtherCAT_Msg_ptr can_msg_rx = std::make_shared<EtherCAT_Msg>(Rx_Message_fd[0]);
//         sendToQueue_fd_rx(0, can_msg_rx);
//         std::this_thread::sleep_for(std::chrono::microseconds(100));
//     }
// }

void receive_loop(int raw_sock, int can_index) {
    struct can_frame frame;
    can_frame_ptr frame_ptr;
    while (!stop_receive) {
        int nbytes = recv(raw_sock, &frame, sizeof(frame), 0);
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            printf("Error receiving frame on can%d: %s", can_index, strerror(errno));
            continue;
        } 
        else if (nbytes == sizeof(frame)) {
            Rx_Message_fd[can_index].motor[0].id = frame.can_id;
            Rx_Message_fd[can_index].motor[0].dlc = frame.can_dlc;
            memcpy(Rx_Message_fd[can_index].motor[0].data, frame.data, sizeof(frame.data));
            EtherCAT_Msg_ptr can_msg_rx = std::make_shared<EtherCAT_Msg>(Rx_Message_fd[can_index]);
            sendToQueue_fd_rx(can_index, can_msg_rx);
        }
    }
}

void clear_motor_error(int sock, int motor_id) {
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = 1;
    frame.data[0] = 0x0B;  // 清除错误命令

    if (sock < 0) {
        printf("Invalid socket for motor_id %d (clear error)", motor_id);
        return;
    }

    if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        printf("Failed to send clear error to motor ID %d", motor_id);
   }// else {
    //     printf("Sent clear error command 0B  %d", motor_id);
    // }


}

void send_fixed_command(int sock, int motor_id) {
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = 5;
    frame.data[0] = 0x44;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;

    if (sock < 0) {
        //
        printf("Invalid socket for motor_id %d", motor_id);
        return;
    }

    if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        printf("Failed to send fixed command to motor ID %d", motor_id);
    }
}

void sendFixedCommandAllMotors() {
    std::vector<std::thread> threads;
    for (int can_idx = 0; can_idx < 5; ++can_idx) {
        int sock = raw_sockets[can_idx];  // Use raw for one-time send
        for (auto id : id_groups[can_idx]) {
            threads.emplace_back(clear_motor_error, sock, id);
            if(id==20||id==21||id==22){threads.emplace_back(send_fixed_command, sock, id);}
        }
    }
    for (auto& t : threads) {
        t.join();
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool init_can() {
    std::vector<std::string> interfaces = {"can0", "can1", "can2", "can3", "can4", "can5"};
    int bitrate = 1000000;
    int max_retries = 3;

    for (size_t i = 0; i < interfaces.size(); ++i) {
        const auto& interface = interfaces[i];
        if (init_socketcan(interface, bitrate, max_retries) != 0) {
            printf("Failed to initialize %s", interface.c_str());
            return false;
        }

        // Create RAW socket first for sending fixed commands
        int raw_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (raw_sock < 0) {
            printf("Error creating RAW socket for %s", interface.c_str());
            return false;
        }

        // int loopback = 0;
        // //if (setsockopt(raw_sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        // if (setsockopt(raw_sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback)) < 0) {
        //     printf("Error disabling loopback for %s", interface.c_str());
        //     close(raw_sock);
        //     return false;
        // }
        int loopback = 0;
        int recv_own_msgs = 0;
        int enable_canfd = 1;

        // setsockopt(raw_sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
        // setsockopt(raw_sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
        // setsockopt(raw_sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface.c_str());
        if (ioctl(raw_sock, SIOCGIFINDEX, &ifr) < 0) {
            printf("Error getting ifindex for %s (RAW)", interface.c_str());
            close(raw_sock);
            return false;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(raw_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            printf("Error binding RAW socket for %s", interface.c_str());
            close(raw_sock);
            return false;
        }

        fcntl(raw_sock, F_SETFL, O_NONBLOCK);
        raw_sockets[i] = raw_sock;

        // Create BCM socket
        int bcm_sock = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
        if (bcm_sock < 0) {
            printf("Error creating BCM socket for %s", interface.c_str());
            close(raw_sock);
            return false;
        }

        std::strcpy(ifr.ifr_name, interface.c_str());
        if (ioctl(bcm_sock, SIOCGIFINDEX, &ifr) < 0) {
            printf("Error getting ifindex for %s (BCM)", interface.c_str());
            close(bcm_sock);
            close(raw_sock);
            return false;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (connect(bcm_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            printf("Error connecting BCM socket for %s", interface.c_str());
            close(bcm_sock);
            close(raw_sock);
            return false;
        }
        bcm_sockets[i] = bcm_sock;
    }

    // Send fixed commands after all sockets are ready
    // sendFixedCommandAllMotors();

    for (size_t i = 0; i < interfaces.size(); ++i) {
        // Start receive thread
        receive_threads.emplace_back(receive_loop, raw_sockets[i], i);
    }
    return true;
}

// void sendCanCommand(YKSMotorData *mot_data) {
    // for (int can_idx = 0; can_idx < 5; ++can_idx) {
    //     int bcm_sock = bcm_sockets[can_idx];
    //     auto& curr_map = current_data[can_idx];
    //     for (auto can_id : id_groups[can_idx]) {
    //         int motor_idx = can_id - 1;
    //         std::vector<uint8_t> new_data;
    //         send_motor_data_convert(new_data, can_id, mot_data[motor_idx].kp_, mot_data[motor_idx].kd_,
    //                                 mot_data[motor_idx].pos_des_, mot_data[motor_idx].vel_des_, mot_data[motor_idx].ff_);

    //         // Update if new or changed
    //         auto it = curr_map.find(can_id);
    //         if (it == curr_map.end() || new_data != it->second) {
    //             curr_map[can_id] = new_data;
    //             update_bcm_periodic_data(bcm_sock, can_id, new_data);
    //         }
    //     }
    // }
// }

void close_canDevice() {
    stop_receive = true;
    for (auto& th : receive_threads) {
        th.join();
    }

    for (int i = 0; i < 6; ++i) {
        if (bcm_sockets[i] >= 0) {
            // Delete BCM setups
            for (auto& pair : current_data[i]) {
                struct bcm_msg_head delete_head = {};
                delete_head.opcode = TX_DELETE;
                delete_head.can_id = pair.first;
                write(bcm_sockets[i], &delete_head, sizeof(delete_head));
            }
            close(bcm_sockets[i]);
        }
        if (raw_sockets[i] >= 0) {
            close(raw_sockets[i]);
        }
        system(("sudo ip link set can" + std::to_string(i) + " down").c_str());
    }
}