#include "pace_stamp.h"

#include <mutex>
#include <time.h>

namespace {

constexpr int kJointNum = 29;

std::mutex g_lock[kJointNum];
PaceJointSample g_sample[kJointNum] = {};

// 与 math_ops.c 的 fd_id_2_index 映射相同, 但对 0 / 0x7FF 等无效 id 静默返回 -1,
// 避免在 1kHz 发送线程里刷屏
int id_to_index(uint32_t id){
    if (id >= 1 && id <= 12) return id - 1;
    if (id >= 16 && id <= 22) return id + 6;
    if (id >= 23 && id <= 29) return id - 8;
    if (id >= 41 && id <= 43) return id - 29;
    return -1;
}

}

int64_t pace_now_ns(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

void pace_record_rx(int index, float q, float qd, float tau, uint8_t temperature, uint8_t error){
    if (index < 0 || index >= kJointNum){
        return;
    }
    const int64_t now = pace_now_ns(); // 先取时间再加锁, 等锁时间不计入时间戳
    std::lock_guard<std::mutex> guard(g_lock[index]);
    PaceJointSample &s = g_sample[index];
    s.q = q;
    s.qd = qd;
    s.tau = tau;
    s.temperature = temperature;
    s.error = error;
    s.rx_ns = now;
    s.rx_count += 1;
}

void pace_record_tx(uint32_t motor_id){
    const int index = id_to_index(motor_id);
    if (index < 0){
        return;
    }
    const int64_t now = pace_now_ns();
    std::lock_guard<std::mutex> guard(g_lock[index]);
    g_sample[index].tx_ns = now;
    g_sample[index].tx_count += 1;
}

int pace_get_joint_sample(int index, PaceJointSample *out){
    if (index < 0 || index >= kJointNum || out == nullptr){
        return -1;
    }
    std::lock_guard<std::mutex> guard(g_lock[index]);
    *out = g_sample[index];
    return 0;
}
