//
// Created by bismarck on 11/18/22.
//

#ifndef LEG_TOOL_QUEUE_H
#define LEG_TOOL_QUEUE_H

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include "transmit_fd.h"
#include "config.h"

using boost::lockfree::spsc_queue;
using boost::lockfree::capacity;
typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;

extern spsc_queue<EtherCAT_Msg_ptr, capacity<1>> messages_fd_tx[CHANNEL_NUMBER];
extern spsc_queue<EtherCAT_Msg_ptr, capacity<2>> messages_fd_rx[CHANNEL_NUMBER];

extern std::atomic<bool> running;
extern std::thread runThread;

#endif //LEG_TOOL_QUEUE_H
