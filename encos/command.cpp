//
// Created by bismarck on 11/19/22.
//

#include "command.h"


void sendToQueue_fd_tx(int channel, const EtherCAT_Msg_ptr& msg) {
    if (messages_fd_tx[channel].write_available()) {
        messages_fd_tx[channel].push(msg);
        // printf("---------------test-queue-tx-------------\n");
    } else {

    }
}
void sendToQueue_fd_rx(int channel, const EtherCAT_Msg_ptr& msg) {
    if (messages_fd_rx[channel].write_available()) {
        messages_fd_rx[channel].push(msg);
        // printf("---------------test-queue-rx-------------\n");
    } else {

    }
}

