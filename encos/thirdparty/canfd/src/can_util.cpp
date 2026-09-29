#include "can_util.hpp"
#include "can_hw.hpp"

extern int raw_sockets[6];

int canfd_set_position(uint32_t id, float position)
{
    int32_t pos_target;
    pos_target = position * 262144 / 360.0;

    return canfd_send_cmd_with_data(id, 0x44, pos_target);
}

int canfd_send_cmd_with_data(uint32_t id, int32_t cmd, int32_t data)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 5;
    frame.data[0] = cmd;
    frame.data[1] = data & 0xFF;
    frame.data[2] = (data >> 8) & 0xFF;
    frame.data[3] = (data >> 16) & 0xFF;
    frame.data[4] = (data >> 24) & 0xFF;

    return write(raw_sockets[0], &frame, sizeof(frame));
}