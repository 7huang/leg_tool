#pragma once
#include <stdint.h>
#include <unistd.h>

int canfd_set_position(uint32_t id, float position);
int canfd_send_cmd_with_data(uint32_t id, int32_t cmd, int32_t data);