#pragma once
// #include "transmit.h"
#include "encos.hpp"

#define CHANNEL_NUMBER 6

void Encos_CANFD_Data_Get();

void Encos_CANFD_Command_Set();

void Encos_CANFD_tx();

void Encos_CANFD_rx();

void Encos_CANFD_startRun();