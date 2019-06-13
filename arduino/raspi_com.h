#ifndef RASPI_COM_H_
#define RASPI_COM_H_

#include <bluefruit.h>

void ExecUartCmd();

void ExecUartCmdCallback(uint8_t cmd, uint8_t a, int b);

#endif