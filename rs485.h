/*
 * rs485.h
 *
 *  Created on: Apr 21, 2018
 *      Author: Ryan
 */

#ifndef RS485_H_
#define RS485_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "crc.h"
#include "motor.h"
#include "isense.h"
#include "temp.h"

void RSInit(uint32_t);
void UARTIntHandler(void);
void UARTSend(const uint8_t*, uint32_t);
bool UARTReady(void);
void UARTSetRead(void);
void UARTSetWrite(void);
void UARTSetAddress(uint8_t);
uint8_t UARTGetAddress(void);

void UARTPrintFloat(float, bool);

uint32_t uartSysClock;
static uint8_t STOPBYTE = '!';
uint8_t cmdmask, parmask, heartmask, addrmask, posval, curval, velval, tempval;

union Flyte
{
  float f;
  uint8_t bytes[sizeof(float)];
};

enum Command {
    Get = 0,
    Set = 1
};

enum Parameter {
    Pos = 0,
    Vel = 1,
    Cur = 2,
    Tmp = 3
};

#endif /* RS485_H_ */
