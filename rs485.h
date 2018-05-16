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

void RSInit(uint32_t);
void UARTIntHandler(void);
void UARTSend(uint8_t, const uint8_t*, uint32_t);
bool UARTReady(void);
void UARTSetRead(void);
void UARTSetWrite(void);
void UARTSetAddress(uint8_t);
void UARTPrintFloat(float, bool);
uint8_t UARTGetAddress(void);

// message functions
// in future message functions
// should return something actionable
// for now just print out an echo

// <<<< set >>>>

// logistics
void setAddress(float);
void setMaxCurrent(uint8_t, float);
void setStopBehavior(uint8_t, float);

// movement
void rotateToPosition(uint8_t, float);
void rotateAtVelocity(uint8_t, float);
void rotateAtCurrent(uint8_t, float);

// <<<< get >>>>

// logistics
void getMaxCurrent(uint8_t);
void getStopBehavior(uint8_t);
void getStatus(uint8_t);

// sensors
void getPosition(uint8_t);
void getVelocity(uint8_t);
void getCurrent(uint8_t);
void getTemperature(uint8_t);

// <<<< data >>>>

uint32_t uartSysClock;
static uint8_t STOPBYTE = '!';
uint8_t cmdmask, parmask, heartmask, addrmask, posval, curval, velval, tempval;

union Flyte {
  float f;
  uint8_t bytes[sizeof(float)];
};

enum Command {
    Get = 0,
    Set = 1
};

enum Parameter {
    Add     = 0,
    Tmp     = 1,
    Cur     = 2,
    Vel     = 3,
    Pos     = 4,
    MaxCur  = 5,
    EStop   = 6,
    Status  = 7
};

#endif /* RS485_H_ */
