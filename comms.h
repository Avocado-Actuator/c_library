#ifndef COMMS_H_
#define COMMS_H_

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
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#include "crc.h"

void CommsInit(uint32_t);
void ConsoleInit(void);
void ConsoleIntHandler(void);
void UARTIntHandler(void);
void UARTSend(const uint8_t*, uint32_t);
void UARTPrintFloat(float, bool);
bool handleUART(uint8_t*, uint32_t, bool, bool);

// C Library Functions

void heartbeat(void);

// <<<< set >>>>

// logistics
uint8_t setAddress(uint8_t);
uint8_t setMaxCurrent(uint8_t, float);
uint8_t setEStopBehavior(uint8_t, uint8_t);

// movement
uint8_t rotateToPosition(uint8_t, float);
uint8_t rotateAtVelocity(uint8_t, float);
uint8_t rotateAtCurrent(uint8_t, float);

// <<<< get >>>>

// logistics
uint8_t getStatus(uint8_t);
uint8_t getMaxCurrent(uint8_t);
uint8_t getStopBehavior(uint8_t);

// sensors
uint8_t getPosition(uint8_t);
uint8_t getVelocity(uint8_t);
uint8_t getCurrent(uint8_t);
uint8_t getTemperature(uint8_t);

// <<<< data >>>>

uint8_t sendMsgFlag, recvIndex, STOP_BYTE, MAX_PAR_VAL, CMD_MASK, PAR_MASK;
uint32_t uartSysClock, response_index;

// data structures
union Flyte {
  float f;
  uint8_t bytes[sizeof(float)];
};

union Flyte response_buffer[256];

enum Command {
    Get = 0,
    Set = 1
};

enum Parameter {
    Adr     = 0,
    Tmp     = 1,
    Cur     = 2,
    Vel     = 3,
    Pos     = 4,
    MaxCur  = 5,
    EStop   = 6,
    Status  = 7
};

#endif /* COMMS_H_ */
