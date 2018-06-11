#include "avocomms.h"

// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< BEGIN COMMUNICATION >>>>>>>>>>
// <<<<<<<<<<<<<<<< (comms.c/h) >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>

uint8_t recv[10];
uint8_t msgID, ADDR, BRAIN_ADDR, BROADCAST_ADDR, ADDR_SET_ADDR;
uint8_t ESTOP_HOLD      = 0b11111111;
uint8_t ESTOP_KILL      = 0b11111110;
uint8_t COMMAND_SUCCESS = 0b11111111;
uint8_t COMMAND_FAILURE = 0b11111101;
uint8_t OUTPUT_LIMITING = 0b11111111;
uint8_t OUTPUT_FREE     = 0b11111011;
uint8_t MAX_PARAMETER_VALUE = 0x9;

union Flyte holderFlyte;

// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
// <<<<<<<<<<<< INITS >>>>>>>>>>
// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>

/**
 * Initialize clock and pins for communication.
 *
 * @param g_ui32SysClock - clock
 */
void CommsInit(uint32_t g_ui32SysClock) {
    // copy over clock created in main
    uartSysClock = g_ui32SysClock;
    // enable peripherals used
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // enable GPIO port C pin 6 as the RS-485 transceiver rx/tx pin
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    // enable tied pin as input to read output of enable pin
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);
    // configure UART for 115,200, 8-N-1 operation
    ROM_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    // enable UART interrupt
    ROM_IntEnable(INT_UART7);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    response_index = 0;
    msgID = 0;
    recvIndex = 0;
    STOP_BYTE = '!';

    BRAIN_ADDR = 0x0;
    ADDR = 0x1;
    BROADCAST_ADDR = 0xFF;
    ADDR_SET_ADDR = 0xFE;

    CMD_MASK = 0b10000000; // 1 is SET and 0 is GET
    PAR_MASK = 0b00000111; // gives just parameter selector bits

    holderFlyte.f = NULL;
    int i;
    for(i = 0; i < 256; ++i) response_buffer[i] = holderFlyte;

    UARTprintf("Communication initialized\n");
}

/**
 * Initializes UART0 for console output using UARTStdio.
 */
void ConsoleInit(void) {
    // enable GPIO port A used for UART0 pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // configure pin muxing for UART0 functions on port A0 and A1
    // this step is not necessary if your part does not support pin muxing
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // enable UART0 so that we can configure the clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // use internal 16MHz oscillator as the UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // select alternate (UART) function for these pins
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // initialize UART for console I/O
    UARTStdioConfig(0, 9600, 16000000);

    // enable UART interrupt
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTprintf("Console initialized\n");
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< HANDLERS >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Console interrupt handler, fires when character received.
 */
void ConsoleIntHandler(void) {
    // get interrupt status
    uint32_t ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    // clear asserted interrupts
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // loop while characters in the receive FIFO
    while(ROM_UARTCharsAvail(UART0_BASE)) {
        // read next character from UART and write it back to the UART
        ROM_UARTCharPutNonBlocking(UART7_BASE, ROM_UARTCharGetNonBlocking(UART0_BASE));
    }
}

/**
 * UART interrupt handler, fires when character received.
 */
void UARTIntHandler(void) {
    // get interrrupt status
    uint32_t ui32Status = ROM_UARTIntStatus(UART7_BASE, true);
    // clear asserted interrupts
    ROM_UARTIntClear(UART7_BASE, ui32Status);
    // so we can't be interrupted by another character arriving
    ROM_UARTIntDisable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    uint8_t character = ROM_UARTCharGetNonBlocking(UART7_BASE);
    recv[recvIndex++] = character;

    if(character == STOP_BYTE) {
        handleUART(recv, recvIndex, true, true);
        recvIndex = 0;
    }
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<< UTILITIES >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Prints given float
 *
 * @param val - float to print
 * @param verbose - how descriptive to be in printing
 */
void UARTPrintFloat(float val, bool verbose) {
    char str[100]; // pretty arbitrarily chosen
    sprintf(str, "%f", val);
    verbose
        ? UARTprintf("val, length: %s, %d\n", str, strlen(str))
        : UARTprintf("%s\n", str);
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<< MESSAGE HANDLING >>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Takes actions on message as appropriate.
 *
 * If address does not match our own, bail out and send message on.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 * @param verbose - if true print to console for debugging
 * @param echo - if true simply echo the message, can also be helpful for debugging
 * @return if we successfully handled a message meant for us
 */
bool handleUART(uint8_t* buffer, uint32_t length, bool verbose, bool echo) {
    int i;
    if(verbose) {
        UARTprintf("*************************************************\n");
        UARTprintf("Address: %x\n", buffer[0]);
        UARTprintf("Message ID: %d\n", buffer[1]);
        if(length == 5) { UARTprintf("Value: %x\n", buffer[2]); }
        else if(length == 8) {
            union Flyte val;
            for(i = 0; i < 4; ++i) val.bytes[i] = buffer[i+2];
            UARTprintf("Value: ");
            UARTPrintFloat(val.f, false);
        } else {
            UARTprintf("Message:\n");
            for(i = 0; i < length; ++i) UARTprintf("[%d]: %x\n", i, buffer[i]);
            return false;
        }
    }

    uint8_t crcin = buffer[length-2];
    if(crc8(0, (uint8_t *)buffer, length-2) != crcin) {
        // handle corrupted message
        UARTprintf("Corrupted message, panic!\n");
        return false;
    } else if(buffer[0] != BRAIN_ADDR) {
        UARTprintf("Not my address, abort\n");
        return false;
    } else if(length == 5) {
        holderFlyte.bytes[0] = buffer[2];
        response_buffer[buffer[1]] = holderFlyte;
    } else if(length == 8) {
        for (i = 0; i < 4; ++i) { holderFlyte.bytes[i] = buffer[i+2]; }
        response_buffer[buffer[1]] = holderFlyte;
    } else {
        UARTprintf("Invalid message, panic!\n");
        return false;
    }

    return true;
}

/**
 * Send message to UART connection.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 */
void UARTSend(const uint8_t *buffer, uint32_t length) {
    int i;
    UARTprintf("Sending: ");
    for (i = 0; i < length; ++i) { UARTprintf("%x ", buffer[i]); }
    // so we can't be interrupted by the heartbeat trying to send
    ROM_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Add CRC byte to message
    uint8_t crc = crc8(0, (const unsigned char*) buffer, length);
    UARTprintf("%x ", crc);
    UARTprintf("%x\n", '!');
    bool space;
    for (i = 0; i < length; ++i) {
        // write the next character to the UART.
        // putchar returns false if the send FIFO is full
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, buffer[i]);
        // if send FIFO is full, wait until we can put the char in
        while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, buffer[i]); }
    }

    space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc);
    // if send FIFO is full, wait until we can put the char in
    while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc); }
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOP_BYTE);
    // if send FIFO is full, wait until we can put the char in
    while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOP_BYTE); }

    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<< C LIBRARY >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

// <<<<<<<<<<<<< SENDS >>>>>>>>>>>>>

/**
 * Send empty message on address BROADCAST_ADDR so avocados know to keep working.
 */
void heartbeat() { UARTSend((uint8_t[]) { BROADCAST_ADDR }, 1); /*UARTprintf("%d*\n", heartbeat_counter);*/ }

/**
 * Send a get message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 */
uint8_t sendGet(uint8_t addr, uint8_t pParMask) {
    // since get is 0 in msb of pParMask no need to do anything
    uint8_t msg[3] = { addr, msgID, pParMask };
    UARTSend(msg, 3);
    return msgID++;
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - float value to set
 */
uint8_t sendSetFloatPar(uint8_t addr, uint8_t pParMask, float pParVal) {
    // 1 byte for addr, 1 byte for ID, 1 for mask, 4 for val
    uint8_t msg[7];
    msg[0] = addr;
    msg[1] = msgID;
    msg[2] = 0b10000000 | pParMask;

    union Flyte parVal;
    parVal.f = pParVal;
    int i;
    for(i = 0; i < 4; ++i)
        msg[i+3] = parVal.bytes[i];

    UARTSend(msg, 7);
    return msgID++;
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - byte value to set
 */
uint8_t sendSetBytePar(uint8_t addr, uint8_t pParMask, uint8_t pParVal) {
    // 1 byte for addr, 1 for ID, 1 for mask, 1 for val
    uint8_t msg[4];
    msg[0] = addr;
    msg[1] = msgID;
    msg[2] = 0b10000000 | pParMask;
    msg[3] = pParVal;
    UARTSend(msg, 4);
    return msgID++;
}

// <<<<<<<<<<<<< MESSAGES >>>>>>>>>>>>>

// <<<<<<< SET >>>>>>>

/**
 * Set address of any other devices on bus.
 *
 * @param addr - address to set on device
 */
uint8_t setAddress(uint8_t addr) {
    // special case doesn't use a parameter mask byte in protocol structure
    uint8_t msg[3];
    msg[0] = ADDR_SET_ADDR;
    msg[1] = msgID;
    msg[2] = addr;
    UARTSend(msg, 3);
    return msgID++;
}

/**
 * Set maximum current for actuator.
 *
 * @param addr - address of actuator
 * @param maxCurr - maximum current in amps that actuator should throttle to
 */
uint8_t setMaxCurrent(uint8_t addr, float maxCurr) {
    return sendSetFloatPar(addr, (uint8_t) MaxCur, maxCurr);
}

/**
 * Set behavior to take in case of brain failure (identified through lack of
 * heartbeats).
 *
 * @param addr - address of actuator
 * @param eStopBehavior - bitmask indicating behavior to take in case of failure. 1 = hold position, 0 = kill motor power
 */
uint8_t setEStopBehavior(uint8_t addr, uint8_t eStopBehavior) {
    return sendSetBytePar(addr, (uint8_t) EStop, eStopBehavior);
}

/**
 * Rotate given actuator to given position.
 *
 * @param addr - address of actuator
 * @param pos - angle to rotate actuator to (in radians)
 */
uint8_t rotateToPosition(uint8_t addr, float pos) {
    return sendSetFloatPar(addr, (uint8_t) Pos, pos);
}

/**
 * Rotate given actuator at given velocity.
 *
 * @param addr - address of actuator
 * @param vel - velocity to rotate at (in rpm)
 */
uint8_t rotateAtVelocity(uint8_t addr, float vel) {
    return sendSetFloatPar(addr, (uint8_t) Vel, vel);
}

/**
 * Rotate given actuator at given current.
 *
 * @param addr - address of actuator
 * @param cur - current to rotate at (in amps)
 */
uint8_t rotateAtCurrent(uint8_t addr, float cur) {
    return sendSetFloatPar(addr, (uint8_t) Cur, cur);
}

// <<<<<<< GET >>>>>>>

/**
 * Get status of avocado.
 *
 * @param addr - address to set on device
 */
uint8_t getStatus(uint8_t addr) {
    return sendGet(addr, (uint8_t) Status);
}

/**
 * Get current maximum current.
 *
 * @param addr - address of actuator
 */
uint8_t getMaxCurrent(uint8_t addr) {
    return sendGet(addr, (uint8_t) MaxCur);
}

/**
 * Get current behavior to take in case of brain failure (identified through
 * lack of heartbeats).
 *
 * @param addr - address of actuator
 */
uint8_t getStopBehavior(uint8_t addr) {
    return sendGet(addr, (uint8_t) EStop);
}

/**
 * Get current position.
 *
 * @param addr - address of actuator
 */
uint8_t getPosition(uint8_t addr) {
    return sendGet(addr, (uint8_t) Pos);
}

/**
 * Get current velocity.
 *
 * @param addr - address of actuator
 */
uint8_t getVelocity(uint8_t addr) {
    return sendGet(addr, (uint8_t) Vel);
}

/**
 * Get current current.
 *
 * @param addr - address of actuator
 */
uint8_t getCurrent(uint8_t addr) {
    return sendGet(addr, (uint8_t) Cur);
}

/**
 * Get temperature of actuator.
 *
 * @param addr - address of actuator
 */
uint8_t getTemperature(uint8_t addr) {
    return sendGet(addr, (uint8_t) Tmp);
}

// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< END COMMUNICATION >>>>>>>>>>>
// <<<<<<<<<<<<<<<< (comms.c/h) >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>


// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<< BEGIN CRC >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<< (crc.c/h) >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>

/*
 * Author: Mark Adler
 * https://stackoverflow.com/questions/15169387/definitive-crc-for-c
 */

// 8-bit CRC using the polynomial x^8+x^6+x^3+x^2+1, 0x14D.
// Chosen based on Koopman, et al. (0xA6 in his notation = 0x14D >> 1):
// http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//
// This implementation is reflected, processing the least-significant bit of the
// input first, has an initial CRC register value of 0xff, and exclusive-or's
// the final register value with 0xff. As a result the CRC of an empty string,
// and therefore the initial CRC value, is zero.
//
// The standard description of this CRC is:
// width=8 poly=0x4d init=0xff refin=true refout=true xorout=0xff check=0xd8
// name="CRC-8/KOOP"

static const uint8_t crc8_table[256] = {
    0xea, 0xd4, 0x96, 0xa8, 0x12, 0x2c, 0x6e, 0x50, 0x7f, 0x41, 0x03, 0x3d,
    0x87, 0xb9, 0xfb, 0xc5, 0xa5, 0x9b, 0xd9, 0xe7, 0x5d, 0x63, 0x21, 0x1f,
    0x30, 0x0e, 0x4c, 0x72, 0xc8, 0xf6, 0xb4, 0x8a, 0x74, 0x4a, 0x08, 0x36,
    0x8c, 0xb2, 0xf0, 0xce, 0xe1, 0xdf, 0x9d, 0xa3, 0x19, 0x27, 0x65, 0x5b,
    0x3b, 0x05, 0x47, 0x79, 0xc3, 0xfd, 0xbf, 0x81, 0xae, 0x90, 0xd2, 0xec,
    0x56, 0x68, 0x2a, 0x14, 0xb3, 0x8d, 0xcf, 0xf1, 0x4b, 0x75, 0x37, 0x09,
    0x26, 0x18, 0x5a, 0x64, 0xde, 0xe0, 0xa2, 0x9c, 0xfc, 0xc2, 0x80, 0xbe,
    0x04, 0x3a, 0x78, 0x46, 0x69, 0x57, 0x15, 0x2b, 0x91, 0xaf, 0xed, 0xd3,
    0x2d, 0x13, 0x51, 0x6f, 0xd5, 0xeb, 0xa9, 0x97, 0xb8, 0x86, 0xc4, 0xfa,
    0x40, 0x7e, 0x3c, 0x02, 0x62, 0x5c, 0x1e, 0x20, 0x9a, 0xa4, 0xe6, 0xd8,
    0xf7, 0xc9, 0x8b, 0xb5, 0x0f, 0x31, 0x73, 0x4d, 0x58, 0x66, 0x24, 0x1a,
    0xa0, 0x9e, 0xdc, 0xe2, 0xcd, 0xf3, 0xb1, 0x8f, 0x35, 0x0b, 0x49, 0x77,
    0x17, 0x29, 0x6b, 0x55, 0xef, 0xd1, 0x93, 0xad, 0x82, 0xbc, 0xfe, 0xc0,
    0x7a, 0x44, 0x06, 0x38, 0xc6, 0xf8, 0xba, 0x84, 0x3e, 0x00, 0x42, 0x7c,
    0x53, 0x6d, 0x2f, 0x11, 0xab, 0x95, 0xd7, 0xe9, 0x89, 0xb7, 0xf5, 0xcb,
    0x71, 0x4f, 0x0d, 0x33, 0x1c, 0x22, 0x60, 0x5e, 0xe4, 0xda, 0x98, 0xa6,
    0x01, 0x3f, 0x7d, 0x43, 0xf9, 0xc7, 0x85, 0xbb, 0x94, 0xaa, 0xe8, 0xd6,
    0x6c, 0x52, 0x10, 0x2e, 0x4e, 0x70, 0x32, 0x0c, 0xb6, 0x88, 0xca, 0xf4,
    0xdb, 0xe5, 0xa7, 0x99, 0x23, 0x1d, 0x5f, 0x61, 0x9f, 0xa1, 0xe3, 0xdd,
    0x67, 0x59, 0x1b, 0x25, 0x0a, 0x34, 0x76, 0x48, 0xf2, 0xcc, 0x8e, 0xb0,
    0xd0, 0xee, 0xac, 0x92, 0x28, 0x16, 0x54, 0x6a, 0x45, 0x7b, 0x39, 0x07,
    0xbd, 0x83, 0xc1, 0xff};

// Return the CRC-8 of data[0..len-1] applied to the seed crc. This permits the
// calculation of a CRC a chunk at a time, using the previously returned value
// for the next seed. If data is NULL, then return the initial seed. See the
// test code for an example of the proper usage.
unsigned crc8(unsigned crc, unsigned char const *data, size_t len) {
    if(data == NULL) return 0;

    crc &= 0xff;
    unsigned char const *end = data + len;
    while (data < end) crc = crc8_table[crc ^ *data++];
    return crc;
}

// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<< END CRC >>>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<< (crc.c/h) >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>

// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<< BEGIN TIMER >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<< (timer.c/h) >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>

uint32_t TIME, HEARTBEAT_TIME, UART_SYS_CLOCK;

void TimerInit(uint32_t systemClock) {
    UART_SYS_CLOCK = systemClock;
    // enable the timer peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // configure 32-bit periodic timers.
    // 1ms timer
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // trigger every 1ms, 1000Hz
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, UART_SYS_CLOCK/1000);
    // setup interrupts for timer timeouts.
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // enable timers
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

    TIME = 0;
    HEARTBEAT_TIME = 0;
    // sendMsgFlag = 0;

    UARTprintf("Communication initialized\n");
}


void Timer0IntHandler(void) {
    // clear timer interrupt
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // update interrupt status
    // ++TIME;
    // send message every 200ms
    // if(TIME % 1000 == 0) { sendMsgFlag = 1; }

    // island time except for heartbeats
    ++HEARTBEAT_TIME;
    // send heartbeat every 300 ms, ~ twice as fast as they're expected
    if(HEARTBEAT_TIME % 300 == 0) heartbeat();
}

// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<< END TIMER >>>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<< (timer.c/h) >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>
