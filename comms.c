#include "comms.h"

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
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

/**
 * Send a get message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 */
uint8_t sendGet(uint8_t addr, uint8_t pParMask) {
    // since get is 0 in msb of pParMask no need to do anything
    uint8_t msg[2] = { addr, pParMask };
    UARTSend(msg, 2);
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - float value to set
 */
uint8_t sendSetFloatPar(uint8_t addr, uint8_t pParMask, float pParVal) {
    uint8_t msgLen = 6; // 1 byte for addr, 1 for mask, 4 for val
    uint8_t msg[msgLen];
    msg[0] = addr;
    msg[1] = 0b10000000 | pParMask;

    union Flyte parVal;
    parVal.f = pParVal;
    int i;
    for(i = 0; i < 4; ++i)
        msg[i+2] = parVal.bytes[i];

    UARTSend(msg, msgLen);
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - byte value to set
 */
uint8_t sendSetBytePar(uint8_t addr, uint8_t pParMask, uint8_t pParVal) {
    uint8_t msgLen = 3; // 1 byte for addr, 1 for mask, 1 for val
    uint8_t msg[msgLen];
    msg[0] = addr;
    msg[1] = 0b10000000 | pParMask;
    msg[2] = pParVal;

    UARTSend(msg, msgLen);
}

/**
 * Send empty message on address BROADCAST_ADDR so avocados know to keep working.
 */
void heatbeat() { UARTSend((uint8_t[]) { BROADCAST_ADDR }, 1); }

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< MESSAGES >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

// <<<<<<< SET >>>>>>>

/**
 * Set address of any other devices on bus.
 *
 * Special case doesn't use a parameter mask byte in protocol structure
 *
 * @param addr - address to set on device
 */
uint8_t setAddress(uint8_t addr) {
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
    sendSetFloatPar(addr, (uint8_t) MaxCur, maxCurr);
}

/**
 * Set behavior to take in case of brain failure (identified through lack of
 * heartbeats).
 *
 * @param addr - address of actuator
 * @param eStopBehavior - bitmask indicating behavior to take in case of failure
 */
uint8_t setEStopBehavior(uint8_t addr, uint8_t eStopBehavior) {
    sendSetBytePar(addr, (uint8_t) EStop, eStopBehavior);
}

/**
 * Rotate given actuator to given position.
 *
 * @param addr - address of actuator
 * @param pos - angle to rotate actuator to (in radians)
 */
uint8_t rotateToPosition(uint8_t addr, float pos) {
    sendSetFloatPar(addr, (uint8_t) Pos, pos);
}

/**
 * Rotate given actuator at given velocity.
 *
 * @param addr - address of actuator
 * @param vel - velocity to rotate at (in rpm)
 */
uint8_t rotateAtVelocity(uint8_t addr, float vel) {
    sendSetFloatPar(addr, (uint8_t) Vel, vel);
}

/**
 * Rotate given actuator at given current.
 *
 * @param addr - address of actuator
 * @param cur - current to rotate at (in amps)
 */
uint8_t rotateAtCurrent(uint8_t addr, float cur) {
    sendSetFloatPar(addr, (uint8_t) Cur, cur);
}

// <<<<<<< GET >>>>>>>

/**
 * Get status of avocado.
 *
 * @param addr - address to set on device
 */
uint8_t getStatus(uint8_t addr) {
    sendGet(addr, (uint8_t) Status);
}

/**
 * Get current maximum current.
 *
 * @param addr - address of actuator
 */
uint8_t getMaxCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) MaxCur);
}

/**
 * Get current behavior to take in case of brain failure (identified through
 * lack of heartbeats).
 *
 * @param addr - address of actuator
 */
uint8_t getStopBehavior(uint8_t addr) {
    sendGet(addr, (uint8_t) EStop);
}

/**
 * Get current position.
 *
 * @param addr - address of actuator
 */
uint8_t getPosition(uint8_t addr) {
    sendGet(addr, (uint8_t) Pos);
}

/**
 * Get current velocity.
 *
 * @param addr - address of actuator
 */
uint8_t getVelocity(uint8_t addr) {
    sendGet(addr, (uint8_t) Vel);
}

/**
 * Get current current.
 *
 * @param addr - address of actuator
 */
uint8_t getCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) Cur);
}

/**
 * Get temperature of actuator.
 *
 * @param addr - address of actuator
 */
uint8_t getTemperature(uint8_t addr) {
    sendGet(addr, (uint8_t) Tmp);
}

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< UTILITIES >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

/**
 * Return string corresponding to given enum value.
 *
 * @param par - enum value whose name to return
 * @return the name of the enum value given
 */
const char* getParameterName(enum Parameter par) {
    switch(par) {
        case Adr: return "Adr";
        case Tmp: return "Tmp";
        case Cur: return "Cur";
        case Vel: return "Vel";
        case Pos: return "Pos";
        case MaxCur: return "MaxCur";
        case EStop: return "EStop";
        case Status: return "Status";
        default: return "NOP";
    }
}

/**
 * Return string corresponding to given enum value.
 *
 * @param par - enum value whose name to return
 * @return the name of the enum value given
 */
const char* getCommandName(enum Command cmd) {
    switch(cmd) {
        case Get: return "Get";
        case Set: return "Set";
        default: return "NOP";
    }
}

/**
 * Check if UART is ready to send.
 *
 * @return true if we are ready, false otherwise
 */
bool UARTReady() {
    return GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7) && !UARTBusy(UART7_BASE);
}

/**
 * Get address.
 *
 * @return our address
 */
uint8_t UARTGetAddress() { return BRAIN_ADDR; }

/**
 * Print a float.
 *
 * @param val - float to print
 * @param verbose - dictates how verbose to print
 */
void UARTPrintFloat(float val, bool verbose) {
    char str[50]; // pretty arbitrarily chosen
    sprintf(str, "%f", val);
    verbose
        ? UARTprintf("val, length: %s, %d\n", str, strlen(str))
        : UARTprintf("%s\n", str);
}

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< HANDLING >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

/**
 * Take actions on message as appropriate.
 *
 * If address does not match our own, bail out and send message on.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 * @param verbose - if true print to console for debugging
 * @param echo - if true simply echo the message, can also be helpful for debugging
 * @return if we successfully handled a message meant for us
 */
bool handleUART(char* buffer, uint32_t length, bool verbose, bool echo) {
    // CURRENTLY ONLY EVER ECHOING
    UARTSend((uint8_t *) buffer, length);
    int i;
    for (i = 0; i < length; ++i) {
        UARTprintf("Text[%d]: %s\n", i, buffer[i]);
    }
    return false;
}
