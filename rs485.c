#include "rs485.h"

uint8_t BRAIN_ADDRESS, BROADCASTADDR, ADDRSETADDR;
uint8_t ESTOP_HOLD      = 0b11111111;
uint8_t ESTOP_KILL      = 0b11111110;
uint8_t COMMAND_SUCCESS = 0b11111111;
uint8_t COMMAND_FAILURE = 0b11111101;
uint8_t OUTPUT_LIMITING = 0b11111111;
uint8_t OUTPUT_FREE     = 0b11111011;
uint8_t MAX_PARAMETER_VALUE = 0x9;

/**
 * Initializes clock and pins for RS485 communication
 *
 * @param g_ui32SysClock - clock
 */
void RSInit(uint32_t g_ui32SysClock) {
    // copy over clock created in main
    uartSysClock = g_ui32SysClock;
    // enable peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // enable GPIO port C pin 6 as RS485 transceiver rx/tx pin
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    // enable tied pin as input to read output of enable pin
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);
    // write transceiver enable pin low for listening
    UARTSetRead();
    // configure UART for `115,200`, 8-N-1 operation.
    ROM_UARTConfigSetExpClk(UART7_BASE, uartSysClock, 115200,
      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    // enable UART interrupt
    ROM_IntEnable(INT_UART7);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
    addrmask = 0b00001111;
    cmdmask = 0b10000000; // command selector bits (0 = get, 1 = set)
    parmask = 0b00000111; // parameter selector bits
    BRAIN_ADDRESS = 0x00;
    BROADCASTADDR = 0xFF;
    ADDRSETADDR = 0xFE;
    UARTprintf("RS485 initialized\n");
}

/**
 * Sends given string buffer over RS485
 *
 * @param pui8Buffer  - pointer to byte buffer to send over RS485
 * @param ui32Count   - length of buffer in bytes
 */
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count) {
    // TODO: implement construction of prefix bytes by ORing masks
    // TODO: create data flags avocados can flip for the brain

    // TODO: CRC need to include address?
    // add CRC byte to message
    uint8_t crc = crc8(0, pui8Buffer, ui32Count);
    // set transceiver rx/tx pin high to send
    UARTSetWrite();
    bool space = true;
    // loop while there are more bytes
    while(ui32Count--) {
        // write next byte to UART
        // putchar returns false if the send FIFO is full
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, *pui8Buffer);
        // if send FIFO is full, wait until we can put the char in
        while (!space) {
            space = ROM_UARTCharPutNonBlocking(UART7_BASE, *pui8Buffer);
        }
        *pui8Buffer++;
    }
    // send CRC for error-checking
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc);
    while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc); }

    // send stopbyte
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOPBYTE);
    while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOPBYTE); }
}

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< SENDS >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

/**
 * Send a get message
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 */
void sendGet(uint8_t addr, uint8_t pParMask) {
    // since get is 0 in msb of pParMask no need to do anything
    uint8_t msg[2] = { addr, pParMask };
    UARTSend(msg, 2);
}

/**
 * Send a set message
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - float value to set
 */
void sendSetFloatPar(uint8_t addr, uint8_t pParMask, float pParVal) {
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
 * Send a set message
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - byte value to set
 */
void sendSetBytePar(uint8_t addr, uint8_t pParMask, uint8_t pParVal) {
    uint8_t msgLen = 3; // 1 byte for addr, 1 for mask, 1 for val
    uint8_t msg[msgLen];
    msg[0] = addr;
    msg[1] = 0b10000000 | pParMask;
    msg[2] = pParVal;

    UARTSend(msg, msgLen);
}

/**
 * Sends empty message on address BROADCASTADDR so avocados know to keep working
 */
void heartBeat() { UARTSend((uint8_t[]) { BROADCASTADDR }, 1); }

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< MESSAGES >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

// <<<<<<< SET >>>>>>>

/**
 * Set address of any other devices on bus
 *
 * @param addr - address to set on device
 */
void setAddress(uint8_t addr) {
    sendSetBytePar(ADDRSETADDR, (uint8_t) Adr, addr);
}

/**
 * Set maximum current for actuator
 *
 * @param addr - address of actuator
 * @param maxCurr - maximum current in amps that actuator should throttle to
 */
void setMaxCurrent(uint8_t addr, float maxCurr) {
    sendSetFloatPar(addr, (uint8_t) MaxCur, maxCurr);
}

/**
 * Set behavior to take in case of brain failure (identified through lack of
 * heartbeats)
 *
 * @param addr - address of actuator
 * @param eStopBehavior - bitmask indicating behavior to take in case of failure
 */
void setEStopBehavior(uint8_t addr, uint8_t eStopBehavior) {
    sendSetBytePar(addr, (uint8_t) EStop, eStopBehavior);
}

/**
 * Rotate given actuator to given position
 *
 * @param addr - address of actuator
 * @param pos - angle to rotate actuator to (in radians)
 */
void rotateToPosition(uint8_t addr, float pos) {
    sendSetFloatPar(addr, (uint8_t) Pos, pos);
}

/**
 * Rotate given actuator at given velocity
 *
 * @param addr - address of actuator
 * @param vel - velocity to rotate at (in rpm)
 */
void rotateAtVelocity(uint8_t addr, float vel) {
    sendSetFloatPar(addr, (uint8_t) Vel, vel);
}

/**
 * Rotate given actuator at given current
 *
 * @param addr - address of actuator
 * @param cur - current to rotate at (in amps)
 */
void rotateAtCurrent(uint8_t addr, float cur) {
    sendSetFloatPar(addr, (uint8_t) Cur, cur);
}

// <<<<<<< GET >>>>>>>

/**
 * Get status of avocado
 *
 * @param addr - address to set on device
 */
void getStatus(uint8_t addr) {
    sendGet(addr, (uint8_t) Status);
}

/**
 * Get current maximum current
 *
 * @param addr - address of actuator
 */
void getMaxCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) MaxCur);
}

/**
 * Get current behavior to take in case of brain failure (identified through
 * lack of heartbeats)
 *
 * @param addr - address of actuator
 */
void getStopBehavior(uint8_t addr) {
    sendGet(addr, (uint8_t) EStop);
}

/**
 * Get current position
 *
 * @param addr - address of actuator
 */
void getPosition(uint8_t addr) {
    sendGet(addr, (uint8_t) Pos);
}

/**
 * Get current velocity
 *
 * @param addr - address of actuator
 */
void getVelocity(uint8_t addr) {
    sendGet(addr, (uint8_t) Vel);
}

/**
 * Get current current
 *
 * @param addr - address of actuator
 */
void getCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) Cur);
}

/**
 * Get temperature of actuator
 *
 * @param addr - address of actuator
 */
void getTemperature(uint8_t addr) {
    sendGet(addr, (uint8_t) Tmp);
}

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< UTILITIES >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

/**
 * Returns string corresponding to given enum value.
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
 * Returns string corresponding to given enum value.
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

// set transceiver rx/tx pin low for read mode
void UARTSetRead() { GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0); }

// set transceiver rx/tx pin low for read mode
void UARTSetWrite() { GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); }

/**
 * Gets address
 *
 * @return our address
 */
uint8_t UARTGetAddress() { return BRAIN_ADDRESS; }

/**
 * Prints a float
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

/**
 * UART interrupt handler, fires when character received.
 *
 * Currently blocks getting all characters.
 * TODO: asynchronously receive characters, add to buffer
 */
void UARTIntHandler(void) {
    // get interrupt status
    uint32_t ui32Status = ROM_UARTIntStatus(UART7_BASE, true);
    // clear asserted interrupts
    ROM_UARTIntClear(UART7_BASE, ui32Status);
    // initialize recv buffer
    char recv[10] = "";
    uint32_t ind = 0;
    char curr = ROM_UARTCharGet(UART7_BASE);
    // Loop while there are characters in the receive FIFO.
    while(curr != STOPBYTE && ind < 10) {
        recv[ind] = curr;
        ind++;
        curr = ROM_UARTCharGet(UART7_BASE);
    }

    // keep stop byte for tokenizing
    recv[ind] = curr;
    ind++;

    /*uint8_t crcin = recv[ind-2];
    if (crc8(0, (uint8_t *)recv, ind-2) != crcin){
        // ********** ERROR ***********
        // Handle corrupted message
    }*/

    // handle given message
    handleUART(recv, ind, true, true);
}
