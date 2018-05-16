#include "rs485.h"

uint8_t BRAIN_ADDRESS = 0x00;

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
    cmdmask = 0b00010000; // when you filter message with this, 1 is SET and 0 is GET
    parmask = 0b01100000; // just parameter selector bits
    heartmask = 0b10000000;
    posval = 0b00000000;  // 00 selector bits
    curval = 0b00100000;  // 01 selector bits
    velval = 0b01000000;  // 10 selector bits
    tempval = 0b01100000; // 11 selector bits
    UARTprintf("RS485 initialized\n");
}

/**
 * Sends given string buffer over RS485
 *
 * @param pui8Buffer  - pointer to byte buffer to send over RS485
 * @param ui32Count   - length of buffer in bytes
 */
void UARTSend(uint8_t address, const uint8_t *pui8Buffer, uint32_t ui32Count) {
    // TODO: implement construction of prefix bytes by ORing masks
    // TODO: create data flags avocados can flip for the brain

    // TODO: CRC need to include address?
    // add CRC byte to message
    uint8_t crc = crc8(0, pui8Buffer, ui32Count);
    // set transceiver rx/tx pin high to send
    UARTSetWrite();
    bool space = true;
    // send the prefix w/address and flags
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, address);
    while(!space) { space = ROM_UARTCharPutNonBlocking(UART7_BASE, address); }
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

/**
 * Sends empty message on address `11111111` so avocados know to keep working
 */
void heartBeat() { UARTSend(0xFF, NULL, 0); }

// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>
// <<<<<<<<<<<<< MESSAGES >>>>>>>>>>>>>
// <<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>

// <<<<<<< SET >>>>>>>

/**
 * Set address of any other devices on bus
 *
 * @param addr - address to set on device
 */
void setAddress(uint8_t addr) {}

/**
 * Set maximum current for actuator
 *
 * @param addr - address of actuator
 * @param maxCurr - maximum current in amps that actuator should throttle to
 */
void setMaxCurrent(uint8_t addr, float maxCurr) {}

/**
 * Set behavior to take in case of brain failure (identified through lack of
 * heartbeats)
 *
 * @param addr - address of actuator
 * @param eStopBehavior - bitmask indicating behavior to take in case of failure
 */
void setStopBehavior(uint8_t addr, float eStopBehavior) {}

/**
 * Rotate given actuator to given position
 *
 * @param addr - address of actuator
 * @param pos - angle to rotate actuator to (in radians)
 */
void rotateToPosition(uint8_t addr, float pos) {}

/**
 * Rotate given actuator at given velocity
 *
 * @param addr - address of actuator
 * @param vel - velocity to rotate at (in rpm)
 */
void rotateAtVelocity(uint8_t addr, float vel) {}

/**
 * Rotate given actuator at given current
 *
 * @param addr - address of actuator
 * @param cur - current to rotate at (in amps)
 */
void rotateAtCurrent(uint8_t addr, float cur) {}

// <<<<<<< GET >>>>>>>

/**
 * Get status of avocado
 *
 * @param addr - address to set on device
 */
void getStatus(uint8_t addr) {}

/**
 * Get current maximum current
 *
 * @param addr - address of actuator
 */
void getMaxCurrent(uint8_t addr) {}

/**
 * Get current behavior to take in case of brain failure (identified through
 * lack of heartbeats)
 *
 * @param addr - address of actuator
 */
void getStopBehavior(uint8_t addr) {}

/**
 * Get current position
 *
 * @param addr - address of actuator
 */
void getPosition(uint8_t addr) {}

/**
 * Get current velocity
 *
 * @param addr - address of actuator
 */
void getVelocity(uint8_t addr) {}

/**
 * Get current current
 *
 * @param addr - address of actuator
 */
void getCurrent(uint8_t addr) {}

/**
 * Get temperature of actuator
 *
 * @param addr - address of actuator
 */
void getTemperature(uint8_t addr) {}

/**
 * Returns string corresponding to given enum value.
 *
 * @param par - enum value whose name to return
 * @return the name of the enum value given
 */
const char* getParameterName(enum Parameter par) {
    switch(par) {
        case Pos: return "Pos";
        case Vel: return "Vel";
        case Cur: return "Cur";
        case Tmp: return "Tmp";
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
bool handleUART(char* buffer, uint32_t length, bool verbose, bool echo) {
    if(echo) {
        // UARTSend((uint8_t *)buffer, length);
        int i;
        for(i = 0; i < length; ++i) {
            UARTprintf("Text[%d]: %s\n", i, buffer[i]);
        }
    } else {
        UARTprintf("\n\nText: %s\n\n", buffer);

        // get address
        char tempaddr = buffer[0] & addrmask;
        if(verbose) UARTprintf("Address: %d\n", tempaddr);

        if(tempaddr != UARTGetAddress()) {
            if(verbose) UARTprintf("Not my address, abort");
            return true; // changed to return true so that an error response is not generated
        }

        if(buffer[0] & heartmask != 0) {
            // called for heartbeat messages, where brain sends just address and 0's in prefix byte
            heartBeat();
            return true;
        }

        enum Command type = buffer[0] & cmdmask ? Set : Get;
        if(verbose) UARTprintf("Command: %s\n", getCommandName(type));

        // get parameter - { pos, vel, cur }
        enum Parameter par;
        uint8_t selector = buffer[0] & parmask;
        if(selector == posval) par = Pos;
        else if(selector == velval) par = Vel;
        else if(selector == curval) par = Cur;
        else if(selector == tempval) par = Tmp;
        else {
            if(verbose) UARTprintf("No parameter specified, abort");
            return false;
        }

        if(verbose) UARTprintf("Parameter: %s\n", getParameterName(par));

        if(type == Set) {
            if(length < 7){
                if(verbose) UARTprintf("No value provided, abort\n");
                return false;
            }
            // if set command then get parameter value to set to
            // since the first byte is the addr/command/parameter,
            // if the cmd is Set then the next entity is a value
            // this value MUST be a single float
            // which takes the next four bytes of buffer (followed by STOPBYTE)
            union Flyte setval;
            int i;
            for(i = 0; i < 4; ++i) { setval.bytes[i] = buffer[i+1]; }

            if(verbose) {
                UARTprintf("Set val: ");
                UARTPrintFloat(setval.f, false);
            }
        } else {}
    }

    return true;
}

/**
 * Handles a UART interrupt
 */
void UARTIntHandler(void) {
    uint32_t ui32Status;
    // get the interrupt status
    ui32Status = ROM_UARTIntStatus(UART7_BASE, true);
    // clear the asserted interrupts
    ROM_UARTIntClear(UART7_BASE, ui32Status);
    // initialize recv buffer
    char recv[10] = "";
    uint32_t ind = 0;
    char curr = ROM_UARTCharGet(UART7_BASE);
    // loop while there are bytes in receive FIFO
    while(curr != STOPBYTE && ind < 10) {
        recv[ind] = curr;
        curr = ROM_UARTCharGet(UART7_BASE);
        ++ind;
    }

    // keep stop byte for tokenizing
    recv[ind] = curr;
    ++ind;

    /*uint8_t crcin = recv[ind-1];
    if(crc8(0, (uint8_t *)recv, ind - 1) != crcin){
        // ********** ERROR ***********
        // Handle corrupted message
    }*/

    handleUART(recv, ind, true, true);
}

/**
 * Checks if UART is ready to send
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
