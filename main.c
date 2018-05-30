#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h" // NEW INCLUDE!!!!
#include "inc/hw_sysctl.h" // NEW INCLUDE!!!
#include "inc/hw_types.h" // NEW INCLUDE!!!
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#include "rs485.h"

// System clock rate in Hz.
uint32_t g_ui32SysClock;

//***************************************************************************
// The error routine that is called if the driver library encounters an error
//***************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif


//*****************************************************
// Initializes UART0 for console output using UARTStdio
//*****************************************************
void ConsoleInit(void) {
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
// SUPER IMPORTANT, CCS IS JANK, NEED TO INCREASE STACK SIZE MANUALLY IN ORDER
// TO USE SPRINTF, GO TO Build -> ARM Linker -> Basic Options IN PROJECT
// PROPERTIES SET C SYSTEM STACK SIZE TO 65536
//*****************************************************************************
int main(void) {
    ConsoleInit(); // initialized UART0 for console output using UARTStdio
    UARTprintf("\n\nTiva has turned on...\n");
    // Set the clocking to run directly from the crystal at 120MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN
            | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    ROM_IntMasterEnable(); // enable processor interrupts.
    RSInit(g_ui32SysClock); // Initialize the RS485 link
    ConsoleInit(); // initialized UART0 for console output using UARTStdio
    //UARTSend((uint8_t *)"\033[2JTiva has turned on\n\r", 24);

    UARTprintf("Initializing...\n");

    int counter = 0;
    while(1) {
        // Check the busy flag in the uart7 register. If not busy, set transceiver pin low
        if (UARTReady()){
            UARTSetRead();
        }

        if(counter == 0) {
            setAddress(1);
            setMaxCurrent(1, 5.0);
            rotateToPosition(1, 7.0);
        }

        ++counter;
    }
}
