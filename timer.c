#include "timer.h"

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
