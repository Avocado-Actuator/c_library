# C Library

This repo holds code for a master controller communicating with avocado actuators. The [`export`](https://github.com/avocado-actuator/c_library/tree/export) branch contains the library in a pair of header and source files - `avocomms.c` & `avocomms.h`. Currently to use this library one should copy these two files to the root of their CCS project (see the [setup guide](https://github.com/Avocado-Actuator/embedded/blob/master/setup.md) in the `embedded` repo for project setup steps).

If you'd prefer to write raw messages yourself feel free to examine the protocol [here](https://github.com/Avocado-Actuator/embedded/blob/master/protocol.md).

## Minimal Usage

Copy over `avocomms.c` & `avocomms.h`, then update your `...startup_ccs.c` to enable the interrupts needed by our library. If you have no other interrupts feel free to copy over `tm4c1294ncpdt_startup_ccs.c` to your project, otherwise you'll specifically want to add external declarations for `UARTIntHandler`, `ConsoleIntHandler` & `Timer0IntHandler` (see lines 59-61 of our `...startup_ccs.c` file for a reference). Then make sure to add these interrupts in the appropriate locations in the vector table (again see `...startup_ccs.c` for guidance here).

You should now be able to communicate with your avocado! The first thing you'll want to do is set its address with the function `setAddress`. Once you've done that you can begin setting position, velocity, current & more.

Here is a minimal main function that moves the avocado to an angle of `60°`.

```c
int main(void) {
    // set clocking to run directly from the crystal at 120MHz
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN
            | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    ROM_IntMasterEnable(); // enable processor interrupts

    // initialize communication across UART
    CommsInit(g_ui32SysClock);
    // initialize timers for heartbeats
    TimerInit(g_ui32SysClock);
    // optional, only if you want debugging readouts
    ConsoleInit(); // initialize UART0 for debugging output using UARTStdio

    // set avocado's address to 1
    setAddress(1);
    // rotate avocado with address 1 (what we just set) to 60°
    rotateToPosition(1, 60);
}
```
