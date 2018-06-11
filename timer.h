#ifndef timer_H_
#define timer_H_

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"

#include "comms.h"

void TimerInit(uint32_t);
void Timer0IntHandler(void);

#endif /* timer_H_ */
