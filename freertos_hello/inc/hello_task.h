#ifndef HELLO_TASK_H_
#define HELLO_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "fsl_debug_console.h"
#include "board.h"
#include <stdint.h>
#include <stdio.h>

void hello_task(void *pvParameters);
void timerCallbackAB(TimerHandle_t xTimerHandle);

extern TimerHandle_t timerHandleAB;

#endif /* HELLO_TASK_H_ */
