#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_

#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "statemachine_AB.h"

void hello_task(void *pvParameters);
void timerCallbackAB(TimerHandle_t xTimerHandle);
void vTaskAB(void *xTimerHandle);


extern TimerHandle_t timerHandleAB;
extern QueueHandle_t queueHandle_AB;

extern sStateMachine_AB fsmMachineAB[];

#endif /* MAIN_TASKS_H_ */
