#ifndef MAIN_H_
#define MAIN_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "mainTasks.h"
#include "statemachine_AB.h"

void BOARD_InitHardware(void);

#endif /* MAIN_H_ */
