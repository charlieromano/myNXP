#ifndef MAIN_H_
#define MAIN_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "hello_task.h"

void BOARD_InitHardware(void);

#endif /* MAIN_H_ */
