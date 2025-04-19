#ifndef MAIN_H_
#define MAIN_H_

#include "FreeRTOS.h"
#include "task.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "hello_task.h"

// Declare the task from another file
void hello_task(void *pvParameters);

#endif /* MAIN_H_ */
