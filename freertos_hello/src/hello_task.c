#include "hello_task.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

void hello_task(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello world from FreeRTOS task!\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
