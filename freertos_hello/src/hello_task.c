#include "hello_task.h"

TimerHandle_t timerHandleAB;

void hello_task(void *pvParameters)
{
    PRINTF("Hello from FreeRTOS task!\r\n");
    //vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelete(NULL);
}

void timerCallbackAB(TimerHandle_t xTimerHandle){

    PRINTF("Timer!\r\n");
/* 
    static uint8_t cnt = 0;
    cnt++;
    eSystemEvent_AB data_AB = cnt%4;
    if(xQueueSend(queueHandle_AB, &data_AB, 0U)!=pdPASS){
          perror("Error sending data to the queueHandle_AB from timer\r\n");
    }
*/
}