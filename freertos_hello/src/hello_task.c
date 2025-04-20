#include "hello_task.h"

TimerHandle_t timerHandleAB;
QueueHandle_t queueHandle_AB;

void hello_task(void *pvParameters)
{
    PRINTF("Hello from FreeRTOS task!\r\n");
    //vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelete(NULL);
}

void timerCallbackAB(TimerHandle_t xTimerHandle){

    PRINTF("Timer!\r\n");

    static uint8_t cnt = 0;
    cnt++;
    eSystemEvent_AB data_AB = cnt%4;
    if(xQueueSend(queueHandle_AB, &data_AB, 0U)!=pdPASS){
          perror("Error sending data to the queueHandle_AB from timer\r\n");
    }
}

      
void vTaskAB(void *xTimerHandle)
{
    (void)xTimerHandle;
    PRINTF("vTaskAB!\r\n");
 
    /*
    if (pdTRUE == xSemaphoreTake( xMutexUART, portMAX_DELAY)){
       vPrintString("Task AB is running.\r\n");
       xSemaphoreGive(xMutexUART);
    }
   */ 
 
    while(true){

        // fsmMachineAB init
        eSystemEvent_AB newEvent = evInit_AB;
        eSystemState_AB nextState = STATE_INIT_AB;
        fsmMachineAB[nextState].fsmEvent = newEvent; 
        nextState = (*fsmMachineAB[nextState].fsmHandler)();
        
        // Active object
        while(true){
            if( pdPASS == xQueueReceive(queueHandle_AB, &newEvent, portMAX_DELAY)){
                fsmMachineAB[nextState].fsmEvent = newEvent;
                nextState = (*fsmMachineAB[nextState].fsmHandler)();
            }
        }
       //vPrintString("This task is running and about to delete itself.\r\n");
       //vTaskDelete(xTaskStateMachineHandler);
    }
}
 