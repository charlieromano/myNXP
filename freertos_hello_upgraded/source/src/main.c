/*****************************************************************************
 * Copyright (c) 2025, Carlos German Carreño Romano <charlieromano@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2025/04/19
 * Version: 1.0
 ****************************************************************************/

 #include "../inc/main.h"
 
 TaskHandle_t helloTaskHandle = NULL;
 TaskHandle_t xTaskStateMachineHandler_AB = NULL;

 void BOARD_InitLEDs(void);

 int main(void)
 {
     BOARD_InitHardware();

     if (xTaskCreate(hello_task, "Hello_task", 
      configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY+2, 
      &helloTaskHandle)
      != pdPASS)
     {
         PRINTF("Task creation failed!.\r\n");
         while (1);
     }

     /***************************************************************************/
     /* State Machine AB tasks */
     
     /* Create the timer */
     if( (timerHandleAB = xTimerCreate( "TimerAB", 1000, true, NULL, 
        timerCallbackAB)) == NULL ) {
            perror("Error creating timer");
            return 1;
        }

     /* Start the timer */
     if(xTimerStart(timerHandleAB, 0) != pdPASS){
        perror("Error starting timer");
        return 1;
        }

     // Create the queue
      queueHandle_AB = xQueueCreate(QUEUE_MAX_LENGTH, sizeof(eSystemEvent_AB));
      if (queueHandle_AB == NULL){
         perror("Error creating queue");
         return 1;
      }

      // Create the task 
      if( xTaskCreate( vTaskAB, "State Machine using active object", 
         configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY+2, 
         &xTaskStateMachineHandler_AB) 
         == pdFAIL ) {
         perror("Error creating task");
         return 1;
      }

     /***********************************************************************/
     vTaskStartScheduler();
     while(true);
 }
 

 void BOARD_InitLEDs(void) {
   // Clock enablement (usually handled in pin_mux.c)
   CLOCK_EnableClock(kCLOCK_PortA);
   CLOCK_EnableClock(kCLOCK_GpioA);

   // GPIO configuration
   gpio_pin_config_t led_config = {
       .pinDirection = kGPIO_DigitalOutput,
       .outputLogic = 1U  // Start OFF (active low)
   };

   GPIO_PinInit(GPIOA, 19U, &led_config);  // GREEN
   GPIO_PinInit(GPIOA, 20U, &led_config);  // BLUE
   GPIO_PinInit(GPIOA, 21U, &led_config);  // RED
}

 