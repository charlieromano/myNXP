/*****************************************************************************
 * Copyright (c) 2025, Carlos German Carreño Romano <charlieromano@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2022/08/28
 * Version: 1.0
 ****************************************************************************/

 #include "main.h"
 
 int main(void)
 {
     BOARD_InitHardware();
     if (xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL) != pdPASS)
     {
         PRINTF("Task creation failed!.\r\n");
         while (1);
     }
     vTaskStartScheduler();
     for (;;);
 }
 