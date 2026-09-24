#include "app_freertos_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "com_debug.h"
#include "pwr_mgmt.h"
#define PWR_TASK_STACK_SIZE 128
#define PWR_TASK_PRIORITY 4

TaskHandle_t xPWRTaskHandle = NULL;

void vPWRTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    PWR_MGMT_Init();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000)); // Delay for 10 seconds
        PWR_MGMT_Start();
    }
}

void App_task_start(void)
{
    // Create Task
    xTaskCreate(vPWRTask, "PWRTask", 
        PWR_TASK_STACK_SIZE, NULL, 
        PWR_TASK_PRIORITY, &xPWRTaskHandle);
    // Start the scheduler
    vTaskStartScheduler();
}