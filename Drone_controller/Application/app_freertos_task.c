#include "app_freertos_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "com_debug.h"
#include "pwr_mgmt.h"
#define PWR_TASK_STACK_SIZE 128
#define TASK1_PRIORITY 4

TaskHandle_t xPWR_TaskHandle = NULL;

void vPWR_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    PWR_MGMT_Init();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000)); // Delay for 10 seconds
        PWR_MGMT_Start();
    }
}


void APP_task_start(void)
{
        // Create Task
    xTaskCreate(vPWR_Task, "PWR_Task", 
        PWR_TASK_STACK_SIZE, NULL, 
        TASK1_PRIORITY, &xPWR_TaskHandle);

    // Start the scheduler
    vTaskStartScheduler();
}