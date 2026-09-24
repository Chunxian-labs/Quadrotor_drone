#include "app_freertos_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "com_debug.h"
#define TASK1_STACK_SIZE 128
#define TASK1_PRIORITY 1
#define TASK2_STACK_SIZE 128
#define TASK2_PRIORITY 1
TaskHandle_t xTask1Handle = NULL;
TaskHandle_t xTask2Handle = NULL;
void vTask1(void *pvParameters)
{
    while (1)
    {
        debug_printf("Task 1 is running.\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ms
    }
}
void vTask2(void *pvParameters)
{
    while (1)
    {
        debug_printf("Task 2 is running.\r\n");
        vTaskDelay(pdMS_TO_TICKS(900)); // Delay for 900 ms
    }
}
void App_task_start(void)
{
    // Create Task
    xTaskCreate(vTask1, "Task1", 
        TASK1_STACK_SIZE, NULL, 
        TASK1_PRIORITY, &xTask1Handle);
    xTaskCreate(vTask2, "Task2", 
        TASK2_STACK_SIZE, NULL, 
        TASK2_PRIORITY, &xTask2Handle);
    // Start the scheduler
    vTaskStartScheduler();
}