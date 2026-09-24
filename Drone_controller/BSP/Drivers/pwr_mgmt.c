#include "stm32f1xx_hal.h"
#include "pwr_mgmt.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
 
void PWR_MGMT_Init(void)
{
    HAL_GPIO_WritePin(Power_Key_GPIO_Port, Power_Key_Pin, GPIO_PIN_SET);
}

void PWR_MGMT_Start(void)
{
    HAL_GPIO_WritePin(Power_Key_GPIO_Port, Power_Key_Pin, GPIO_PIN_RESET);
    vTaskDelay(100); // Delay for 100 milliseconds
    HAL_GPIO_WritePin(Power_Key_GPIO_Port, Power_Key_Pin, GPIO_PIN_SET);
}