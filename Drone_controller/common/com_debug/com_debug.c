#include "usart.h"
#include "com_debug.h"
#include "stdio.h"

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}