#include "usart.h"
#include "stdio.h"

/* syscalls.c routes stdout to this hook. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
    return ch;
}
